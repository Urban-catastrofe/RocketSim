//! Lateral wheel friction measurement (`RLLATDUMP=1`).
//!
//! The same trick as [`super::suspdump`], on a different axis. On a flat floor
//! with an upright car, the body right axis projected into the contact plane
//! carries *only* lateral wheel friction:
//!
//! - gravity and the sticky force act along the contact normal (the sticky
//!   direction is the normalised sum of the wheel contact normals, so on a flat
//!   floor it is exactly `+z`), which is perpendicular to the axle;
//! - `calc_friction_impulses` builds `forward_dir = normal x axle_dir`, so the
//!   engine force, the brake and the whole longitudinal friction term are
//!   exactly perpendicular to the axle - provided the front and back axles
//!   coincide, which needs `steer_angle == 0`;
//! - boost acts along body forward, which is *not* quite perpendicular once the
//!   car is rolled, so it is subtracted explicitly;
//! - the car body has zero linear damping; only the ball is given drag.
//!
//! What is left is an absolute measurement of RL lateral friction impulse:
//!
//! ```text
//! dv_lat = (vel[i+1] - vel[i]) . axle - boost_dv * (forward . axle)
//! ```
//!
//! and the sim model for it reduces to a *two-parameter* line, because
//! `curves::LAT_FRICTION` is a two-point curve `[(0, 1.0), (1, 0.2)]`:
//!
//! ```text
//! dv_lat = sum_w q_w * mu(x_w),   mu(x) = a + b * x   (shipped: a = 1, b = -0.8)
//! q_w    = BT_TO_UU * dt / 3 * side_impulse_w
//! ```
//!
//! `side_impulse_w` is `resolve_single_bilateral` along the axle against the
//! static floor, reproduced from the pose; the `dt / 3` and the `BT_TO_UU` come
//! from `friction_scale = mass / 3` cancelling against the `massed` divide in
//! `apply_friction_impulses`. So the curve is recoverable by regressing the
//! measured `dv_lat` on `(sum q_w, sum q_w x_w)` - and because the curve has
//! only two points, that is the whole model, not an approximation to it.
//!
//! **The coefficient lags the pose by one tick.** `calc_friction_impulses` runs
//! inside `update_vehicle_first`, but `wheel.lat_friction` and
//! `wheel.steer_angle` are written by `Car::update_wheels`, which runs *after*
//! it. So the impulse applied on step `i -> i+1` uses `side_impulse` from the
//! fresh tick-`i` pose but a friction coefficient computed from the tick-`i-1`
//! pose. Rows therefore carry `x` from both ticks, so the fit can decide which
//! alignment RL actually uses rather than assuming the port is right. It also
//! means the pass must step every tick: `lat_friction` is internal vehicle
//! state that `set_state_to_record_tick` does not restore, so the lag is only
//! faithful when the previous tick was really stepped (the same reason
//! `suspdump` steps every tick, and the sharded runner needs warmup ticks).
//!
//! Each row is one usable step:
//!
//! ```text
//! LATROW <recording> <tick> <car> <handbrake_val>
//!        (<q> <x_prev> <x_cur> <x_logged>) x4 <rl_dv> <sim_dv>
//! ```
//!
//! `x_logged` is RL own `friction_curve_input` at the lagging tick - one of the
//! few observer wheel fields that is actually populated - which makes the
//! pose-derived `x` checkable against ground truth rather than merely assumed.
//!
//! The handbrake scales `lat_friction` by `1 + (0.1 - 1) * handbrake_val`, which
//! is independent of `x` and so factors straight out of the sum. That factor is
//! how this pass found the harness defect that
//! `normalize::normalize_handbrake_val` now fixes: the observer logs
//! `handbrake_val` snapped to 0/1, and restoring that snapped value was worth
//! 17% of the whole suite error.

use glam::{Mat3A, Vec3A};
use rocketsim::consts::{BT_TO_UU, TICK_TIME, UU_TO_BT};
use rocketsim::{CarBodyConfig, CarControls};

use super::flat_floor::{self, BOOST_GROUND_DV, MASS, grounded, isolated, wheel_state};
use super::recording::Recording;
use super::recording::cpp_records::{CarRecord, PhysRecord};
use super::runner::{has_discontinuity, make_arena, set_state_to_record_tick};

/// The contact damping hard-coded into `resolve_single_bilateral`.
const CONTACT_DAMPING: f32 = -0.2;
/// `collision_margin::CONVEX_DISTANCE_MARGIN`, which is not public.
const CONVEX_MARGIN: f32 = 0.04;
/// Below this lateral slip speed (uu/s) `update_wheels` pins the curve input to
/// zero outright.
const SLIP_DEADZONE: f32 = 5.0;

/// Per wheel, the coefficient `q` that multiplies the friction curve, and the
/// curve input from the previous tick, from this tick, and as Rocket League
/// logged it. See the row format in the module docs.
type WheelTerms = [(f32, f32, f32, f32); 4];

/// Local inverse inertia of the Octane chassis, BT, reproducing
/// `BoxShape::calculate_local_intertia` on the hitbox built in `Car::new`.
fn inv_inertia_local() -> Vec3A {
    let hb = CarBodyConfig::OCTANE.hitbox_size;
    let half = Vec3A::new(hb.x, hb.y, hb.z) * 0.5 * UU_TO_BT;
    // `BoxShape::new` shrinks the implicit box by the full convex margin and
    // then adds back a *clamped* safe margin, so the two do not cancel.
    let margin = (0.1 * half.min_element()).min(CONVEX_MARGIN);
    let l = (half - CONVEX_MARGIN + margin) * 2.0;
    let inertia = MASS / 12.0
        * Vec3A::new(
            l.y * l.y + l.z * l.z,
            l.x * l.x + l.z * l.z,
            l.x * l.x + l.y * l.y,
        );
    Vec3A::new(1.0 / inertia.x, 1.0 / inertia.y, 1.0 / inertia.z)
}

/// True when every wheel reports the floor normal *exactly*, which is what makes
/// the lateral axis exactly rather than approximately closed: the four axle
/// directions then coincide, `NON_STICKY_FRICTION_FACTOR` is exactly 1, and the
/// longitudinal terms cancel to the bit. 99.9% of flat-floor steps in the suite
/// qualify, so the extra strictness is nearly free.
fn exactly_flat(car: &CarRecord) -> bool {
    car.wheels.iter().all(|w| {
        w.contact_normal.x == 0.0 && w.contact_normal.y == 0.0 && w.contact_normal.z == 1.0
    })
}

/// The axle direction as `calc_friction_impulses` builds it: body right,
/// projected into the contact plane and normalised. Only valid at
/// `steer_angle == 0`, where the front pair matches the back pair.
fn axle_dir(rot: &Mat3A) -> Vec3A {
    let right = rot.y_axis;
    (right - Vec3A::Z * right.z).normalize()
}

/// `resolve_single_bilateral` along the axle against the static floor.
fn side_impulse(phys: &PhysRecord, rot: &Mat3A, w: usize, axle: Vec3A, inv_inertia: Vec3A) -> f32 {
    let up = rot.z_axis;
    let hp = flat_floor::hard_point(phys, rot, w);
    let contact = hp - up * flat_floor::trace(hp, up);

    let rel_pos = (contact - Vec3A::from(phys.pos)) * UU_TO_BT;
    let vel = Vec3A::from(phys.lin_vel) * UU_TO_BT;
    let point_vel = vel + Vec3A::from(phys.ang_vel).cross(rel_pos);

    // `get_jacobian_diagonal`, where the static floor contributes nothing
    // because its inverse mass and inverse inertia are both zero.
    let aj = rot.transpose() * rel_pos.cross(axle);
    let jac_diag = 1.0 / MASS + (inv_inertia * aj).dot(aj);

    CONTACT_DAMPING * axle.dot(point_vel) / jac_diag
}

/// The `friction_curve_input` of `Car::update_wheels` for one wheel: the share
/// of the hardpoint slip that is lateral. Uses the *unprojected* axle and the
/// velocity at the hardpoint rather than the contact point, as the sim does.
fn curve_input(phys: &PhysRecord, rot: &Mat3A, w: usize) -> f32 {
    let axle_raw = rot.y_axis;
    let hp = flat_floor::hard_point(phys, rot, w);
    let delta = hp - Vec3A::from(phys.pos);
    let slip = Vec3A::from(phys.ang_vel).cross(delta) + Vec3A::from(phys.lin_vel);

    let lateral = slip.dot(axle_raw).abs();
    if lateral <= SLIP_DEADZONE {
        return 0.0;
    }
    let long_dir = axle_raw.cross(Vec3A::Z);
    lateral / (slip.dot(long_dir).abs() + lateral)
}

/// Per-wheel `(q, x_prev, x_cur, x_logged)` for a measurable step, or `None`.
fn lateral_terms(
    prev: &CarRecord,
    from: &CarRecord,
    to: &CarRecord,
    inv_inertia: Vec3A,
) -> Option<WheelTerms> {
    // The suspension gate also proves the wheels sit on the floor plane at
    // z = 0, which the contact point below depends on.
    wheel_state(from, to)?;
    wheel_state(prev, from)?;
    if !exactly_flat(prev) || !exactly_flat(from) {
        return None;
    }
    // The `steer_angle` used to build the axle on this step was set one tick
    // earlier, and the one behind that shaped the lagging curve input.
    if from.prev_controls.steer != 0.0 || prev.prev_controls.steer != 0.0 {
        return None;
    }
    // The boost correction below needs an unambiguous flag for the step.
    if from.is_boosting != to.is_boosting {
        return None;
    }

    let rot_from = flat_floor::rot_of(&from.phys);
    let rot_prev = flat_floor::rot_of(&prev.phys);
    let axle = axle_dir(&rot_from);

    let mut out: WheelTerms = [(0.0, 0.0, 0.0, 0.0); 4];
    for (w, slot) in out.iter_mut().enumerate() {
        let impulse = side_impulse(&from.phys, &rot_from, w, axle, inv_inertia);
        *slot = (
            BT_TO_UU * TICK_TIME / 3.0 * impulse,
            curve_input(&prev.phys, &rot_prev, w),
            curve_input(&from.phys, &rot_from, w),
            prev.wheels[w].friction_curve_input,
        );
    }
    Some(out)
}

pub fn analyze(recording: &Recording) {
    let num_cars = recording.info.num_cars as usize;
    if num_cars == 0 {
        return;
    }
    let stride = recording.stride;
    let inv_inertia = inv_inertia_local();
    let (mut arena, car_idcs) = make_arena(num_cars);
    let mut controls_buf: Vec<CarControls> = vec![CarControls::DEFAULT; num_cars];

    let last = recording.ticks.len().saturating_sub(stride + 1);
    for i in (0..=last).step_by(stride) {
        let from_tick = &recording.ticks[i];
        let to_tick = &recording.ticks[i + stride];

        // A measurable step needs the tick before it as well, for the lagging
        // friction coefficient, and needs that tick to run continuously into it.
        let have_prev = i >= stride
            && !has_discontinuity(recording, i - stride, stride)
            && !has_discontinuity(recording, i, stride);
        let states: Vec<Option<WheelTerms>> = (0..num_cars)
            .map(|j| {
                if !have_prev {
                    return None;
                }
                let prev_tick = &recording.ticks[i - stride];
                let prev = &prev_tick.car_records[j];
                let (from, to) = (&from_tick.car_records[j], &to_tick.car_records[j]);
                if !grounded(prev, from) || !grounded(from, to) {
                    return None;
                }
                let here = Vec3A::from(from.phys.pos);
                if !isolated(prev_tick, j, here)
                    || !isolated(from_tick, j, here)
                    || !isolated(to_tick, j, here)
                {
                    return None;
                }
                lateral_terms(prev, from, to, inv_inertia)
            })
            .collect();

        // Step every tick regardless - see the module docs on the one-tick lag.
        for (j, car_record) in to_tick.car_records.iter().enumerate() {
            controls_buf[j] = car_record.prev_controls.into();
        }
        set_state_to_record_tick(
            &mut arena,
            &car_idcs,
            from_tick,
            i.checked_sub(stride).map(|i| &recording.ticks[i]),
            &controls_buf,
        );
        arena.step_tick();

        for (j, &car_idx) in car_idcs.iter().enumerate() {
            let Some(terms) = states[j] else { continue };
            let (from, to) = (&from_tick.car_records[j], &to_tick.car_records[j]);
            // The coefficient was computed one tick earlier, so this is the
            // handbrake state that scaled it. `normalize_handbrake_val` has
            // already rebuilt the ramp the observer snapped away.
            let prev_hb = recording.ticks[i - stride].car_records[j].handbrake_val;
            let rot_from = flat_floor::rot_of(&from.phys);
            let axle = axle_dir(&rot_from);

            let mut offset = 0.0;
            if from.is_boosting {
                offset -= BOOST_GROUND_DV * rot_from.x_axis.dot(axle);
            }
            let before = Vec3A::from(from.phys.lin_vel);
            let rl_dv = (Vec3A::from(to.phys.lin_vel) - before).dot(axle) + offset;
            let sim_vel = arena.get_car_state(car_idx).phys.vel;
            let sim_dv = (Vec3A::new(sim_vel.x, sim_vel.y, sim_vel.z) - before).dot(axle) + offset;

            // One `println!` per row: the harness runs recordings on several
            // threads, and a partially written line lets another thread output
            // splice into the middle of this one.
            use std::fmt::Write as _;
            let mut row = format!("LATROW {} {i} {j}", recording.name);
            let _ = write!(row, " {prev_hb:.6}");
            for (q, x_prev, x_cur, x_log) in terms {
                let _ = write!(row, " {q:.6} {x_prev:.6} {x_cur:.6} {x_log:.6}");
            }
            let _ = write!(row, " {rl_dv:.6} {sim_dv:.6}");
            println!("{row}");
        }
    }
}
