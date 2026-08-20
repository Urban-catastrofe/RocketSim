//! Suspension force measurement (`RLSUSPDUMP=1`).
//!
//! On a flat floor with an upright car and its wheels down, the *vertical*
//! equation of motion is closed and scalar. The only vertical impulses are
//! gravity, the sticky force, boost (through `forward.z`) and the suspension,
//! because the wheel friction impulse lies in the contact plane and so has no
//! vertical component. That makes Rocket League's own suspension impulse for a
//! step recoverable from the recording alone, with no simulation involved:
//!
//! ```text
//! dv_susp = (vel[i+1].z - vel[i].z) + 1.5 * g * dt - boost_z
//! ```
//!
//! (1.5 g because the sticky force adds half a gravity on the floor.) The same
//! expression applied to the sim's post-step velocity gives the sim's
//! suspension impulse, so the two are directly comparable — and unlike the
//! `RLCENSUS` banding, this is an *absolute* measurement of RL's suspension
//! force rather than a "required multiplier" read off a probe sweep. Rebuilding
//! `SUSPENSION_STIFFNESS` / `WHEELS_DAMPING_*` from it needs no probe at all.
//!
//! Compression and rate come from the pose in closed form: the wheel ray leaves
//! the hardpoint along `-up` and meets the floor plane `z = 0` at a distance
//! `hard_point.z / up.z`. That is deliberate rather than reading the logged
//! `susp_length`, whose tick alignment relative to the pose is not resolvable
//! from the recordings (its own lag test is a coin flip), and pairing the wrong
//! tick would corrupt exactly the fast-rate steps this is here to measure. The
//! logged block is used only as a gate.
//!
//! Each row is one usable step:
//!
//! ```text
//! SUSPROW <recording> <tick> <car> (<compression_uu> <rate_uu_per_s>) x4 <rl_dv> <sim_dv>
//! ```
//!
//! Compression is positive when the suspension is compressed; rate is positive
//! when it is extending. Rows are per recording, so aggregating across the suite
//! is an external step, as it is for `RLCENSUS`.
//!
//! To rebuild a coefficient from the rows, regress `rl_dv` on the sim's own
//! basis, which follows `WheelInfo::update_suspension` term for term (uu/s of
//! velocity change, `m` = 180, `scale` = `SUSPENSION_FORCE_SCALE_FRONT`/`_BACK`):
//!
//! ```text
//! spring_w = SUSPENSION_STIFFNESS / (120 * m) * compression_w * scale_w
//! damp_w   = -D_branch / (120 * m) * rate_w * scale_w      // branch on sign(rate)
//! push_w   = extra_pushback_w / m                           // only if compression > 2.5
//! ```
//!
//! summed over wheels, with each wheel's `spring_w + damp_w` clamped at zero
//! (which drops that wheel's pushback too, as the sim does). Fit it *trimmed*:
//! the residual rms sits two orders of magnitude above the median residual, so
//! plain least squares reports the handful of landing impacts rather than the
//! bulk. Results and conclusions are in `how-to-test.md`.

use glam::{Mat3A, Vec3A};
use rocketsim::CarControls;

use super::recording::Recording;
use super::recording::cpp_records::{CarRecord, PhysRecord};
use super::runner::{has_discontinuity, is_car_sentinel, make_arena, set_state_to_record_tick};

/// A contact normal at least this vertical counts as the flat floor.
const FLAT_NORMAL_Z: f32 = 0.999;
/// A car tilted less than this counts as upright.
const UPRIGHT_UP_Z: f32 = 0.999;
/// Ball or car nearer than this and the vertical bookkeeping is not closed.
const CLEARANCE: f32 = 350.0;
/// Gravity plus the floor sticky force, as a velocity change per tick (uu/s).
const GRAV_STICKY_DV: f32 = 1.5 * 650.0 / 120.0;
const BOOST_GROUND_DV: f32 = (2975.0 / 3.0) / 120.0;
const MAX_TRAVEL: f32 = 12.0;

/// Octane wheel geometry, uu: `(offset_x, offset_y, offset_z, radius, rest1)`,
/// where `rest1` is the configured rest length minus `MAX_SUSPENSION_TRAVEL`,
/// matching `WheelInfo::suspension_rest_length_1`. Wheels 0/1 are the front
/// pair. The y sign is irrelevant here: this pass only ever sums over a pair.
const WHEEL_GEO: [(f32, f32, f32, f32, f32); 4] = [
    (51.25, 25.90, 20.755, 12.5, 26.755),
    (51.25, -25.90, 20.755, 12.5, 26.755),
    (-33.75, -29.50, 20.755, 15.0, 25.055),
    (-33.75, 29.50, 20.755, 15.0, 25.055),
];

fn rot_of(phys: &PhysRecord) -> Mat3A {
    Mat3A::from_cols(
        phys.rot.rows[0].into(),
        phys.rot.rows[1].into(),
        phys.rot.rows[2].into(),
    )
}

fn four_flat(car: &CarRecord) -> bool {
    car.wheels
        .iter()
        .all(|w| w.has_contact && w.contact_normal.z > FLAT_NORMAL_Z)
}

/// Per-wheel `(compression_uu, rate_uu_per_s)` from the pose, or `None` when
/// the flat-floor model does not hold for this step.
fn wheel_state(from: &CarRecord, to: &CarRecord) -> Option<[(f32, f32); 4]> {
    let phys = &from.phys;
    let rot = rot_of(phys);
    let up = rot.z_axis;
    if up.z <= UPRIGHT_UP_Z {
        return None;
    }
    let pos = Vec3A::from(phys.pos);
    let vel = Vec3A::from(phys.lin_vel);
    let ang_vel = Vec3A::from(phys.ang_vel);

    let mut out = [(0.0f32, 0.0f32); 4];
    for (w, &(ox, oy, oz, radius, rest1)) in WHEEL_GEO.iter().enumerate() {
        let hard_point = pos + rot * Vec3A::new(ox, oy, oz);
        let trace = hard_point.z / up.z;
        let length = trace - radius - rest1;
        if length > MAX_TRAVEL - 0.05 {
            return None; // the log says touching, the geometry says clear
        }
        let rel = hard_point - pos - up * trace;
        let rate = (vel + ang_vel.cross(rel)).z / up.z;
        // The pose-derived length assumes the contact surface is the floor plane
        // at z = 0. A car on a corner ramp reports a near-vertical normal on a
        // surface that is *rising*, which would fake a fast-extending
        // suspension, so require the logged length to agree - allowing one tick
        // of travel, since the block's own tick alignment is not resolvable.
        let tol = 0.25 + rate.abs() / 120.0 * 1.5;
        let agrees = (length - from.wheels[w].susp_length).abs() < tol
            || (length - to.wheels[w].susp_length).abs() < tol;
        if !agrees {
            return None;
        }
        out[w] = (-length, rate);
    }
    Some(out)
}

pub fn analyze(recording: &Recording) {
    let num_cars = recording.info.num_cars as usize;
    if num_cars == 0 {
        return;
    }
    let stride = recording.stride;
    let (mut arena, car_idcs) = make_arena(num_cars);
    let mut controls_buf: Vec<CarControls> = vec![CarControls::DEFAULT; num_cars];

    let last = recording.ticks.len().saturating_sub(stride + 1);
    for i in (0..=last).step_by(stride) {
        if has_discontinuity(recording, i, stride) {
            continue;
        }
        let from_tick = &recording.ticks[i];
        let to_tick = &recording.ticks[i + stride];

        let candidate = |j: usize| -> Option<[(f32, f32); 4]> {
            let (from, to) = (&from_tick.car_records[j], &to_tick.car_records[j]);
            if from.is_demoed || to.is_demoed || is_car_sentinel(&to.phys) {
                return None;
            }
            if from.is_jumping || to.is_jumping || from.is_flipping || to.is_flipping {
                return None;
            }
            if from.prev_controls.jump || to.prev_controls.jump {
                return None;
            }
            if !from.is_on_ground || !four_flat(from) || !four_flat(to) {
                return None;
            }
            let here = Vec3A::from(from.phys.pos);
            for tick in [from_tick, to_tick] {
                let ball = Vec3A::from(tick.ball_record.pos);
                if tick.ball_record.pos[2] > -1000.0 && (ball - here).length() < CLEARANCE {
                    return None;
                }
                for (k, other) in tick.car_records.iter().enumerate() {
                    if k != j
                        && !other.is_demoed
                        && !is_car_sentinel(&other.phys)
                        && (Vec3A::from(other.phys.pos) - here).length() < CLEARANCE
                    {
                        return None;
                    }
                }
            }
            wheel_state(from, to)
        };

        let states: Vec<Option<[(f32, f32); 4]>> = (0..num_cars).map(candidate).collect();

        // Every tick is stepped, even one with no usable car. `set_state_to_record_tick`
        // restores car and ball state but not Bullet's persistent contact manifolds,
        // whose cached impulses are warm-started at `WARMSTARTING_FACTOR`; skipping
        // ticks leaves that cache describing a pose from hundreds of ticks ago and it
        // then fires into an unrelated state. An earlier version of this pass skipped
        // non-candidate ticks and reported a steady +49 uu/s of phantom suspension
        // force on cars alone in an empty half of the field. This is the same reason
        // the sharded runner needs `SHARD_WARMUP_TICKS`.
        for (j, car_record) in to_tick.car_records.iter().enumerate() {
            controls_buf[j] = car_record.prev_controls.into();
        }
        set_state_to_record_tick(&mut arena, &car_idcs, from_tick, &controls_buf);
        arena.step_tick();

        for (j, &car_idx) in car_idcs.iter().enumerate() {
            let Some(state) = states[j] else { continue };
            let (from, to) = (&from_tick.car_records[j], &to_tick.car_records[j]);
            let from_z = from.phys.lin_vel[2];
            let mut offset = GRAV_STICKY_DV;
            if from.is_boosting {
                offset -= BOOST_GROUND_DV * from.phys.rot.rows[0][2];
            }
            let rl_dv = to.phys.lin_vel[2] - from_z + offset;
            let sim_dv = arena.get_car_state(car_idx).phys.vel.z - from_z + offset;
            // One `println!` per row: the harness runs recordings on several
            // threads, and a partially written line lets another thread's
            // output splice into the middle of this one.
            use std::fmt::Write as _;
            let mut row = format!("SUSPROW {} {i} {j}", recording.name);
            for (c, rate) in state {
                let _ = write!(row, " {c:.5} {rate:.4}");
            }
            let _ = write!(row, " {rl_dv:.5} {sim_dv:.5}");
            println!("{row}");
        }
    }
}
