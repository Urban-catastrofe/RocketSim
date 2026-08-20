//! Shared flat-floor gate for the absolute-measurement passes.
//!
//! `suspdump` and `latdump` both rest on the same observation: on a flat floor
//! with an upright car and its wheels down, one axis of the equation of motion
//! closes, so Rocket League's own impulse along that axis is recoverable from
//! the recording with no simulation involved. Both therefore need the same
//! population of steps - isolated, upright, four wheels on the floor - and the
//! same wheel geometry. Keeping that here stops the two passes from quietly
//! disagreeing about which steps are measurable.

use glam::{Mat3A, Vec3A};

use super::recording::cpp_records::{CarRecord, PhysRecord};
use super::recording::tick_record::TickRecord;
use super::runner::is_car_sentinel;

/// A contact normal at least this vertical counts as the flat floor.
pub const FLAT_NORMAL_Z: f32 = 0.999;
/// A car tilted less than this counts as upright.
pub const UPRIGHT_UP_Z: f32 = 0.999;
/// Ball or car nearer than this and the bookkeeping is not closed.
pub const CLEARANCE: f32 = 350.0;
/// `bullet_vehicle::MAX_SUSPENSION_TRAVEL`, uu.
pub const MAX_TRAVEL: f32 = 12.0;
/// `car::MASS_BT`.
pub const MASS: f32 = 180.0;
/// Boost acceleration on the ground as a velocity change per tick, uu/s.
pub const BOOST_GROUND_DV: f32 = (2975.0 / 3.0) / 120.0;

/// Octane wheel geometry, uu: `(offset_x, offset_y, offset_z, radius, rest1)`,
/// where `rest1` is the configured rest length minus `MAX_SUSPENSION_TRAVEL`,
/// matching `WheelInfo::suspension_rest_length_1`. Wheels 0/1 are the front
/// pair, and the y signs follow `Car::new`, which negates the configured
/// offset for the second wheel of each pair.
pub const WHEEL_GEO: [(f32, f32, f32, f32, f32); 4] = [
    (51.25, 25.90, 20.755, 12.5, 26.755),
    (51.25, -25.90, 20.755, 12.5, 26.755),
    (-33.75, -29.50, 20.755, 15.0, 25.055),
    (-33.75, 29.50, 20.755, 15.0, 25.055),
];

pub fn rot_of(phys: &PhysRecord) -> Mat3A {
    Mat3A::from_cols(
        phys.rot.rows[0].into(),
        phys.rot.rows[1].into(),
        phys.rot.rows[2].into(),
    )
}

/// World-space suspension hardpoint for one wheel, uu.
pub fn hard_point(phys: &PhysRecord, rot: &Mat3A, w: usize) -> Vec3A {
    let (ox, oy, oz, _, _) = WHEEL_GEO[w];
    Vec3A::from(phys.pos) + *rot * Vec3A::new(ox, oy, oz)
}

/// Distance from the hardpoint to the floor plane `z = 0` along `-up`, uu.
pub fn trace(hard_point: Vec3A, up: Vec3A) -> f32 {
    hard_point.z / up.z
}

fn four_flat(car: &CarRecord) -> bool {
    car.wheels
        .iter()
        .all(|w| w.has_contact && w.contact_normal.z > FLAT_NORMAL_Z)
}

/// True when the car is present and driving on all four wheels across the step,
/// with no jump or flip in flight to add an impulse of its own.
pub fn grounded(from: &CarRecord, to: &CarRecord) -> bool {
    if from.is_demoed || to.is_demoed || is_car_sentinel(&to.phys) {
        return false;
    }
    if from.is_jumping || to.is_jumping || from.is_flipping || to.is_flipping {
        return false;
    }
    if from.prev_controls.jump || to.prev_controls.jump {
        return false;
    }
    from.is_on_ground && four_flat(from) && four_flat(to)
}

/// True when no ball or other car in `tick` is close enough to car `j` to add an
/// unmodelled impulse.
pub fn isolated(tick: &TickRecord, j: usize, here: Vec3A) -> bool {
    let ball = Vec3A::from(tick.ball_record.pos);
    if tick.ball_record.pos[2] > -1000.0 && (ball - here).length() < CLEARANCE {
        return false;
    }
    !tick.car_records.iter().enumerate().any(|(k, other)| {
        k != j
            && !other.is_demoed
            && !is_car_sentinel(&other.phys)
            && (Vec3A::from(other.phys.pos) - here).length() < CLEARANCE
    })
}

/// Per-wheel `(compression_uu, rate_uu_per_s)` from the pose, or `None` when the
/// flat-floor model does not hold for this step.
///
/// Compression and rate come from the pose in closed form: the wheel ray leaves
/// the hardpoint along `-up` and meets the floor plane `z = 0` at
/// [`trace`]. That is deliberate rather than reading the logged `susp_length`,
/// whose tick alignment relative to the pose is not resolvable from the
/// recordings (its own lag test is a coin flip), and pairing the wrong tick
/// would corrupt exactly the fast-rate steps this is here to measure. The
/// logged block is used only as a gate.
pub fn wheel_state(from: &CarRecord, to: &CarRecord) -> Option<[(f32, f32); 4]> {
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
    for (w, &(_, _, _, radius, rest1)) in WHEEL_GEO.iter().enumerate() {
        let hp = hard_point(phys, &rot, w);
        let tr = trace(hp, up);
        let length = tr - radius - rest1;
        if length > MAX_TRAVEL - 0.05 {
            return None; // the log says touching, the geometry says clear
        }
        let rel = hp - pos - up * tr;
        let rate = (vel + ang_vel.cross(rel)).z / up.z;
        // The pose-derived length assumes the contact surface is the floor plane
        // at z = 0. A car on a corner ramp reports a near-vertical normal on a
        // surface that is *rising*, which would fake a fast-extending
        // suspension, so require the logged length to agree - allowing one tick
        // of travel, since the block tick alignment is not resolvable.
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
