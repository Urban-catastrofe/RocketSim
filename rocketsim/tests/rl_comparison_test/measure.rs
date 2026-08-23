//! Lean per-tick measurement.
//!
//! Computes the raw sim-vs-recording deltas for one tick without allocating a
//! comparison map. This is the hot path: it runs once per tick per entity, so
//! on a 30-minute (216k tick) replay it must be cheap. The rich, allocating
//! [`super::compare`] path is reserved for deep-dive snapshots.

use glam::{DVec3, Vec3A};
use rocketsim::PhysState;

use super::recording::cpp_records::{CarRecord, PhysRecord};
use super::stats::{Field, Segment};

/// One field's error for one tick: magnitude + signed vector (for bias).
#[derive(Debug, Clone, Copy, Default)]
pub struct FieldSample {
    pub mag: f32,
    pub signed: DVec3,
}

/// All field errors for one entity on one tick, indexed in [`Field::ALL`] order.
#[derive(Debug, Clone, Copy, Default)]
pub struct PhysicsDelta {
    pub samples: [FieldSample; Field::ALL.len()],
}

impl PhysicsDelta {
    pub fn get(&self, field: Field) -> &FieldSample {
        &self.samples[field_index(field)]
    }
}

pub fn field_index(field: Field) -> usize {
    match field {
        Field::Pos => 0,
        Field::Vel => 1,
        Field::AngVel => 2,
        Field::RotFwd => 3,
        Field::RotUp => 4,
    }
}

fn to_dvec3(v: Vec3A) -> DVec3 {
    DVec3::new(v.x as f64, v.y as f64, v.z as f64)
}

fn sample(pred: Vec3A, real: Vec3A) -> FieldSample {
    let diff = pred - real;
    FieldSample {
        mag: diff.length(),
        signed: to_dvec3(diff),
    }
}

/// Compute all field deltas between a simulated state and a recorded state.
pub fn compute_delta(pred: &PhysState, real: &PhysRecord) -> PhysicsDelta {
    let mut d = PhysicsDelta::default();
    d.samples[field_index(Field::Pos)] = sample(pred.pos, real.pos.into());
    d.samples[field_index(Field::Vel)] = sample(pred.vel, real.lin_vel.into());
    d.samples[field_index(Field::AngVel)] = sample(pred.ang_vel, real.ang_vel.into());
    d.samples[field_index(Field::RotFwd)] = sample(pred.get_forward_dir(), real.rot.rows[0].into());
    d.samples[field_index(Field::RotUp)] = sample(pred.get_up_dir(), real.rot.rows[2].into());
    d
}

/// Invoke `f` for every breakdown segment the tick belongs to, derived from the
/// recording's own flags (the physical regime the step started in).
pub fn for_each_segment(car: &CarRecord, mut f: impl FnMut(Segment)) {
    if car.is_on_ground {
        f(Segment::Grounded);
    } else {
        f(Segment::Airborne);
    }
    if car.is_boosting {
        f(Segment::Boosting);
    }
    if car.is_jumping {
        f(Segment::Jumping);
    }
    if car.is_flipping {
        f(Segment::Flipping);
    }
    if car.is_supersonic {
        f(Segment::Supersonic);
    }
}
