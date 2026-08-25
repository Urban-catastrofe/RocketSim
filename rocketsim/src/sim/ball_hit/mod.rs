//! Car-ball extra hit impulse — a self-contained, config-driven subsystem.
//!
//! The bullet contact machinery only *detects* car-ball contact; the extra
//! impulse (the "carry" that makes dribbling/touching feel right) is computed
//! here as a pure function of the hit geometry and a [`BallHitConfig`]. The
//! cadence at which the impulse may fire is a first-class [`HitCadence`] enum,
//! so the calibration loop can fit the cadence and the factor curve against
//! recordings without restructuring the physics.

mod config;
mod impulse;
mod state;

pub use config::{BallHitConfig, HitCadence};
pub use impulse::{HitContext, can_fire, can_fire_transient_follow_up, compute_impulse};
pub use state::BallHitState;
