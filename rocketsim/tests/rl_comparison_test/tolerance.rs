//! Physics accuracy tolerances.
//!
//! These are the per-tick divergence budgets we hold the sim to when
//! reproducing a recorded Rocket League replay. They are the "angle to
//! chase": tighten them as the sim gets more accurate.
//!
//! Two gates exist:
//! - **hard** fields fail the test when their percentile exceeds the budget.
//! - **soft** fields are reported (and warned about) but never fail the test.
//!   Promote a soft field to hard once its baseline is understood.

/// Per-tick divergence budgets, in the field's native units.
#[derive(Debug, Clone, Copy)]
pub struct PhysicsTolerance {
    /// Position error budget in UU (1 UU ≈ 1 cm).
    pub pos: f32,
    /// Linear velocity error budget in UU/s.
    pub vel: f32,
    /// Angular velocity error budget in rad/s.
    pub ang_vel: f32,
    /// Rotation axis error budget (unit-vector chord distance, 0..=2).
    pub rot: f32,
    /// Percentile used for gating (0..=1). Spikes rarer than this are tolerated.
    pub percentile: f32,
}

impl Default for PhysicsTolerance {
    fn default() -> Self {
        Self {
            pos: 0.1,
            vel: 0.5,
            ang_vel: 0.5,
            rot: 0.02,
            percentile: 0.95,
        }
    }
}

impl PhysicsTolerance {
    /// Apply `RL_*_TOL` env overrides on top of the defaults.
    pub fn from_env(mut self) -> Self {
        if let Ok(v) = env_f32("RL_POS_TOL") {
            self.pos = v;
        }
        if let Ok(v) = env_f32("RL_VEL_TOL") {
            self.vel = v;
        }
        if let Ok(v) = env_f32("RL_ANGVEL_TOL") {
            self.ang_vel = v;
        }
        if let Ok(v) = env_f32("RL_ROT_TOL") {
            self.rot = v;
        }
        if let Ok(v) = env_f32("RL_PERCENTILE") {
            self.percentile = v.clamp(0.0, 1.0);
        }
        self
    }
}

pub fn env_f32(key: &str) -> Result<f32, ()> {
    std::env::var(key)
        .ok()
        .and_then(|s| s.trim().parse::<f32>().ok())
        .filter(|v| v.is_finite())
        .ok_or(())
}

pub fn env_usize(key: &str) -> Result<usize, ()> {
    std::env::var(key)
        .ok()
        .and_then(|s| s.trim().parse::<usize>().ok())
        .ok_or(())
}
