//! Physics accuracy tolerances.
//!
//! These are the per-tick divergence budgets we hold the sim to when
//! reproducing a recorded Rocket League replay. They are the "angle to
//! chase": tighten them as the sim gets more accurate.
//!
//! The bar is **0.03 divergence on the worst single step** (`percentile = 1.0`)
//! for both hard-gated fields: 0.03 UU of position and 0.03 UU/s of velocity.
//! A case fails if any one entity, on any one tick, exceeds that. Loosen via
//! `RL_POS_TOL` / `RL_VEL_TOL` / `RL_PERCENTILE` when surveying rather than
//! gating.
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
    /// Percentile used for gating (0..=1). `1.0` gates on the **worst single
    /// step** — no spike is tolerated. Lower it to tolerate rare outliers.
    pub percentile: f32,
    /// Velocity error budget in UU/s for a button-triggered impulse, measured
    /// across the two-step window that brackets the press.
    ///
    /// Separate from [`Self::vel`] because it is a different measurement, not a
    /// looser one: the two steps either side of a jump/flip press carry an
    /// arbitrary share of the impulse set by a sub-frame phase the recording
    /// does not store, so only their *sum* is defined (see `impulse_window.rs`).
    /// The
    /// budget covers one whole impulse plus two steps of ordinary integration
    /// error, so it is wider than the per-step bar by roughly those two steps.
    pub impulse_vel: f32,
}

impl Default for PhysicsTolerance {
    fn default() -> Self {
        Self {
            // The accuracy bar: no single simulated step may diverge from the
            // recording by more than 0.03 in the field's native unit.
            pos: 0.03,
            vel: 0.03,
            ang_vel: 0.5,
            rot: 0.02,
            // Gate on the maximum, not a percentile: one bad step is a failure.
            percentile: 1.0,
            impulse_vel: 0.1,
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
        if let Ok(v) = env_f32("RL_IMPULSE_TOL") {
            self.impulse_vel = v;
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
