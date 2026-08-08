//! Tunable constants for the car-ball extra hit impulse.
//!
//! These used to be scattered across `consts::ball::car_hit_impulse` and
//! `consts::curves::BALL_CAR_EXTRA_IMPULSE_FACTOR`. Grouping them here makes
//! the impulse a calibratable surface: run a residual analysis, nudge one of
//! these, re-run the 3v3 recordings, and watch the ball vel p95 move.

use crate::sim::linear_piece_curve::LinearPieceCurve;

/// How often the extra ball-hit impulse may fire while the ball stays in
/// contact with a car.
///
/// The `EveryTick` and `OncePerEpisode` variants are not exercised by the
/// default config (legacy `EveryOtherTick` preserves current behavior); they
/// exist so the calibration loop can try each cadence against the recordings.
#[allow(dead_code)]
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum HitCadence {
    /// One impulse every contact tick (most aggressive; RL-like carrying).
    EveryTick,
    /// One impulse every other tick (legacy RocketSim behavior).
    EveryOtherTick,
    /// One impulse per contact episode — re-arms only after the ball has
    /// separated from the car for at least one tick.
    OncePerEpisode,
}

/// All tunable constants for the car-ball extra hit impulse.
#[derive(Clone, Copy, Debug)]
pub struct BallHitConfig {
    /// The z-component of the hit direction is scaled by this (RL's extra
    /// impulse is mostly horizontal even for hits from below).
    pub z_scale: f32,
    /// Hoops: when the car is grounded and the ball is nearly above it, the
    /// z-scale is boosted to pop the ball up.
    pub z_scale_hoops_ground: f32,
    /// Hoops: the car's up-axis z must exceed this for the hoops z-scale.
    pub z_scale_hoops_normal_z_thresh: f32,
    /// The hit direction is nudged toward the car's forward axis by this
    /// factor (a smaller value = more forward-biased impulse).
    pub forward_scale: f32,
    /// Relative speed is clamped to this before the factor curve is applied.
    pub max_delta_vel_uu: f32,
    /// Impulse magnitude multiplier as a function of relative speed (UU units).
    pub factor_curve: LinearPieceCurve<4>,
    /// How often the impulse may fire during sustained contact.
    pub cadence: HitCadence,
}

impl Default for BallHitConfig {
    fn default() -> Self {
        use crate::consts::{ball::car_hit_impulse, curves};

        Self {
            z_scale: car_hit_impulse::Z_SCALE_NORMAL,
            z_scale_hoops_ground: car_hit_impulse::Z_SCALE_HOOPS_GROUND,
            z_scale_hoops_normal_z_thresh: car_hit_impulse::Z_SCALE_HOOPS_NORMAL_Z_THRESH,
            forward_scale: car_hit_impulse::FORWARD_SCALE,
            max_delta_vel_uu: car_hit_impulse::MAX_DELTA_VEL_UU,
            factor_curve: curves::BALL_CAR_EXTRA_IMPULSE_FACTOR,
            cadence: HitCadence::OncePerEpisode,
        }
    }
}