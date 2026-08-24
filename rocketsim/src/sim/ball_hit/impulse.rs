//! The pure extra-hit impulse function.
//!
//! `compute_impulse` is a pure function of the hit geometry and the config —
//! no side effects, no mutable state. The caller decides *whether* to fire
//! (via [`can_fire`]) and applies the returned delta-velocity to the ball.

use glam::Vec3A;

use crate::{GameMode, sim::ball_hit::config::BallHitConfig};

use super::config::HitCadence;
use super::state::BallHitState;

/// Everything the impulse needs to know about a single car-ball contact.
#[derive(Clone, Copy, Debug)]
pub struct HitContext {
    pub ball_pos: Vec3A,
    pub ball_vel: Vec3A,
    pub car_pos: Vec3A,
    pub car_vel: Vec3A,
    pub car_forward: Vec3A,
    pub game_mode: GameMode,
    pub car_is_on_ground: bool,
    /// The car's up-axis z-component (1.0 = perfectly upright).
    pub car_up_z: f32,
}

/// Whether the impulse may fire this tick, given the cadence and hit state.
pub fn can_fire(state: &BallHitState, config: &BallHitConfig, tick_count: u64) -> bool {
    let spacing_ok = state
        .last_impulse_tick
        .is_none_or(|last| last + 1 < tick_count);
    match config.cadence {
        HitCadence::EveryTick => true,
        HitCadence::EveryOtherTick => spacing_ok,
        // Once the ball has had a tick without contact, the episode is over
        // and the impulse may re-arm.
        HitCadence::OncePerEpisode => {
            spacing_ok
                && state
                    .last_contact_tick
                    .is_none_or(|last| last + 1 < tick_count)
        }
    }
}

/// Compute the extra hit impulse (delta-velocity, UU units) for one contact.
///
/// Returns the zero vector when the relative speed is zero (i.e. no impulse).
/// This is the exact formula the legacy RocketSim behavior used, lifted into a
/// pure function so it can be unit-tested and calibrated.
pub fn compute_impulse(ctx: &HitContext, config: &BallHitConfig) -> Vec3A {
    let rel_pos = ctx.ball_pos - ctx.car_pos;
    let rel_vel = ctx.ball_vel - ctx.car_vel;
    let rel_speed = rel_vel.length().min(config.max_delta_vel_uu);

    if rel_speed <= 0.0 {
        return Vec3A::ZERO;
    }

    let extra_z_scale = ctx.game_mode == GameMode::Hoops
        && ctx.car_is_on_ground
        && ctx.car_up_z > config.z_scale_hoops_normal_z_thresh;
    let z_scale = if extra_z_scale {
        config.z_scale_hoops_ground
    } else {
        config.z_scale
    };

    let mut hit_dir = (rel_pos * Vec3A::new(1.0, 1.0, z_scale)).normalize_or_zero();
    let forward_dir_adjustment =
        ctx.car_forward * hit_dir.dot(ctx.car_forward) * (1.0 - config.forward_scale);
    hit_dir = (hit_dir - forward_dir_adjustment).normalize_or_zero();

    hit_dir * rel_speed * config.factor_curve.get_output(rel_speed)
}

#[cfg(test)]
mod tests {
    use glam::Vec3A;

    use crate::GameMode;
    use crate::sim::ball_hit::config::{BallHitConfig, HitCadence};
    use crate::sim::ball_hit::state::BallHitState;

    use super::{HitContext, can_fire, compute_impulse};

    // Ball directly in front of the car (slightly above center), car driving
    // forward into it.
    fn ctx() -> HitContext {
        HitContext {
            ball_pos: Vec3A::new(100.0, 0.0, 20.0),
            ball_vel: Vec3A::ZERO,
            car_pos: Vec3A::ZERO,
            car_vel: Vec3A::new(500.0, 0.0, 0.0),
            car_forward: Vec3A::X,
            game_mode: GameMode::Soccar,
            car_is_on_ground: true,
            car_up_z: 1.0,
        }
    }

    #[test]
    fn impulse_forward_hit() {
        let cfg = BallHitConfig::default();
        let imp = compute_impulse(&ctx(), &cfg);
        // rel_speed = 500, factor = 0.65 (flat segment), z_scale=0.35 → hit_dir
        // is nearly +X. Magnitude ≈ 500 * 0.65 = 325.
        assert!(imp.x > 300.0, "impulse should push forward, got {imp:?}");
        assert!(imp.y.abs() < 1e-3);
        // z is a small positive fraction of the x component (z_scale=0.35).
        let z_frac = imp.z / imp.x;
        assert!(z_frac > 0.0 && z_frac < 0.5, "z_frac = {z_frac}");
    }

    #[test]
    fn zero_relative_speed_gives_no_impulse() {
        let cfg = BallHitConfig::default();
        let mut c = ctx();
        c.ball_vel = c.car_vel; // ball moving with the car
        assert_eq!(compute_impulse(&c, &cfg), Vec3A::ZERO);
    }

    #[test]
    fn every_other_tick_blocks_consecutive() {
        let cfg = BallHitConfig {
            cadence: HitCadence::EveryOtherTick,
            ..BallHitConfig::default()
        };
        let mut st = BallHitState::DEFAULT;
        assert!(can_fire(&st, &cfg, 10));
        st.last_impulse_tick = Some(10);
        assert!(!can_fire(&st, &cfg, 11), "next tick must be blocked");
        assert!(can_fire(&st, &cfg, 12), "must re-arm after one tick");
    }

    #[test]
    fn every_tick_always_fires() {
        let cfg = BallHitConfig {
            cadence: HitCadence::EveryTick,
            ..BallHitConfig::default()
        };
        let mut st = BallHitState::DEFAULT;
        st.last_impulse_tick = Some(10);
        assert!(can_fire(&st, &cfg, 11));
        assert!(can_fire(&st, &cfg, 12));
    }

    #[test]
    fn once_per_episode_rearms_after_separation() {
        let cfg = BallHitConfig {
            cadence: HitCadence::OncePerEpisode,
            ..BallHitConfig::default()
        };
        let mut st = BallHitState::DEFAULT;
        st.last_contact_tick = Some(10);
        st.last_impulse_tick = Some(10);
        // Still in contact: blocked (impulse spacing not met either).
        assert!(!can_fire(&st, &cfg, 11));
        // Separated for a tick and the impulse is old: re-armed.
        st.last_contact_tick = Some(9);
        st.last_impulse_tick = Some(5);
        assert!(can_fire(&st, &cfg, 11));
    }
}
