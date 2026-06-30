use rocketsim::{Arena, ArenaEvent, GameMode};

use crate::conversions::*;
use crate::ffi;

/// Per-car ball hit info, synthesized from CarHitBall events during stepping.
/// This replicates the C++ BallHitInfo that Player::UpdateFromCar reads.
#[derive(Clone, Debug)]
pub struct SynthBallHitInfo {
    pub is_valid: bool,
    pub relative_pos_on_ball: ffi::FfiVec,
    pub ball_pos: ffi::FfiVec,
    pub extra_hit_vel: ffi::FfiVec,
    pub tick_count_when_hit: u64,
    pub tick_count_when_extra_impulse_applied: u64,
}

impl Default for SynthBallHitInfo {
    fn default() -> Self {
        let zero = ffi::FfiVec { x: 0.0, y: 0.0, z: 0.0 };
        Self {
            is_valid: false,
            relative_pos_on_ball: zero,
            ball_pos: zero,
            extra_hit_vel: zero,
            tick_count_when_hit: 0,
            tick_count_when_extra_impulse_applied: 0,
        }
    }
}

/// Wrapper around the Rust Arena that adds event collection and BallHitInfo synthesis.
pub struct RsArena {
    pub(crate) arena: Arena,
    /// Per-car synthesized BallHitInfo (indexed by car_idx)
    pub(crate) ball_hit_infos: Vec<SynthBallHitInfo>,
    /// Accumulated CarHitBall events from last step (across all ticks)
    pub(crate) last_car_hit_ball_events: Vec<ffi::FfiCarHitBallEvent>,
    /// Accumulated CarHitCar events from last step (across all ticks)
    pub(crate) last_car_hit_car_events: Vec<ffi::FfiCarHitCarEvent>,
    /// Tick rate scale factor (for prediction arenas at non-120Hz)
    pub(crate) tick_rate_scale: u32,
}

impl RsArena {
    pub fn new(arena: Arena, tick_rate_scale: u32) -> Self {
        let num_cars = arena.num_cars();
        Self {
            arena,
            ball_hit_infos: vec![SynthBallHitInfo::default(); num_cars],
            last_car_hit_ball_events: Vec::new(),
            last_car_hit_car_events: Vec::new(),
            tick_rate_scale,
        }
    }

    /// Step the arena for N logical ticks (scaled by tick_rate_scale).
    /// Collects events across all ticks and updates BallHitInfo.
    ///
    /// ADAPTED for new rocketsim API: `step_tick()` now returns `&[ArenaEvent]`
    /// directly, so we consume the returned slice instead of calling
    /// `get_last_step_events()` separately.
    pub fn step(&mut self, ticks: u32) {
        let actual_ticks = ticks * self.tick_rate_scale;

        // Clear accumulated events
        self.last_car_hit_ball_events.clear();
        self.last_car_hit_car_events.clear();

        for _ in 0..actual_ticks {
            // New API: step_tick() returns &[ArenaEvent] directly.
            // The returned slice borrows self.arena mutably, so we must clone it
            // before accessing self.arena again (for tick_count, ball_pos, etc.).
            let events = self.arena.step_tick().to_vec();

            let tick_count = self.arena.tick_count();
            // New API: get_ball_state() returns &BallState — clone for owned access
            let ball_pos = self.arena.get_ball_state().phys.pos;

            for event in &events {
                match event {
                    ArenaEvent::CarHitBall(e) => {
                        let contact_point = vec3a_to_ffi(e.contact_point);
                        let extra_hit_vel = vec3a_to_ffi(e.extra_hit_vel);
                        let ball_pos_ffi = vec3a_to_ffi(ball_pos);

                        // Update per-car BallHitInfo
                        if e.car_idx < self.ball_hit_infos.len() {
                            let info = &mut self.ball_hit_infos[e.car_idx];
                            info.is_valid = true;
                            info.relative_pos_on_ball = ffi::FfiVec {
                                x: contact_point.x - ball_pos_ffi.x,
                                y: contact_point.y - ball_pos_ffi.y,
                                z: contact_point.z - ball_pos_ffi.z,
                            };
                            info.ball_pos = ball_pos_ffi;
                            info.extra_hit_vel = extra_hit_vel;
                            info.tick_count_when_hit = tick_count;
                            info.tick_count_when_extra_impulse_applied = tick_count;
                        }

                        // Accumulate for C++ callback dispatch
                        self.last_car_hit_ball_events.push(ffi::FfiCarHitBallEvent {
                            car_idx: e.car_idx as u32,
                            contact_point,
                            extra_hit_vel,
                        });
                    }
                    ArenaEvent::CarHitCar(e) => {
                        self.last_car_hit_car_events.push(ffi::FfiCarHitCarEvent {
                            bumper_car_idx: e.bumper_car_idx as u32,
                            victim_car_idx: e.victim_car_idx as u32,
                            contact_point: vec3a_to_ffi(e.contact_point),
                            is_demo: e.is_demo,
                        });
                    }
                    // Other events not needed by the C++ compat layer
                    _ => {}
                }
            }
        }
    }

    /// Grow ball_hit_infos when a car is added
    pub fn ensure_ball_hit_info_size(&mut self) {
        let num_cars = self.arena.num_cars();
        while self.ball_hit_infos.len() < num_cars {
            self.ball_hit_infos.push(SynthBallHitInfo::default());
        }
    }
}

/// Lightweight ball-only arena for trajectory prediction.
///
/// ADAPTED for new rocketsim API: the upstream crate removed `new_ball_only()`
/// and `step_tick_ball_only()`. We work around this by using a regular Arena
/// with `TheVoid` game mode (no arena collision meshes, no boost pads) and
/// stepping via the normal `step_tick()` path. Cars are not added.
///
/// Performance note: this is functionally correct but may be slower than the
/// old dedicated ball-only path since it still runs the full physics pipeline.
/// Optimize later if profiling shows this as a bottleneck.
pub struct RsBallArena {
    pub(crate) arena: Arena,
    pub(crate) tick_time: f32,
}

impl RsBallArena {
    pub fn new(game_mode: GameMode, tick_rate: f32) -> Self {
        // Use regular Arena — TheVoid has no collision meshes or boost pads,
        // so the overhead is minimal. Car physics are skipped if no cars exist.
        Self {
            arena: Arena::new(game_mode),
            tick_time: 1.0 / tick_rate,
        }
    }

    pub fn step(&mut self, ticks: u32) {
        // Step the arena at 120Hz internal rate, scaling for the desired tick rate.
        // For 15Hz prediction: each logical tick = 8 internal ticks.
        let tick_rate_scale = (120.0 * self.tick_time) as u32;
        let actual_ticks = ticks * tick_rate_scale.max(1);
        for _ in 0..actual_ticks {
            self.arena.step_tick();
        }
    }
}
