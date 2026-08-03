//! Arena driving: restore state from recordings, step the sim, and feed the
//! lean per-tick deltas into a [`Report`].
//!
//! Two modes:
//! - [`run_per_tick`] restores the full recorded state every tick, isolating
//!   single-tick physics accuracy. This is the gated, deep-divable mode.
//! - [`run_continuous`] sets state once at tick 0 and runs freely, showing how
//!   errors compound. Informational only.

use rocketsim::{Arena, BallState, CarBodyConfig, CarControls, CarState, GameMode, PhysState, Team};

use super::config::HarnessConfig;
use super::measure::compute_delta;
use super::recording::Recording;
use super::recording::cpp_records::PhysRecord;
use super::recording::tick_record::TickRecord;
use super::report::Report;
use super::stats::Field;

/// The logger parks the ball at a sentinel position far below the arena when it
/// is not part of a (car-focused) scenario. A real ball is always inside the
/// arena (z >= ~ball radius), so a deeply-negative z marks an absent ball —
/// comparing against it would fabricate a huge divergence.
fn is_ball_sentinel(phys: &PhysRecord) -> bool {
    phys.pos.z < -1000.0
}

/// Max displacement any entity can physically travel in one recorded step.
/// Cars top out near 2300 UU/s (~19 UU/tick at 120 Hz) and the ball near
/// ~50 UU/tick, so anything beyond this is a game-state discontinuity — a goal
/// reset, demo/respawn, or a logger car-index swap — not physics the sim can
/// reproduce. Such ticks are voided from measurement.
const MAX_PHYS_DISPLACEMENT: f32 = 100.0;

/// Restore+step cycles to run on a fresh shard arena before measuring, so the
/// Bullet contact manifolds settle (a cold arena's first steps diverge).
const SHARD_WARMUP_TICKS: usize = 2;

/// True if the recording's `i -> i + stride` transition is non-physical (any
/// car or the ball jumps farther than [`MAX_PHYS_DISPLACEMENT`]).
fn has_discontinuity(recording: &Recording, i: usize, stride: usize) -> bool {
    let from = &recording.ticks[i];
    let to = &recording.ticks[i + stride];

    for (fc, tc) in from.car_records.iter().zip(&to.car_records) {
        let a: glam::Vec3A = fc.phys.pos.into();
        let b: glam::Vec3A = tc.phys.pos.into();
        if (b - a).length() > MAX_PHYS_DISPLACEMENT {
            return true;
        }
    }

    if !is_ball_sentinel(&to.ball_record) {
        let a: glam::Vec3A = from.ball_record.pos.into();
        let b: glam::Vec3A = to.ball_record.pos.into();
        if (b - a).length() > MAX_PHYS_DISPLACEMENT {
            return true;
        }
    }

    false
}

pub fn make_arena(num_cars: usize) -> (Arena, Vec<usize>) {
    let mut arena = Arena::new(GameMode::Soccar);
    let car_idcs: Vec<usize> = (0..num_cars)
        .map(|i| {
            let team = if (i % 2) == 0 {
                Team::Blue
            } else {
                Team::Orange
            };
            arena.add_car(team, CarBodyConfig::OCTANE)
        })
        .collect();
    (arena, car_idcs)
}

pub fn set_state_to_record_tick(
    arena: &mut Arena,
    car_idcs: &[usize],
    tick: &TickRecord,
    car_controls: &[CarControls],
) {
    for (i, &car_idx) in car_idcs.iter().enumerate() {
        let mut cs = *arena.get_car_state(car_idx);
        let rep_cs: CarState = tick.car_records[i].into();
        // ── core physics ──
        cs.phys = rep_cs.phys;
        // ── jump/flip state machine ──
        cs.is_on_ground = rep_cs.is_on_ground;
        cs.is_jumping = rep_cs.is_jumping;
        cs.is_flipping = rep_cs.is_flipping;
        cs.jump_time = rep_cs.jump_time;
        cs.flip_time = rep_cs.flip_time;
        cs.has_jumped = rep_cs.has_jumped;
        cs.has_double_jumped = rep_cs.has_double_jumped;
        cs.has_flipped = rep_cs.has_flipped;
        cs.flip_rel_torque = rep_cs.flip_rel_torque;
        // ── controls ──
        cs.prev_controls = rep_cs.prev_controls;
        // ── v2 fields restored from recording ──
        cs.boost = rep_cs.boost;
        cs.is_boosting = rep_cs.is_boosting;
        cs.is_supersonic = rep_cs.is_supersonic;
        cs.handbrake_val = rep_cs.handbrake_val;
        cs.is_demoed = rep_cs.is_demoed;
        cs.demo_respawn_timer = rep_cs.demo_respawn_timer;
        cs.air_time = rep_cs.air_time;
        cs.air_time_since_jump = rep_cs.air_time_since_jump;

        // The observer tracks is_flipping until landing; the sim ends it
        // after TORQUE_TIME. If the recording has is_flipping=true with
        // flip_time=0 (free-flight frame), push it past TORQUE_TIME so
        // the sim doesn't re-apply flip torque.
        if cs.is_flipping && cs.flip_time == 0.0 {
            cs.flip_time = rocketsim::consts::car::flip::TORQUE_TIME;
        }

        // The RLPR format doesn't store per-wheel contact state, but the
        // sim derives is_on_ground from wheels_with_contact at step start.
        // Without this, the step function overwrites our restored is_on_ground
        // with stale contact data from the previous sim tick.
        cs.wheels_with_contact = if cs.is_on_ground {
            [true; 4]
        } else {
            [false; 4]
        };

        // With v2 RLPR recordings, air_time_since_jump is populated by the
        // observer. Use the recording value directly — the old forced 2.0
        // (to close the double-jump window) would cause false divergence on
        // every airborne tick. If the recording has 0 (v1 or grounded),
        // still close the window to prevent false flips.
        if cs.air_time_since_jump == 0.0 {
            cs.air_time_since_jump = 2.0; // > DOUBLEJUMP_MAX_DELAY (1.25)
        }

        // Reset internal fields the recording does NOT capture to defaults so
        // each tick is truly isolated and reproducible. Without this, residual
        // values leak from the previous tick (or from a shard's fresh arena),
        // making single- and multi-threaded runs disagree.
        cs.world_contact_normal = None;
        cs.bump_cooldown_timer = 0.0;
        cs.supersonic_grace_timer = 0.0;
        cs.boosting_time = 0.0;
        cs.time_since_boosted = 0.0;
        cs.is_auto_flipping = false;
        cs.auto_flip_timer = 0.0;
        cs.auto_flip_torque_scale = 0.0;

        cs.controls = car_controls[i];

        arena.set_car_state(car_idx, cs);
    }

    let rep_bs_phys: PhysState = tick.ball_record.into();
    let mut bs = *arena.get_ball_state();
    bs.phys = rep_bs_phys;
    arena.set_ball_state(bs);
}

/// Upper loop bound shared by both modes.
fn loop_bound(recording: &Recording, max_ticks: Option<usize>) -> usize {
    let stride = recording.stride;
    let raw_max = recording.ticks.len().saturating_sub(stride + 1);
    let n = max_ticks.map_or(raw_max, |m| m.min(raw_max));
    (n / stride) * stride
}

fn controls_for(to_tick: &TickRecord, buf: &mut Vec<CarControls>) {
    for (j, car_record) in to_tick.car_records.iter().enumerate() {
        buf[j] = car_record.prev_controls.into();
    }
}

/// Per-tick restore worker for one contiguous shard of start-tick indices.
///
/// Owns its own arena so shards can run on separate threads. Per-tick mode
/// restores the full recorded state every tick, so ticks are independent —
/// that is what makes the pass embarrassingly parallel.
fn run_per_tick_shard(recording: &Recording, shard: &[usize]) -> Report {
    let num_cars = recording.info.num_cars as usize;
    let stride = recording.stride;
    let (mut arena, car_idcs) = make_arena(num_cars);
    let mut report = Report::new(recording.name.clone(), stride, num_cars, shard.len());
    let mut controls_buf: Vec<CarControls> = vec![CarControls::DEFAULT; num_cars];

    // The first step or two on a fresh (cold) arena diverge before the Bullet
    // contact manifolds settle, so run a few restore+step cycles without
    // measuring them to warm the arena up.
    let mut warmup_left = SHARD_WARMUP_TICKS;

    for &i in shard {
        // Void non-physical recording discontinuities (goal resets, demos,
        // respawns, logger car-index swaps): a physics step cannot reproduce a
        // teleport, so measuring it would fabricate divergence.
        if has_discontinuity(recording, i, stride) {
            report.ticks_voided += 1;
            continue;
        }

        let from_tick = &recording.ticks[i];
        let to_tick = &recording.ticks[i + stride];
        controls_for(to_tick, &mut controls_buf);

        set_state_to_record_tick(&mut arena, &car_idcs, from_tick, &controls_buf);
        arena.step_tick();

        if warmup_left > 0 {
            warmup_left -= 1;
            continue;
        }

        for (j, &car_idx) in car_idcs.iter().enumerate() {
            let real = &to_tick.car_records[j];
            // A demoed car is parked/absent (the logger parks it at origin);
            // measuring it against the sim fabricates divergence.
            if real.is_demoed {
                continue;
            }
            let cs: CarState = *arena.get_car_state(car_idx);
            let from_car = &from_tick.car_records[j];
            let delta = compute_delta(&cs.phys, &real.phys);

            let ent = &mut report.entities[j];
            for field in Field::ALL {
                let s = *delta.get(field);
                ent.field_mut(field).add(s.mag, s.signed, i, Some(from_car));
            }
            ent.state.record(&cs, real);
        }

        // Skip the ball when it is parked at the "absent" sentinel; comparing
        // against it would fabricate divergence (see is_ball_sentinel).
        if !is_ball_sentinel(&to_tick.ball_record) {
            let ball_state: &BallState = arena.get_ball_state();
            let ball_delta = compute_delta(&ball_state.phys, &to_tick.ball_record);
            let ball_ent = &mut report.entities[report.num_cars];
            for field in Field::ALL {
                let s = *ball_delta.get(field);
                ball_ent.field_mut(field).add(s.mag, s.signed, i, None);
            }
        }

        report.ticks_measured += 1;
    }

    report
}

/// Per-tick restore mode, sharded across `cfg.threads` worker threads.
///
/// Fills a [`Report`] with lean physics deltas + state mismatch counts. This
/// is the gated mode. Each worker owns an arena and a tick shard; results are
/// merged at the end.
pub fn run_per_tick(recording: &Recording, cfg: &HarnessConfig) -> Report {
    let stride = recording.stride;
    let n = loop_bound(recording, None);
    let num_cars = recording.info.num_cars as usize;
    let starts: Vec<usize> = (0..=n).step_by(stride).collect();

    if starts.is_empty() {
        return Report::new(recording.name.clone(), stride, num_cars, 0);
    }

    // Not worth sharding tiny recordings: thread + per-arena setup overhead
    // exceeds the stepping cost. Parallelize only above a tick threshold.
    const PARALLEL_MIN_TICKS: usize = 1024;
    let threads = if starts.len() < PARALLEL_MIN_TICKS {
        1
    } else {
        cfg.threads.clamp(1, starts.len())
    };
    if threads <= 1 {
        return run_per_tick_shard(recording, &starts);
    }

    let mut merged = Report::new(recording.name.clone(), stride, num_cars, 0);
    let chunk = starts.len().div_ceil(threads);
    std::thread::scope(|s| {
        let handles: Vec<_> = starts
            .chunks(chunk)
            .map(|shard| s.spawn(move || run_per_tick_shard(recording, shard)))
            .collect();
        for handle in handles {
            merged.merge(handle.join().expect("per-tick shard panicked"));
        }
    });
    merged
}

/// Summary of free-running divergence (informational, not gated).
#[derive(Debug, Clone, Copy, Default)]
pub struct ContinuousSummary {
    pub ticks: u64,
    pub max_pos: f32,
    pub max_pos_tick: usize,
    pub max_vel: f32,
    pub max_vel_tick: usize,
}

/// Continuous mode: state set once at tick 0, then runs freely. Tracks the
/// focused car's (or car 0's) divergence growth.
pub fn run_continuous(
    recording: &Recording,
    arena: &mut Arena,
    car_idcs: &[usize],
    cfg: &HarnessConfig,
) -> ContinuousSummary {
    let stride = recording.stride;
    let n = loop_bound(recording, None);
    let num_cars = car_idcs.len();
    let mut controls_buf: Vec<CarControls> = vec![CarControls::DEFAULT; num_cars];
    let focus = cfg.car_focus.unwrap_or(0).min(num_cars.saturating_sub(1));

    let mut summary = ContinuousSummary::default();

    for i in (0..=n).step_by(stride) {
        let to_tick = &recording.ticks[i + stride];
        controls_for(to_tick, &mut controls_buf);

        if i == 0 {
            let from_tick = &recording.ticks[i];
            set_state_to_record_tick(arena, car_idcs, from_tick, &controls_buf);
        } else {
            for (j, &car_idx) in car_idcs.iter().enumerate() {
                arena.set_car_controls(car_idx, controls_buf[j]);
            }
        }
        arena.step_tick();

        let cs: CarState = *arena.get_car_state(car_idcs[focus]);
        let real = &to_tick.car_records[focus];
        let pos_delta = (cs.phys.pos - glam::Vec3A::from(real.phys.pos)).length();
        let vel_delta = (cs.phys.vel - glam::Vec3A::from(real.phys.lin_vel)).length();
        if pos_delta > summary.max_pos {
            summary.max_pos = pos_delta;
            summary.max_pos_tick = i;
        }
        if vel_delta > summary.max_vel {
            summary.max_vel = vel_delta;
            summary.max_vel_tick = i;
        }
        summary.ticks += 1;
    }

    summary
}
