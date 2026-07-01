use glam::Vec3A as GlamVec3A;
use rocketsim::{Arena, CarBodyConfig, CarControls, CarState, GameMode, PhysState, Team};

use crate::rl_comparison_test::recording::Recording;
use crate::rl_comparison_test::recording::tick_record::TickRecord;

mod compare;
mod recording;

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

        // RL stores jump_time=0 on the activation tick; the sim uses
        // jump_time=TICK_TIME to skip Phase A's immediate-force branch.
        // Use MIN_TIME so Phase B can end the jump immediately if conditions
        // are met (matching RL's ability to end jumps within 1 tick).
        if cs.is_jumping && cs.jump_time == 0.0 {
            cs.jump_time = rocketsim::consts::car::jump::MIN_TIME;
        }

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

        cs.controls = car_controls[i];

        arena.set_car_state(car_idx, cs);
    }

    let rep_bs_phys: PhysState = tick.ball_record.into();
    let mut bs = *arena.get_ball_state();
    bs.phys = rep_bs_phys;
    arena.set_ball_state(bs);
}

fn run_mode(
    recording: &Recording,
    arena: &mut Arena,
    car_idcs: &[usize],
    per_tick_restore: bool,
    max_ticks: Option<usize>,
) -> (f32, f32, f32, f32, usize, Vec<(String, f32, String)>) {
    // (max_error, avg_error, max_vel_raw, max_pos_raw, max_err_tick, top_fields)
    let stride = recording.stride;
    let mut max_err = 0.0f32;
    let mut sum_err = 0.0f32;
    let mut max_vel_raw = 0.0f32;
    let mut max_pos_raw = 0.0f32;
    // We access ticks[i] and ticks[i + stride].  The last valid i
    // must satisfy i + stride < ticks.len(), so i ≤ len - stride - 1.
    let raw_max = recording.ticks.len().saturating_sub(stride + 1);
    let n = max_ticks.map_or(raw_max, |m| m.min(raw_max));
    // Round down to stride boundary
    let n = (n / stride) * stride;

    let mut max_err_tick = 0usize;
    let mut max_err_fields: Vec<(String, f32, String)> = Vec::new();
    let mut steps = 0u32;
    for i in (0..=n).step_by(stride) {
        let from_tick = &recording.ticks[i];
        let to_tick = &recording.ticks[i + stride];
        let controls_during: Vec<CarControls> = to_tick
            .car_records
            .iter()
            .map(|car_record| car_record.prev_controls.into())
            .collect();

        if per_tick_restore || i == 0 {
            set_state_to_record_tick(arena, car_idcs, from_tick, &controls_during);
        } else {
            // Continuous mode: only update controls via the lightweight
            // set_controls (single field assignment), NOT set_car_state
            // which resets position/velocity/accum/wheels.
            for (j, &car_idx) in car_idcs.iter().enumerate() {
                arena.set_car_controls(car_idx, controls_during[j]);
            }
        }

        arena.step_tick();

        let car_states: Vec<CarState> = car_idcs
            .iter()
            .map(|&car_idx| *arena.get_car_state(car_idx))
            .collect();
        let ball_state = arena.get_ball_state();

        let comparison = compare::compare_states_to_tick(&car_states, ball_state, to_tick);
        let norm_error = comparison.calc_norm_error();

        if norm_error > max_err {
            max_err = norm_error;
            max_err_tick = i;
            // Save top-5 fields with raw pred/real values for debug
            let mut fields: Vec<(String, f32, String)> = comparison.map().iter()
                .map(|(k, v)| {
                    let detail = match v {
                        compare::Comparison::Float(d) =>
                            format!("{} vs {}", d.pred, d.real),
                        compare::Comparison::Bool(d) =>
                            format!("{} vs {}", d.pred, d.real),
                        compare::Comparison::Vec(d) =>
                            format!("{:.1} vs {:.1}", d.pred, d.real),
                    };
                    (k.clone(), v.rel_error(), detail)
                })
                .collect();
            fields.sort_by(|a, b| b.1.partial_cmp(&a.1).unwrap());
            fields.truncate(5);
            max_err_fields = fields;
        }
        sum_err += norm_error;
        steps += 1;

        // Track raw velocity/position deltas
        let pred_vel = car_states[0].phys.vel;
        let real_vel: GlamVec3A = to_tick.car_records[0].phys.lin_vel.into();
        let vel_delta = (pred_vel - real_vel).length();
        if vel_delta > max_vel_raw {
            max_vel_raw = vel_delta;
        }

        let pred_pos = car_states[0].phys.pos;
        let real_pos: GlamVec3A = to_tick.car_records[0].phys.pos.into();
        let pos_delta = (pred_pos - real_pos).length();
        if pos_delta > max_pos_raw {
            max_pos_raw = pos_delta;
        }
    }

    let avg_err = if steps > 0 { sum_err / steps as f32 } else { 0.0 };
    (
        max_err,
        avg_err,
        max_vel_raw,
        max_pos_raw,
        max_err_tick,
        max_err_fields,
    )
}

fn test_recording(recording: &Recording) {
    let num_cars = recording.info.num_cars as usize;

    // ── Per-tick restore mode ──
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
    let (max_e, avg_e, max_v, max_p, max_tick, max_fields) =
        run_mode(recording, &mut arena, &car_idcs, true, None);

    // ── Continuous mode (full) ──
    let mut arena2 = Arena::new(GameMode::Soccar);
    let car_idcs2: Vec<usize> = (0..num_cars)
        .map(|i| {
            let team = if (i % 2) == 0 {
                Team::Blue
            } else {
                Team::Orange
            };
            arena2.add_car(team, CarBodyConfig::OCTANE)
        })
        .collect();
    let (max_e2, avg_e2, max_v2, max_p2, _, _) =
        run_mode(recording, &mut arena2, &car_idcs2, false, None);

    // ── Continuous mode (capped at 120 / 240 / 480 / 960 ticks) ──
    let mut caps = Vec::new();
    for &cap in &[120usize, 240, 480, 960] {
        let mut arena = Arena::new(GameMode::Soccar);
        let car_idcs: Vec<usize> = (0..num_cars)
            .map(|i| {
                let team = if (i % 2) == 0 { Team::Blue } else { Team::Orange };
                arena.add_car(team, CarBodyConfig::OCTANE)
            })
            .collect();
        let (max_e, avg_e, _, max_p, _, _) =
            run_mode(recording, &mut arena, &car_idcs, false, Some(cap));
        caps.push((cap, max_e, avg_e, max_p));
    }

    let stride_label = if recording.stride > 1 {
        format!(" stride={}", recording.stride)
    } else {
        String::new()
    };
    let top_fields_str = max_fields.iter()
        .map(|(k, v, d)| format!("{k}={v:.1}[{d}]"))
        .collect::<Vec<_>>()
        .join(" ");
    println!(
        "STAT {} | per-tick: max={:.3}@t={} avg={:.3} v={:.0} p={:.0}{} | full: max={:.3} avg={:.3} v={:.0} p={:.0} | caps: {} | top: {}",
        recording.name,
        max_e, max_tick, avg_e, max_v, max_p, stride_label,
        max_e2, avg_e2, max_v2, max_p2,
        caps.iter().map(|(t, max, _, p)| format!("{t}t: max={max:.1} p={p:.0}")).collect::<Vec<_>>().join(" | "),
        top_fields_str,
    );

    // Panic only on extreme per-tick divergence (infrastructure bug)
    if max_e > 1000.0 {
        panic!("{}: per-tick max error {max_e:.1} > 1000 — likely dead sim", recording.name);
    }
}

// This will be called from each test file
#[allow(unused)]
fn run_comparison_test(name: &str, recording_bytes: &[u8]) {
    if !rocketsim::is_initialized() {
        rocketsim::init_from_default(true).unwrap();
    }
    let recording = Recording::from_bytes(name, recording_bytes).unwrap();
    test_recording(&recording);
}
include!(concat!(env!("OUT_DIR"), "/gen_comparison_tests.rs"));
