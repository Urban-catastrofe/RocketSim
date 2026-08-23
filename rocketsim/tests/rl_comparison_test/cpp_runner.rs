//! On-demand C++ RocketSim comparison pass.
//!
//! Enabled with `RLCPP=1` and the `cpp-compare` cargo feature. Replays the
//! exact same recording through the original C++ RocketSim (via the
//! `rocketsim_rs` bindings) using the same per-tick restore protocol as
//! [`super::runner`], then reports its accuracy next to ours. Whatever C++
//! nails and our port misses points straight at the bug in our port.
//!
//! The C++ pass is informational only: it never gates.

use std::sync::OnceLock;

use glam::{Mat3A, Vec3A};
use rocketsim_rs::cxx::UniquePtr;
use rocketsim_rs::math::{RotMat, Vec3};
use rocketsim_rs::sim::{
    Arena, BallState, CarConfig, CarContact, CarControls, CarState, Team, WorldContact,
};

use super::config::HarnessConfig;
use super::measure::{PhysicsDelta, compute_delta};
use super::recording::Recording;
use super::recording::cpp_records::{ControlsRecord, Mat3Record, VecRecord};
use super::recording::tick_record::TickRecord;
use super::report::Report;
use super::rollout::RolloutReport;
use super::runner;
use super::stats::Field;

fn vec3(v: VecRecord) -> Vec3 {
    vec3_new(v.x, v.y, v.z)
}

fn vec3_new(x: f32, y: f32, z: f32) -> Vec3 {
    let mut out = Vec3::default();
    out.x = x;
    out.y = y;
    out.z = z;
    out
}

fn vec3a(v: Vec3) -> Vec3A {
    Vec3A::new(v.x, v.y, v.z)
}

/// RLPR stores the rotation as axis-vector rows (0=forward, 1=right, 2=up),
/// which is exactly C++ RocketSim's `RotMat` member layout.
fn rot_mat(m: Mat3Record) -> RotMat {
    RotMat {
        forward: vec3(m.rows[0]),
        right: vec3(m.rows[1]),
        up: vec3(m.rows[2]),
    }
}

fn controls(c: ControlsRecord) -> CarControls {
    CarControls {
        throttle: c.throttle,
        steer: c.steer,
        pitch: c.pitch,
        yaw: c.yaw,
        roll: c.roll,
        jump: c.jump,
        boost: c.boost,
        handbrake: c.handbrake,
    }
}

/// The C++ sim's global collision-mesh init. Tests run with the crate root as
/// CWD, so the meshes live in `collision_meshes/` (soccar subdir).
fn init_cpp() {
    static CPP_READY: OnceLock<()> = OnceLock::new();
    CPP_READY.get_or_init(|| {
        rocketsim_rs::init(Some("collision_meshes"), true);
    });
}

fn make_cpp_arena(num_cars: usize) -> (UniquePtr<Arena>, Vec<u32>) {
    let mut arena = Arena::default_standard();
    let car_ids: Vec<u32> = (0..num_cars)
        .map(|i| {
            let team = if (i % 2) == 0 {
                Team::Blue
            } else {
                Team::Orange
            };
            arena.pin_mut().add_car(team, CarConfig::octane())
        })
        .collect();
    (arena, car_ids)
}

/// C++-side twin of [`super::runner::set_state_to_record_tick`]: restore the
/// full recorded state (including the same RLPR observer fixups) so each tick
/// is measured in isolation.
///
/// With `rust_aligned` the jump-time observer fixup mirrors the Rust runner's
/// velocity-conditional version exactly (needed by the direct sim-vs-sim pass
/// so both sims start from an identical state); the standalone C++ passes keep
/// the historical unconditional fixup.
fn set_cpp_state_to_record_tick(
    arena: &mut UniquePtr<Arena>,
    car_ids: &[u32],
    tick: &TickRecord,
    step_controls: &[CarControls],
) {
    set_cpp_state_to_record_tick_impl(arena, car_ids, tick, step_controls, false)
}

fn set_cpp_state_to_record_tick_impl(
    arena: &mut UniquePtr<Arena>,
    car_ids: &[u32],
    tick: &TickRecord,
    step_controls: &[CarControls],
    rust_aligned: bool,
) {
    for (i, &car_id) in car_ids.iter().enumerate() {
        let rec = &tick.car_records[i];
        let mut cs = arena.pin_mut().get_car(car_id);

        // The logger parks demoed cars at the origin WITHOUT setting
        // is_demoed (and the recording keeps them is_demoed=true while parked
        // there). Either way they are absent from real play; restoring several
        // of them into the same origin pose makes bullet's solver produce NaN
        // (fully-overlapping rigid bodies). Park them far below the arena,
        // demoed with a huge respawn timer, so they never disturb the live
        // cars/ball. They are skipped in measurement anyway; the tick where
        // the real game respawns them is voided as a discontinuity.
        if runner::is_car_sentinel(&rec.phys) || rec.is_demoed {
            cs.pos = vec3_new(0.0, 0.0, -5000.0 - (i as f32) * 200.0);
            cs.vel = Vec3::default();
            cs.ang_vel = Vec3::default();
            cs.is_demoed = true;
            cs.demo_respawn_timer = 9999.0;
            cs.last_controls = controls(rec.prev_controls);
            arena
                .pin_mut()
                .set_car(car_id, cs)
                .expect("cpp car id vanished");
            arena
                .pin_mut()
                .set_car_controls(car_id, step_controls[i])
                .expect("cpp car id vanished");
            continue;
        }

        // ── core physics ──
        cs.pos = vec3(rec.phys.pos);
        cs.vel = vec3(rec.phys.lin_vel);
        cs.ang_vel = vec3(rec.phys.ang_vel);
        cs.rot_mat = rot_mat(rec.phys.rot);

        // ── jump/flip state machine ──
        cs.is_on_ground = rec.is_on_ground;
        cs.is_jumping = rec.is_jumping;
        cs.is_flipping = rec.is_flipping;
        cs.jump_time = rec.jump_time;
        cs.flip_time = rec.flip_time;
        cs.has_jumped = rec.has_jumped;
        cs.has_double_jumped = rec.double_jumped_or_flipped && !rec.is_flipping;
        cs.has_flipped = rec.double_jumped_or_flipped && rec.is_flipping;
        cs.flip_rel_torque = vec3(rec.flip_rel_torque);

        // ── v2 fields restored from recording ──
        cs.boost = rec.boost_amount;
        cs.is_boosting = rec.is_boosting;
        cs.is_supersonic = rec.is_supersonic;
        cs.handbrake_val = rec.handbrake_val;
        cs.is_demoed = rec.is_demoed;
        cs.demo_respawn_timer = rec.demo_respawn_timer;
        cs.air_time = rec.air_time;
        cs.air_time_since_jump = rec.air_time_since_jump;

        // The observer tracks is_flipping until landing; the sim ends it after
        // TORQUE_TIME. Push a free-flight is_flipping past TORQUE_TIME so the
        // flip torque is not re-applied. (Same fixup as the Rust runner.)
        if cs.is_flipping && cs.flip_time == 0.0 {
            cs.flip_time = rocketsim::consts::car::flip::TORQUE_TIME;
        }

        // Reconstruct the persistent flip state (observer only reports it on
        // the first flip tick). Mirrors the Rust runner's heuristic exactly.
        if rec.double_jumped_or_flipped && cs.flip_time > 0.0 {
            let av = rec.phys.ang_vel;
            let hard_tumble = (av.x * av.x + av.y * av.y).sqrt() > 2.0;
            let active = cs.flip_time <= rocketsim::consts::car::flip::Z_DAMP_END;
            if hard_tumble && active {
                cs.has_flipped = true;
                cs.has_double_jumped = false;
                let zd = rocketsim::consts::car::flip::Z_DAMP_START
                    ..=rocketsim::consts::car::flip::Z_DAMP_END;
                let up_z = rec.phys.rot.rows[2].z;
                if zd.contains(&cs.flip_time) && up_z > 0.0 && up_z < 0.9 {
                    cs.is_flipping = true;
                }
            }
        }

        // The observer reports is_jumping=true + jump_time=0 on the tick where
        // the impulse has already been applied; advance jump_time so it is not
        // re-applied. The rust-aligned variant mirrors the Rust runner's
        // velocity condition (a still-grounded vz means the impulse must still
        // be applied this tick).
        let jump_fixup = if rust_aligned {
            cs.is_jumping && cs.jump_time == 0.0 && cs.vel.z > 100.0
        } else {
            cs.is_jumping && cs.jump_time == 0.0
        };
        if jump_fixup {
            cs.jump_time = rocketsim::consts::TICK_TIME;
        }

        // Derived from is_on_ground. RLPR *does* carry per-wheel contact in
        // `WheelRecord::has_contact`, but this restore is inert either way -
        // `pre_tick_update` recomputes it from the raycast. See runner.rs.
        cs.wheels_with_contact = if cs.is_on_ground {
            [true; 4]
        } else {
            [false; 4]
        };

        // Reconstruct air_time_since_jump (the observer leaves it 0).
        if cs.air_time_since_jump == 0.0 {
            cs.air_time_since_jump = if cs.has_jumped {
                (cs.air_time
                    - rocketsim::consts::car::jump::MAX_TICKS as f32 * rocketsim::consts::TICK_TIME)
                    .max(0.0)
            } else {
                2.0 // > DOUBLEJUMP_MAX_DELAY (1.25)
            };
        }

        // Reset internal fields the recording does NOT capture.
        cs.time_since_boosted = 0.0;
        cs.boosting_time = 0.0;
        cs.supersonic_time = 0.0;
        cs.is_auto_flipping = false;
        cs.auto_flip_timer = 0.0;
        cs.auto_flip_torque_scale = 0.0;
        cs.car_contact = CarContact {
            other_car_id: 0,
            cooldown_timer: 0.0,
        };
        cs.ball_hit_info = Default::default();
        cs.tick_count_since_update = 0;

        // World contact IS recorded (PhysRecord) and C++ consumes it as an
        // input next tick (wall throttle/autoflip/ground-up-dir), so restore
        // it from the recording.
        cs.world_contact = WorldContact {
            has_contact: rec.phys.has_world_contact,
            contact_normal: vec3(rec.phys.world_contact_normal),
        };

        // Edge detection (jump press etc.) compares against lastControls.
        cs.last_controls = controls(rec.prev_controls);

        arena
            .pin_mut()
            .set_car(car_id, cs)
            .expect("cpp car id vanished");
        arena
            .pin_mut()
            .set_car_controls(car_id, step_controls[i])
            .expect("cpp car id vanished");
    }

    let mut bs = arena.pin_mut().get_ball();
    bs.pos = vec3(tick.ball_record.pos);
    bs.vel = vec3(tick.ball_record.lin_vel);
    bs.ang_vel = vec3(tick.ball_record.ang_vel);
    bs.rot_mat = rot_mat(tick.ball_record.rot);
    arena.pin_mut().set_ball(bs);
}

/// Adapt a C++ car state to our [`rocketsim::PhysState`] so the shared
/// [`compute_delta`] can measure it.
fn car_phys(cs: &CarState) -> rocketsim::PhysState {
    rocketsim::PhysState {
        pos: vec3a(cs.pos),
        vel: vec3a(cs.vel),
        ang_vel: vec3a(cs.ang_vel),
        rot_mat: Mat3A::from_cols(
            vec3a(cs.rot_mat.forward),
            vec3a(cs.rot_mat.right),
            vec3a(cs.rot_mat.up),
        ),
    }
}

fn ball_phys(bs: &BallState) -> rocketsim::PhysState {
    rocketsim::PhysState {
        pos: vec3a(bs.pos),
        vel: vec3a(bs.vel),
        ang_vel: vec3a(bs.ang_vel),
        rot_mat: Mat3A::from_cols(
            vec3a(bs.rot_mat.forward),
            vec3a(bs.rot_mat.right),
            vec3a(bs.rot_mat.up),
        ),
    }
}

/// The C++ sim can produce non-finite states when a restore puts rigid bodies
/// into a degenerate configuration (e.g. several parked cars stacked at the
/// origin). That would poison the percentile sort with NaN; fail loudly with
/// context instead.
fn assert_finite_delta(delta: &PhysicsDelta, entity: &str, tick: usize) {
    for field in Field::ALL {
        let s = delta.get(field);
        assert!(
            s.mag.is_finite(),
            "cpp pass produced non-finite {:?} error for {} at tick {}",
            field,
            entity,
            tick,
        );
    }
}

/// Proxy carrying only the state-machine flags + timers, for the shared state
/// channel.
fn flag_proxy(cs: &CarState) -> rocketsim::CarState {
    rocketsim::CarState {
        is_on_ground: cs.is_on_ground,
        is_jumping: cs.is_jumping,
        has_jumped: cs.has_jumped,
        is_flipping: cs.is_flipping,
        has_double_jumped: cs.has_double_jumped,
        has_flipped: cs.has_flipped,
        is_boosting: cs.is_boosting,
        is_supersonic: cs.is_supersonic,
        is_demoed: cs.is_demoed,
        jump_ticks: (cs.jump_time * rocketsim::consts::TICK_RATE).round() as u32,
        air_time: cs.air_time,
        ..Default::default()
    }
}

/// Per-tick restore pass through the C++ sim. Single-threaded: the C++ arena
/// (and bullet behind it) is driven from one thread only.
pub fn run_cpp_per_tick(recording: &Recording, _cfg: &HarnessConfig) -> Report {
    init_cpp();

    let num_cars = recording.info.num_cars as usize;
    let stride = recording.stride;
    let n = runner::loop_bound(recording, None);
    let starts: Vec<usize> = (0..=n).step_by(stride).collect();

    let mut report = Report::new(
        format!("{}|cpp", recording.name),
        stride,
        num_cars,
        starts.len(),
    );
    if starts.is_empty() {
        return report;
    }

    let (mut arena, car_ids) = make_cpp_arena(num_cars);
    let mut controls_buf: Vec<CarControls> = vec![CarControls::default(); num_cars];
    let mut warmup_left = runner::SHARD_WARMUP_TICKS;

    for &i in &starts {
        if runner::has_discontinuity(recording, i, stride) {
            report.ticks_voided += 1;
            continue;
        }

        let from_tick = &recording.ticks[i];
        let to_tick = &recording.ticks[i + stride];
        for (j, car_record) in to_tick.car_records.iter().enumerate() {
            controls_buf[j] = controls(car_record.prev_controls);
        }

        set_cpp_state_to_record_tick(&mut arena, &car_ids, from_tick, &controls_buf);
        arena.pin_mut().step(1);

        if warmup_left > 0 {
            warmup_left -= 1;
            continue;
        }

        for (j, &car_id) in car_ids.iter().enumerate() {
            let real = &to_tick.car_records[j];
            if real.is_demoed || runner::is_car_sentinel(&real.phys) {
                continue;
            }
            let cs = arena.pin_mut().get_car(car_id);
            let phys = car_phys(&cs);
            let delta = compute_delta(&phys, &real.phys);
            assert_finite_delta(&delta, &format!("car_{j}"), i);

            let ent = &mut report.entities[j];
            for field in Field::ALL {
                let s = *delta.get(field);
                ent.field_mut(field)
                    .add(s.mag, s.signed, i, Some(&from_tick.car_records[j]));
            }
            let proxy = flag_proxy(&cs);
            ent.state.record(&proxy, real);
        }

        if !runner::is_ball_sentinel(&to_tick.ball_record) {
            let bs = arena.pin_mut().get_ball();
            let phys = ball_phys(&bs);
            let delta = compute_delta(&phys, &to_tick.ball_record);
            assert_finite_delta(&delta, "ball", i);
            let ball_ent = &mut report.entities[report.num_cars];
            for field in Field::ALL {
                let s = *delta.get(field);
                ball_ent.field_mut(field).add(s.mag, s.signed, i, None);
            }
        }

        report.ticks_measured += 1;
    }

    report
}

/// C++ twin of [`super::rollout::run_rollout`]: restore at sampled start
/// ticks, free-run the C++ sim for `rollout_ticks` steps with recorded
/// controls, and capture the same error-growth curve. Single-threaded.
pub fn run_cpp_rollout(recording: &Recording, cfg: &HarnessConfig) -> RolloutReport {
    init_cpp();

    let stride = recording.stride;
    let rollout_ticks = cfg.rollout_ticks;
    let num_cars = recording.info.num_cars as usize;
    let n = runner::loop_bound(recording, None);
    let starts: Vec<usize> = (0..=n).step_by(cfg.rollout_stride * stride).collect();

    let mut report = RolloutReport::new(format!("{}|cpp", recording.name), recording, cfg);
    if starts.is_empty() || rollout_ticks == 0 {
        return report;
    }

    let (mut arena, car_ids) = make_cpp_arena(num_cars);
    let mut controls_buf: Vec<CarControls> = vec![CarControls::default(); num_cars];
    let mut last_mag: Vec<[f32; Field::ALL.len()]> = vec![[0.0; Field::ALL.len()]; num_cars + 1];
    let mut warmed = false;

    for &i in &starts {
        let bound = recording.ticks.len();
        if i + stride >= bound {
            continue;
        }

        let from_tick = &recording.ticks[i];
        for (j, car_record) in recording.ticks[i + stride].car_records.iter().enumerate() {
            controls_buf[j] = controls(car_record.prev_controls);
        }
        set_cpp_state_to_record_tick(&mut arena, &car_ids, from_tick, &controls_buf);

        if !warmed {
            for _ in 0..runner::SHARD_WARMUP_TICKS {
                arena.pin_mut().step(1);
            }
            set_cpp_state_to_record_tick(&mut arena, &car_ids, from_tick, &controls_buf);
            warmed = true;
        }

        let mut last_step = 0usize;
        for k in 1..=rollout_ticks {
            let target = i + k * stride;
            if target >= bound {
                break;
            }
            if runner::has_discontinuity(recording, target - stride, stride) {
                break;
            }

            arena.pin_mut().step(1);
            let to_tick = &recording.ticks[target];

            for (j, &car_id) in car_ids.iter().enumerate() {
                let real = &to_tick.car_records[j];
                if real.is_demoed || runner::is_car_sentinel(&real.phys) {
                    continue;
                }
                let cs = arena.pin_mut().get_car(car_id);
                let phys = car_phys(&cs);
                let delta = compute_delta(&phys, &real.phys);
                assert_finite_delta(&delta, &format!("car_{j}"), target);
                let ent = &mut report.entities[j];
                for field in Field::ALL {
                    let s = *delta.get(field);
                    let fr = ent.field_mut(field);
                    fr.per_step[k - 1].add(s.mag, s.signed, i);
                    last_mag[j][super::measure::field_index(field)] = s.mag;
                }
                let proxy = flag_proxy(&cs);
                ent.state.record(&proxy, real);
            }

            if !runner::is_ball_sentinel(&to_tick.ball_record) {
                let bs = arena.pin_mut().get_ball();
                let phys = ball_phys(&bs);
                let delta = compute_delta(&phys, &to_tick.ball_record);
                assert_finite_delta(&delta, "ball", target);
                let ball_ent = &mut report.entities[report.num_cars];
                for field in Field::ALL {
                    let s = *delta.get(field);
                    let fr = ball_ent.field_mut(field);
                    fr.per_step[k - 1].add(s.mag, s.signed, i);
                    last_mag[report.num_cars][super::measure::field_index(field)] = s.mag;
                }
            }

            last_step = k;

            let next = target + stride;
            if next < bound {
                for (j, car_record) in recording.ticks[next].car_records.iter().enumerate() {
                    controls_buf[j] = controls(car_record.prev_controls);
                }
                for (j, &car_id) in car_ids.iter().enumerate() {
                    let _ = arena.pin_mut().set_car_controls(car_id, controls_buf[j]);
                }
            }
        }

        if last_step > 0 {
            report.rollouts_measured += 1;
            if last_step < rollout_ticks {
                report.rollouts_truncated += 1;
            }
            for (ei, mags) in last_mag.iter().enumerate() {
                for field in Field::ALL {
                    let fi = super::measure::field_index(field);
                    let fr = report.entities[ei].field_mut(field);
                    if mags[fi] > fr.worst_end_mag {
                        fr.worst_end_mag = mags[fi];
                        fr.worst_end_start = i;
                    }
                }
            }
        }
    }

    report
}

/// Side-by-side rollout comparison: ours vs C++ error at the final horizon.
pub fn rollout_comparison_lines(
    ours: &RolloutReport,
    cpp: &RolloutReport,
    cfg: &HarnessConfig,
) -> Vec<String> {
    let mut lines = Vec::new();
    let h = ours.rollout_ticks;
    lines.push(format!(
        "==== ROLLOUT-CPP-COMPARE {} | error at final step t{} (ratio>1 = C++ more accurate) ====",
        ours.name, h,
    ));

    let focused: Vec<usize> = match cfg.car_focus {
        Some(c) if c < ours.num_cars => vec![c],
        Some(_) => vec![],
        None => (0..ours.entities.len()).collect(),
    };

    let mut rows: Vec<(f32, String)> = Vec::new();
    for ei in focused {
        let o_ent = &ours.entities[ei];
        let c_ent = &cpp.entities[ei];
        let is_ball = !o_ent.is_car;
        for field in Field::ALL {
            let fi = super::measure::field_index(field);
            let o_stats = match o_ent.fields[fi].per_step.get(h - 1) {
                Some(s) if s.count > 0 => s,
                _ => continue,
            };
            let c_stats = match c_ent.fields[fi].per_step.get(h - 1) {
                Some(s) if s.count > 0 => s,
                _ => continue,
            };
            let o_mean = o_stats.mean();
            let c_mean = c_stats.mean();
            // C++ default arena has noBallRot: ball rot is echoed, not
            // simulated — don't rank it.
            let cpp_no_rot =
                is_ball && matches!(field, Field::RotFwd | Field::RotUp) && c_mean < 1e-6;
            let ratio = if c_mean > 1e-9 {
                o_mean / c_mean
            } else {
                o_mean * 1e9
            };
            let verdict = if cpp_no_rot {
                "(cpp noBallRot: rot not simulated)"
            } else if o_mean.max(c_mean) < 1e-3 {
                ""
            } else if ratio > 1.05 {
                "<- C++ BETTER"
            } else if ratio < 0.95 {
                "<- OURS BETTER"
            } else {
                ""
            };
            rows.push((
                if cpp_no_rot { -1.0 } else { ratio as f32 },
                format!(
                    "{:>7} {:<7}: ours mean={:.4} | cpp mean={:.4} | ratio {:.2}x {}",
                    o_ent.label,
                    field.label(),
                    o_mean,
                    c_mean,
                    ratio,
                    verdict,
                ),
            ));
        }
    }
    rows.sort_by(|a, b| b.0.partial_cmp(&a.0).unwrap());
    for (_, line) in rows {
        lines.push(format!("[{}] {}", ours.name, line));
    }
    lines
}

/// Side-by-side summary: for every entity+field, our gated percentile vs the
/// C++ sim's, sorted worst-first (largest ratio = C++ nails it, we don't).
pub fn comparison_lines(ours: &mut Report, cpp: &mut Report, cfg: &HarnessConfig) -> Vec<String> {
    let p = cfg.tolerance.percentile;
    let mut rows: Vec<(f32, String)> = Vec::new();

    let n_ent = ours.entities.len().min(cpp.entities.len());
    for ei in 0..n_ent {
        let label = ours.entities[ei].label.clone();
        let is_ball = !ours.entities[ei].is_car;
        for field in Field::ALL {
            let fi = super::measure::field_index(field);
            let o_p = ours.entities[ei].fields[fi].overall.percentile(p);
            let c_p = cpp.entities[ei].fields[fi].overall.percentile(p);
            let o_mean = ours.entities[ei].fields[fi].overall.run.mean();
            let c_mean = cpp.entities[ei].fields[fi].overall.run.mean();
            // C++ RocketSim's default ArenaConfig has noBallRot=true: the ball
            // rigid body never integrates rotation (bullet m_noRot), so the
            // restored rot is echoed back unchanged. A 0.0000 "error" there is
            // not accuracy — skip ranking it.
            let cpp_no_rot = is_ball && matches!(field, Field::RotFwd | Field::RotUp) && c_p < 1e-6;
            // Ratio > 1: C++ is more accurate. Guard against div-by-zero.
            let ratio = if c_p > 1e-6 { o_p / c_p } else { o_p * 1e6 };
            // Only call a winner when the difference is above noise floor.
            let verdict = if cpp_no_rot {
                "(cpp noBallRot: rot not simulated)"
            } else if o_p.max(c_p) < 1e-3 {
                ""
            } else if ratio > 1.05 {
                "<- C++ BETTER"
            } else if ratio < 0.95 {
                "<- OURS BETTER"
            } else {
                ""
            };
            rows.push((
                if cpp_no_rot { -1.0 } else { ratio },
                format!(
                    "{:>7} {:<7}: ours p={:.4} (mean {:.4}) | cpp p={:.4} (mean {:.4}) | ratio {:.2}x {}",
                    label,
                    field.label(),
                    o_p,
                    o_mean,
                    c_p,
                    c_mean,
                    ratio,
                    verdict,
                ),
            ));
        }
    }

    rows.sort_by(|a, b| b.0.partial_cmp(&a.0).unwrap());
    std::iter::once(format!(
        "==== CPP-COMPARE {} | ours vs C++ RocketSim p{:0>2} (ratio>1 = C++ more accurate) ====",
        ours.name,
        (p * 100.0) as i32,
    ))
    .chain(
        rows.into_iter()
            .map(|(_, line)| format!("[{}] {}", ours.name, line)),
    )
    .collect()
}

// ── Direct sim-vs-sim comparison ──
//
// The per-tick and rollout passes score each sim against the Rocket League
// recording and compare the scores. That is indirect: two sims can miss RL in
// different ways yet look "similar". The direct pass restores the *identical*
// recorded state into both sims, steps both once, and diffs every CarState
// field ours-vs-cpp — the true port-equivalence check.

/// Vector fields diffed directly (magnitude of the per-tick difference).
const DIRECT_VEC: &[&str] = &[
    "pos",
    "vel",
    "ang_vel",
    "rot_fwd",
    "rot_right",
    "rot_up",
    "flip_rel_torque",
    "world_normal",
];
/// Scalar fields diffed directly. `supersonic_timer` compares Rust's
/// `supersonic_grace_timer` against C++'s `supersonic_time` (both count the
/// supersonic-maintain window; the names differ).
const DIRECT_FLOAT: &[&str] = &[
    "jump_time",
    "flip_time",
    "air_time",
    "air_time_since_jump",
    "boost",
    "handbrake_val",
    "boosting_time",
    "time_since_boosted",
    "auto_flip_timer",
    "auto_flip_torque_scale",
    "demo_respawn_timer",
    "bump_cooldown",
    "supersonic_timer",
];
/// Flag fields diffed directly (mismatch counts). `wheels` counts per-wheel
/// mismatches (4 per tick); `world_has_contact` compares Rust's
/// `world_contact_normal.is_some()` against C++'s `world_contact.has_contact`.
const DIRECT_BOOL: &[&str] = &[
    "is_on_ground",
    "has_jumped",
    "has_double_jumped",
    "has_flipped",
    "is_flipping",
    "is_jumping",
    "is_boosting",
    "is_supersonic",
    "is_demoed",
    "is_auto_flipping",
    "wheels",
    "world_has_contact",
];

#[derive(Debug, Clone, Copy, Default)]
struct DirectAcc {
    count: u64,
    sum: f64,
    max: f32,
}

impl DirectAcc {
    fn add(&mut self, v: f32) {
        self.count += 1;
        self.sum += v.abs() as f64;
        self.max = self.max.max(v.abs());
    }

    fn mean(&self) -> f64 {
        if self.count == 0 {
            0.0
        } else {
            self.sum / self.count as f64
        }
    }

    fn merge(&mut self, other: Self) {
        self.count += other.count;
        self.sum += other.sum;
        self.max = self.max.max(other.max);
    }
}

#[derive(Debug, Clone, Copy, Default)]
struct DirectBoolAcc {
    total: u64,
    mismatches: u64,
}

impl DirectBoolAcc {
    fn add(&mut self, o: bool, c: bool) {
        self.total += 1;
        if o != c {
            self.mismatches += 1;
        }
    }

    fn merge(&mut self, other: Self) {
        self.total += other.total;
        self.mismatches += other.mismatches;
    }
}

#[derive(Debug, Clone)]
pub struct DirectEntity {
    pub label: String,
    pub is_car: bool,
    pub vecs: Vec<DirectAcc>,
    pub floats: Vec<DirectAcc>,
    pub bools: Vec<DirectBoolAcc>,
}

impl DirectEntity {
    fn new(label: String, is_car: bool) -> Self {
        Self {
            label,
            is_car,
            vecs: vec![DirectAcc::default(); DIRECT_VEC.len()],
            floats: vec![DirectAcc::default(); DIRECT_FLOAT.len()],
            bools: vec![DirectBoolAcc::default(); DIRECT_BOOL.len()],
        }
    }

    fn merge(&mut self, other: DirectEntity) {
        for (mine, theirs) in self.vecs.iter_mut().zip(other.vecs) {
            mine.merge(theirs);
        }
        for (mine, theirs) in self.floats.iter_mut().zip(other.floats) {
            mine.merge(theirs);
        }
        for (mine, theirs) in self.bools.iter_mut().zip(other.bools) {
            mine.merge(theirs);
        }
    }
}

#[derive(Debug, Clone)]
pub struct DirectReport {
    pub name: String,
    pub num_cars: usize,
    pub ticks_measured: u64,
    pub entities: Vec<DirectEntity>,
}

impl DirectReport {
    fn new(name: String, num_cars: usize) -> Self {
        let mut entities = Vec::with_capacity(num_cars + 1);
        for i in 0..num_cars {
            entities.push(DirectEntity::new(format!("car_{i}"), true));
        }
        entities.push(DirectEntity::new("ball".to_string(), false));
        Self {
            name,
            num_cars,
            ticks_measured: 0,
            entities,
        }
    }
}

fn record_direct_car(ent: &mut DirectEntity, o: &rocketsim::CarState, c: &CarState) {
    let d = |a: Vec3A, b: Vec3| (a - vec3a(b)).length();
    ent.vecs[0].add(d(o.phys.pos, c.pos));
    ent.vecs[1].add(d(o.phys.vel, c.vel));
    ent.vecs[2].add(d(o.phys.ang_vel, c.ang_vel));
    ent.vecs[3].add(d(o.phys.rot_mat.col(0), c.rot_mat.forward));
    ent.vecs[4].add(d(o.phys.rot_mat.col(1), c.rot_mat.right));
    ent.vecs[5].add(d(o.phys.rot_mat.col(2), c.rot_mat.up));
    ent.vecs[6].add(d(o.flip_rel_torque, c.flip_rel_torque));
    match (o.world_contact_normal, c.world_contact.has_contact) {
        (Some(n), true) => ent.vecs[7].add(d(n, c.world_contact.contact_normal)),
        _ => {}
    }

    ent.floats[0].add(o.jump_time() - c.jump_time);
    ent.floats[1].add(o.flip_time - c.flip_time);
    ent.floats[2].add(o.air_time - c.air_time);
    ent.floats[3].add(o.air_time_since_jump - c.air_time_since_jump);
    ent.floats[4].add(o.boost - c.boost);
    ent.floats[5].add(o.handbrake_val - c.handbrake_val);
    ent.floats[6].add(o.boosting_time - c.boosting_time);
    ent.floats[7].add(o.time_since_boosted - c.time_since_boosted);
    ent.floats[8].add(o.auto_flip_timer - c.auto_flip_timer);
    ent.floats[9].add(o.auto_flip_torque_scale - c.auto_flip_torque_scale);
    ent.floats[10].add(o.demo_respawn_timer - c.demo_respawn_timer);
    ent.floats[11].add(o.bump_cooldown_timer - c.car_contact.cooldown_timer);
    ent.floats[12].add(o.supersonic_grace_timer - c.supersonic_time);

    ent.bools[0].add(o.is_on_ground, c.is_on_ground);
    ent.bools[1].add(o.has_jumped, c.has_jumped);
    ent.bools[2].add(o.has_double_jumped, c.has_double_jumped);
    ent.bools[3].add(o.has_flipped, c.has_flipped);
    ent.bools[4].add(o.is_flipping, c.is_flipping);
    ent.bools[5].add(o.is_jumping, c.is_jumping);
    ent.bools[6].add(o.is_boosting, c.is_boosting);
    ent.bools[7].add(o.is_supersonic, c.is_supersonic);
    ent.bools[8].add(o.is_demoed, c.is_demoed);
    ent.bools[9].add(o.is_auto_flipping, c.is_auto_flipping);
    let wheels = &mut ent.bools[10];
    for k in 0..4 {
        wheels.add(o.wheels_with_contact[k], c.wheels_with_contact[k]);
    }
    ent.bools[11].add(
        o.world_contact_normal.is_some(),
        c.world_contact.has_contact,
    );
}

fn record_direct_ball(ent: &mut DirectEntity, o: &rocketsim::BallState, c: &BallState) {
    let d = |a: Vec3A, b: Vec3| (a - vec3a(b)).length();
    ent.vecs[0].add(d(o.phys.pos, c.pos));
    ent.vecs[1].add(d(o.phys.vel, c.vel));
    ent.vecs[2].add(d(o.phys.ang_vel, c.ang_vel));
    ent.vecs[3].add(d(o.phys.rot_mat.col(0), c.rot_mat.forward));
    ent.vecs[4].add(d(o.phys.rot_mat.col(1), c.rot_mat.right));
    ent.vecs[5].add(d(o.phys.rot_mat.col(2), c.rot_mat.up));
}

/// Restore the identical recorded state into both sims, step both once, and
/// diff every comparable output field directly (ours vs C++).
///
/// `RLDIRECT_TRACE=1` prints the jump/boost state-machine fields of car 0
/// side by side for every tick — use it on a single case to localize a
/// divergence.
pub fn run_direct_compare(recording: &Recording, _cfg: &HarnessConfig) -> DirectReport {
    init_cpp();
    let trace = matches!(
        std::env::var("RLDIRECT_TRACE").as_deref(),
        Ok("1") | Ok("true")
    );

    let num_cars = recording.info.num_cars as usize;
    let stride = recording.stride;
    let n = runner::loop_bound(recording, None);
    let starts: Vec<usize> = (0..=n).step_by(stride).collect();

    let mut report = DirectReport::new(recording.name.clone(), num_cars);
    if starts.is_empty() {
        return report;
    }

    let (mut rust_arena, rust_idcs) = runner::make_arena(num_cars);
    let (mut cpp_arena, cpp_ids) = make_cpp_arena(num_cars);
    let mut rust_controls: Vec<rocketsim::CarControls> =
        vec![rocketsim::CarControls::DEFAULT; num_cars];
    let mut cpp_controls: Vec<CarControls> = vec![CarControls::default(); num_cars];
    let mut warmup_left = runner::SHARD_WARMUP_TICKS;

    for &i in &starts {
        if runner::has_discontinuity(recording, i, stride) {
            continue;
        }

        let from_tick = &recording.ticks[i];
        let to_tick = &recording.ticks[i + stride];
        for (j, car_record) in to_tick.car_records.iter().enumerate() {
            rust_controls[j] = car_record.prev_controls.into();
            cpp_controls[j] = controls(car_record.prev_controls);
        }

        runner::set_state_to_record_tick(&mut rust_arena, &rust_idcs, from_tick, &rust_controls);
        // Align the Rust restore with the C++ one so both sims start from the
        // exact same state: the C++ restore gives world contact back from the
        // recording (the Rust runner clears it) and resets the bump cooldown.
        for (j, &car_idx) in rust_idcs.iter().enumerate() {
            let rec = &from_tick.car_records[j];
            let mut cs = *rust_arena.get_car_state(car_idx);
            cs.world_contact_normal = if rec.phys.has_world_contact {
                Some(rec.phys.world_contact_normal.into())
            } else {
                None
            };
            cs.bump_cooldown_timer = 0.0;
            rust_arena.set_car_state(car_idx, cs);
        }
        set_cpp_state_to_record_tick_impl(&mut cpp_arena, &cpp_ids, from_tick, &cpp_controls, true);

        if trace {
            let r0: rocketsim::CarState = *rust_arena.get_car_state(rust_idcs[0]);
            let c0 = cpp_arena.pin_mut().get_car(cpp_ids[0]);
            let rec = &from_tick.car_records[0];
            println!(
                "RESTORE t{:>4} | rec boost={:.4} ib={} ctrl_boost={} | ours boost={:.4} ib={} bt={:.4} | cpp boost={:.4} ib={} bt={:.4}",
                i,
                rec.boost_amount,
                rec.is_boosting as u8,
                rust_controls[0].boost as u8,
                r0.boost,
                r0.is_boosting as u8,
                r0.boosting_time,
                c0.boost,
                c0.is_boosting as u8,
                c0.boosting_time,
            );
        }

        rust_arena.step_tick();
        cpp_arena.pin_mut().step(1);

        if warmup_left > 0 {
            warmup_left -= 1;
            continue;
        }

        for (j, &car_idx) in rust_idcs.iter().enumerate() {
            let real = &to_tick.car_records[j];
            if real.is_demoed || runner::is_car_sentinel(&real.phys) {
                continue;
            }
            let o: rocketsim::CarState = *rust_arena.get_car_state(car_idx);
            let c = cpp_arena.pin_mut().get_car(cpp_ids[j]);
            record_direct_car(&mut report.entities[j], &o, &c);
            if trace && j == 0 {
                println!(
                    "TRACE t{:>4} | rec from_boost={:.4} to_boost={:.4} to_ib={} \
                     | jump  ours={} hj={} jt={:.4} | cpp={} hj={} jt={:.4} \
                     | air ours={:.4} atsj={:.4} | cpp={:.4} atsj={:.4} \
                     | boost ours={:.4} ib={} bt={:.4} | cpp={:.4} ib={} bt={:.4} \
                     | ground ours={} cpp={}",
                    i,
                    from_tick.car_records[0].boost_amount,
                    to_tick.car_records[0].boost_amount,
                    to_tick.car_records[0].is_boosting as u8,
                    o.is_jumping as u8,
                    o.has_jumped as u8,
                    o.jump_time(),
                    c.is_jumping as u8,
                    c.has_jumped as u8,
                    c.jump_time,
                    o.air_time,
                    o.air_time_since_jump,
                    c.air_time,
                    c.air_time_since_jump,
                    o.boost,
                    o.is_boosting as u8,
                    o.boosting_time,
                    c.boost,
                    c.is_boosting as u8,
                    c.boosting_time,
                    o.is_on_ground as u8,
                    c.is_on_ground as u8,
                );
            }
        }

        if !runner::is_ball_sentinel(&to_tick.ball_record) {
            let o: rocketsim::BallState = *rust_arena.get_ball_state();
            let c = cpp_arena.pin_mut().get_ball();
            record_direct_ball(&mut report.entities[num_cars], &o, &c);
        }

        report.ticks_measured += 1;
    }

    report
}

/// Render the direct ours-vs-C++ table: nonzero deltas first, then the count
/// of bit-exact fields.
pub fn direct_lines(report: &DirectReport, cfg: &HarnessConfig) -> Vec<String> {
    let mut lines = Vec::new();
    lines.push(format!(
        "==== DIRECT {} | ours vs C++ field-by-field after identical restore+step ({} ticks) ====",
        report.name, report.ticks_measured,
    ));

    let focused: Vec<usize> = match cfg.car_focus {
        Some(c) if c < report.num_cars => vec![c],
        Some(_) => vec![],
        None => (0..report.entities.len()).collect(),
    };

    for ei in focused {
        let ent = &report.entities[ei];
        let mut rows: Vec<(f32, String)> = Vec::new();
        let mut exact = 0usize;
        let mut total = 0usize;
        for (fi, name) in DIRECT_VEC.iter().enumerate() {
            let a = ent.vecs[fi];
            if a.count == 0 || (!ent.is_car && fi > 5) {
                continue;
            }
            total += 1;
            if a.max == 0.0 {
                exact += 1;
            } else {
                rows.push((
                    a.max,
                    format!(
                        "{:<18} mean={:.6} max={:.6} (n={})",
                        name,
                        a.mean(),
                        a.max,
                        a.count
                    ),
                ));
            }
        }
        if ent.is_car {
            for (fi, name) in DIRECT_FLOAT.iter().enumerate() {
                let a = ent.floats[fi];
                if a.count == 0 {
                    continue;
                }
                total += 1;
                if a.max == 0.0 {
                    exact += 1;
                } else {
                    rows.push((
                        a.max,
                        format!(
                            "{:<18} mean={:.6} max={:.6} (n={})",
                            name,
                            a.mean(),
                            a.max,
                            a.count
                        ),
                    ));
                }
            }
            for (bi, name) in DIRECT_BOOL.iter().enumerate() {
                let b = ent.bools[bi];
                if b.total == 0 {
                    continue;
                }
                total += 1;
                if b.mismatches == 0 {
                    exact += 1;
                } else {
                    rows.push((
                        b.mismatches as f32,
                        format!(
                            "{:<18} {}/{} mismatches ({:.2}%)",
                            name,
                            b.mismatches,
                            b.total,
                            b.mismatches as f64 / b.total as f64 * 100.0
                        ),
                    ));
                }
            }
        }
        rows.sort_by(|a, b| b.0.partial_cmp(&a.0).unwrap());
        lines.push(format!(
            "[{}] DIRECT {:>7}: {}/{} fields bit-exact{}",
            report.name,
            ent.label,
            exact,
            total,
            if rows.is_empty() {
                " — IDENTICAL OUTPUT".to_string()
            } else {
                String::new()
            },
        ));
        for (_, row) in rows {
            lines.push(format!(
                "[{}] DIRECT {:>7}  {}",
                report.name, ent.label, row
            ));
        }
    }

    lines
}

/// Consistency / outlier comparison.
///
/// p95 (the gate) and even the single max can hide the shape of the tail. This
/// prints the full tail side-by-side — p50/p95/p99/p99.9/max plus the fraction
/// of ticks exceeding a few absolute error thresholds — so "we're less
/// consistent" can be checked against data instead of vibes. Both sims are
/// scored against the same Rocket League recording.
pub fn tail_comparison_lines(
    ours: &mut Report,
    cpp: &mut Report,
    _cfg: &HarnessConfig,
) -> Vec<String> {
    let mut lines = Vec::new();
    lines.push(format!(
        "==== TAIL/CONSISTENCY {} | percentile & over-threshold rates vs Rocket League (lower = tighter) ====",
        ours.name,
    ));

    // Absolute error thresholds for the "how often are we badly wrong" rates.
    const THRESHOLDS: [(Field, f32); 2] = [(Field::Vel, 50.0), (Field::Pos, 1.0)];

    let n_ent = ours.entities.len().min(cpp.entities.len());
    for ei in 0..n_ent {
        let label = ours.entities[ei].label.clone();
        for field in [Field::Pos, Field::Vel] {
            let fi = super::measure::field_index(field);
            let o_f = &mut ours.entities[ei].fields[fi];
            let c_f = &mut cpp.entities[ei].fields[fi];
            if o_f.overall.run.count == 0 || c_f.overall.run.count == 0 {
                continue;
            }
            let o_ps = [
                o_f.overall.percentile(0.50),
                o_f.overall.percentile(0.95),
                o_f.overall.percentile(0.99),
                o_f.overall.percentile(0.999),
            ];
            let c_ps = [
                c_f.overall.percentile(0.50),
                c_f.overall.percentile(0.95),
                c_f.overall.percentile(0.99),
                c_f.overall.percentile(0.999),
            ];
            let o_max = o_f.overall.run.max;
            let c_max = c_f.overall.run.max;
            let thresh = THRESHOLDS
                .iter()
                .find(|(f, _)| *f == field)
                .map(|(_, t)| *t)
                .unwrap_or(1.0);
            let o_over = o_f.overall.fraction_over(thresh) * 100.0;
            let c_over = c_f.overall.fraction_over(thresh) * 100.0;

            lines.push(format!(
                "[{}] TAIL {:>7} {:<4}: p50/p95/p99/p99.9  ours {:>7.3}/{:>7.3}/{:>8.2}/{:>8.2} max {:>8.1} | cpp {:>7.3}/{:>7.3}/{:>8.2}/{:>8.2} max {:>8.1}",
                ours.name, label, field.label(),
                o_ps[0], o_ps[1], o_ps[2], o_ps[3], o_max,
                c_ps[0], c_ps[1], c_ps[2], c_ps[3], c_max,
            ));
            lines.push(format!(
                "[{}] TAIL {:>7} {:<4}: >{:>4.0} {}  ours {:6.2}%  |  cpp {:6.2}%   ({})",
                ours.name,
                label,
                field.label(),
                thresh,
                field.unit(),
                o_over,
                c_over,
                if o_over > c_over * 1.1 {
                    "C++ fewer bad ticks"
                } else if c_over > o_over * 1.1 {
                    "OURS fewer bad ticks"
                } else {
                    "~same"
                },
            ));
        }
    }
    lines
}
