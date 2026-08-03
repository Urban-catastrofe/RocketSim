//! Residual-force analysis (`RLRESID=1`).
//!
//! For every tick it restores the recorded state, steps the sim, and compares
//! the *game's* per-tick velocity change (`Δvel = rec[to] − rec[from]`) against
//! the *sim's* (`Δvel = sim_after − rec[from]`). The residual
//! (`sim_after − rec[to]`) is binned by physics regime and by contact with the
//! ball, and printed as mean per-tick vectors.
//!
//! A large mean residual in one bin means the sim applies a wrong force/impulse
//! for that subsystem. `bias ≈ rms` in the regular report is the same signal;
//! this splits it up by *when* it happens.
//!
//! `RLRESID=2` additionally prints the per-tick game Δvel for the ball while it
//! is touching a car, so the impulse *pattern* (every tick vs every other tick)
//! is visible.

use glam::{Mat3A, Vec3A};
use rocketsim::CarControls;

use super::recording::Recording;
use super::recording::cpp_records::CarRecord;
use super::runner::{make_arena, set_state_to_record_tick};

/// Octane hitbox (the harness always spawns Octanes) + ball radius geometry.
const HITBOX_HALF: Vec3A = Vec3A::new(120.507 / 2.0, 86.6994 / 2.0, 38.6591 / 2.0);
const HITBOX_OFFSET: Vec3A = Vec3A::new(13.8757, 0.0, 20.755);
const BALL_RADIUS: f32 = 91.25;
const CONTACT_SLACK: f32 = 5.0;

/// True if the ball center is within the car's hitbox inflated by the ball
/// radius (i.e. the two are in contact). The logger's `is_touching_ball` flag
/// is unreliable across recordings, so we detect contact geometrically.
fn ball_touches_car(ball_pos: Vec3A, car_pos: Vec3A, car_rot: Mat3A) -> bool {
    let local = car_rot.transpose() * (ball_pos - car_pos) - HITBOX_OFFSET;
    let d = Vec3A::new(
        (local.x.abs() - HITBOX_HALF.x).max(0.0),
        (local.y.abs() - HITBOX_HALF.y).max(0.0),
        (local.z.abs() - HITBOX_HALF.z).max(0.0),
    );
    d.length() <= BALL_RADIUS + CONTACT_SLACK
}

fn is_ball_sentinel(phys: &super::recording::cpp_records::PhysRecord) -> bool {
    phys.pos.z < -1000.0
}

// ── per-tick Δvel accumulation ──────────────────────────────────────────────

#[derive(Default, Clone, Copy)]
struct Acc {
    n: u32,
    game: Vec3A,
    sim: Vec3A,
    resid: Vec3A,
}

impl Acc {
    fn add(&mut self, game: Vec3A, sim: Vec3A) {
        self.n += 1;
        self.game += game;
        self.sim += sim;
        self.resid += sim - game;
    }
}

fn v(name: &str, a: &Acc) {
    if a.n == 0 {
        return;
    }
    let g = a.game / a.n as f32;
    let s = a.sim / a.n as f32;
    let r = a.resid / a.n as f32;
    println!(
        "  {name:<28} n={} gameΔv=({:>8.1},{:>8.1},{:>8.1}) simΔv=({:>8.1},{:>8.1},{:>8.1}) resid=({:>8.1},{:>8.1},{:>8.1})",
        a.n, g.x, g.y, g.z, s.x, s.y, s.z, r.x, r.y, r.z
    );
}

// ── regime classification from the recording's own car record ───────────────

enum Regime {
    Grounded,
    Airborne,
    Jumping,
    Flipping,
}

fn classify(car: &CarRecord) -> Regime {
    if car.is_flipping {
        Regime::Flipping
    } else if car.is_jumping {
        Regime::Jumping
    } else if car.is_on_ground {
        Regime::Grounded
    } else {
        Regime::Airborne
    }
}

/// The wheel contact normal's z (0 = wall/ceiling, 1 = floor).
fn min_contact_z(car: &CarRecord) -> f32 {
    let mut z = 1.0f32;
    for w in &car.wheels {
        if w.has_contact {
            z = z.min(w.contact_normal.z);
        }
    }
    z
}

pub fn analyze(recording: &Recording) {
    let num_cars = recording.info.num_cars as usize;
    let stride = recording.stride;
    let (mut arena, car_idcs) = make_arena(num_cars);
    let mut controls_buf: Vec<CarControls> = vec![CarControls::DEFAULT; num_cars];
    let trace = std::env::var("RLRESID").unwrap_or_default() == "2";

    // Bins: per car, per (regime, boost, surface) → Acc.
    // Key layout: "car{i} {regime} {boost} {surface}".
    let mut cbin: std::collections::BTreeMap<String, Acc> = Default::default();
    // Ball bins: touching a car vs free.
    let mut ball_touch = Acc::default();
    let mut ball_free = Acc::default();

    let n = recording.ticks.len().saturating_sub(stride + 1);
    let mut ball_touch_traces: Vec<(usize, Vec3A, Vec3A)> = Vec::new();

    for i in (0..=n).step_by(stride) {
        let from_tick = &recording.ticks[i];
        let to_tick = &recording.ticks[i + stride];

        for (j, car_record) in to_tick.car_records.iter().enumerate() {
            controls_buf[j] = car_record.prev_controls.into();
        }

        set_state_to_record_tick(&mut arena, &car_idcs, from_tick, &controls_buf);
        arena.step_tick();

        for (j, &car_idx) in car_idcs.iter().enumerate() {
            let from_car = &from_tick.car_records[j];
            let to_car = &to_tick.car_records[j];
            if to_car.is_demoed {
                continue;
            }

            let cs = *arena.get_car_state(car_idx);
            let game_dv = Vec3A::from(to_car.phys.lin_vel) - Vec3A::from(from_car.phys.lin_vel);
            let sim_dv = cs.phys.vel - Vec3A::from(from_car.phys.lin_vel);

            let regime = match classify(from_car) {
                Regime::Grounded => "ground",
                Regime::Airborne => "air",
                Regime::Jumping => "jump",
                Regime::Flipping => "flip",
            };
            let boost = if from_car.is_boosting { "boost" } else { "b0" };
            let surface = if min_contact_z(from_car) < 0.5 {
                "wall"
            } else {
                "flat"
            };
            let key = format!("car{j} {regime} {boost} {surface}");
            cbin.entry(key).or_default().add(game_dv, sim_dv);
        }

        // Ball ──
        if !is_ball_sentinel(&to_tick.ball_record) {
            let ball_pos = Vec3A::from(from_tick.ball_record.pos);
            let touching = from_tick.car_records.iter().any(|c| {
                ball_touches_car(
                    ball_pos,
                    Vec3A::from(c.phys.pos),
                    Mat3A::from_cols(
                        c.phys.rot.rows[0].into(),
                        c.phys.rot.rows[1].into(),
                        c.phys.rot.rows[2].into(),
                    ),
                )
            });
            let bs = *arena.get_ball_state();
            let game_dv = Vec3A::from(to_tick.ball_record.lin_vel) - Vec3A::from(from_tick.ball_record.lin_vel);
            let sim_dv = bs.phys.vel - Vec3A::from(from_tick.ball_record.lin_vel);
            if touching {
                ball_touch.add(game_dv, sim_dv);
                if trace {
                    ball_touch_traces.push((i, game_dv, sim_dv));
                }
            } else {
                ball_free.add(game_dv, sim_dv);
            }
        }
    }

    println!(
        "[{}] RESIDUAL (Δv per tick, uu/s; game=recording, sim=sim, resid=sim-game)",
        recording.name
    );
    for (key, acc) in &cbin {
        v(key, acc);
    }
    println!("[{}] RESIDUAL ball:", recording.name);
    v("ball_touch", &ball_touch);
    v("ball_free", &ball_free);

    if trace && !ball_touch_traces.is_empty() {
        println!("[{}] RESIDUAL ball-touch trace (tick, gameΔv, simΔv):", recording.name);
        for (t, g, s) in ball_touch_traces.iter().take(40) {
            println!(
                "  t={t:<5} gameΔv=({:>8.1},{:>8.1},{:>8.1}) simΔv=({:>8.1},{:>8.1},{:>8.1})",
                g.x, g.y, g.z, s.x, s.y, s.z
            );
        }
    }
}

// ── hit-cadence analysis (RLRESID=3) ─────────────────────────────────────────
//
// Uses the v3 hit records (the game's OnHitBall events) to settle the
// same-tick vs delayed impulse question. For each hit we know the ball velocity
// *before* the impulse (`ball_vel_before`). We compare it against the tick-end
// ball velocity (same tick) and the next tick's (delayed). The tick that
// carries the bulk of the velocity change is where the game applies the impulse.

const GRAVITY_PER_TICK: f32 = -650.0 / 120.0;

pub fn analyze_cadence(recording: &Recording) {
    let num_cars = recording.info.num_cars as usize;
    let stride = recording.stride;
    let mut hits: Vec<(usize, usize, Vec3A, Vec3A, Vec3A)> = Vec::new();
    let mut same_tick = Acc::default();
    let mut next_tick = Acc::default();

    for i in (0..recording.ticks.len().saturating_sub(stride + 1)).step_by(stride) {
        for (j, car) in recording.ticks[i].car_records.iter().enumerate() {
            if !car.hit.has_hit {
                continue;
            }
            let before = Vec3A::from(car.hit.ball_vel_before);
            // Same-tick change: hit-tick ball velocity minus pre-impulse velocity.
            let same_vel = Vec3A::from(recording.ticks[i].ball_record.lin_vel);
            let mut dv_same = same_vel - before;
            dv_same.z -= GRAVITY_PER_TICK;
            // Next-tick change: next tick's ball velocity minus the hit-tick's.
            let next = recording.ticks.get(i + stride).map(|t| Vec3A::from(t.ball_record.lin_vel));
            let mut dv_next = Vec3A::ZERO;
            if let Some(next_vel) = next {
                dv_next = next_vel - same_vel;
                dv_next.z -= GRAVITY_PER_TICK;
            }
            same_tick.add(dv_same, Vec3A::ZERO);
            next_tick.add(dv_next, Vec3A::ZERO);
            hits.push((i, j, before, dv_same, dv_next));
        }
    }

    if hits.is_empty() {
        println!("[{}] HIT CADENCE: no v3 hit records (re-record with the v3 logger)", recording.name);
        return;
    }

    let n = hits.len() as f32;
    let mut same_share_sum = 0.0f32;
    let mut same_first = 0usize;
    for (_, _, _, dv_same, dv_next) in &hits {
        let s = dv_same.length();
        let nx = dv_next.length();
        if s + nx > 1e-6 {
            same_share_sum += s / (s + nx);
        }
        if s > nx {
            same_first += 1;
        }
    }

    println!(
        "[{}] HIT CADENCE (n={}): same-tick mean share {:.3} | {}/{} hits carry the impulse in the hit tick",
        recording.name,
        hits.len(),
        same_share_sum / n,
        same_first,
        hits.len()
    );
    for (t, j, before, dv_same, dv_next) in hits.iter().take(20) {
        println!(
            "  t={t:<5} car{j} | ball_vel_before=({:>7.1},{:>7.1},{:>7.1}) | same-tick Δv=({:>7.1},{:>7.1},{:>7.1}) | next-tick Δv=({:>7.1},{:>7.1},{:>7.1})",
            before.x, before.y, before.z,
            dv_same.x, dv_same.y, dv_same.z,
            dv_next.x, dv_next.y, dv_next.z
        );
    }
    let _ = num_cars;
}

// ── boost acceleration audit (RLRESID=10) ─────────────────────────────────────
//
// Measures the game's vs sim's per-tick forward acceleration while boosting on
// the ground, bucketed by speed. The sim's boost accel is a constant
// (ACCEL_GROUND = 2975/3 ≈ 991.7 uu/s²); the game's may differ with speed.

pub fn analyze_boost(recording: &Recording) {
    let num_cars = recording.info.num_cars as usize;
    let stride = recording.stride;
    let (mut arena, car_idcs) = make_arena(num_cars);
    let mut controls_buf: Vec<CarControls> = vec![CarControls::DEFAULT; num_cars];

    // Buckets by starting speed: (game_fwd_accel, sim_fwd_accel, n)
    let mut buckets: Vec<(f32, f32, u32)> = vec![(0., 0., 0); 8];

    for i in (0..recording.ticks.len().saturating_sub(stride + 1)).step_by(stride) {
        let from_tick = &recording.ticks[i];
        let to_tick = &recording.ticks[i + stride];
        for (j, car_record) in to_tick.car_records.iter().enumerate() {
            controls_buf[j] = car_record.prev_controls.into();
        }
        set_state_to_record_tick(&mut arena, &car_idcs, from_tick, &controls_buf);
        arena.step_tick();

        for (j, &car_idx) in car_idcs.iter().enumerate() {
            let to_car = &to_tick.car_records[j];
            if to_car.is_demoed {
                continue;
            }
            let prev = &to_car.prev_controls;
            if !prev.boost || !to_car.is_on_ground {
                continue;
            }
            let from_car = &from_tick.car_records[j];
            let fwd = glam::Vec3A::from(to_car.phys.rot.rows[0]);
            let game_dv = glam::Vec3A::from(to_car.phys.lin_vel) - glam::Vec3A::from(from_car.phys.lin_vel);
            let game_fwd = game_dv.dot(fwd);
            let sim_dv = arena.get_car_state(car_idx).phys.vel - glam::Vec3A::from(from_car.phys.lin_vel);
            let sim_fwd = sim_dv.dot(fwd);
            let speed = glam::Vec3A::from(from_car.phys.lin_vel).length();
            let b = ((speed / 500.0) as usize).min(7);
            let t = &mut buckets[b];
            t.0 += game_fwd;
            t.1 += sim_fwd;
            t.2 += 1;
        }
    }

    println!("[{}] BOOST AUDIT (grounded, per-tick fwd Δv): speed | game | sim | n", recording.name);
    for (b, (g, s, n)) in buckets.iter().enumerate() {
        if *n == 0 {
            continue;
        }
        let nf = *n as f32;
        println!(
            "  [{:>4},{:>4})  game={:>+7.3}  sim={:>+7.3}  n={}",
            b * 500,
            (b + 1) * 500,
            g / nf,
            s / nf,
            n
        );
    }
}

// ── car speed audit (RLRESID=9) ──────────────────────────────────────────────
//
// The sim's car caps at ~1410 uu/s with throttle (DRIVE_SPEED_TORQUE_FACTOR
// drops to 0 at 1410), but the game's car reaches 2300. This breaks non-boost
// driving at high speed. This audit reports the game's vs sim's speed on
// throttling (non-boosting) ticks.

pub fn analyze_car_speed(recording: &Recording) {
    let num_cars = recording.info.num_cars as usize;
    let stride = recording.stride;
    let (mut arena, car_idcs) = make_arena(num_cars);
    let mut controls_buf: Vec<CarControls> = vec![CarControls::DEFAULT; num_cars];

    // Per-car: (game_speed, sim_speed, game_max_speed, sim_max_speed, n)
    let mut acc = vec![(0.0f32, 0.0f32, 0.0f32, 0.0f32, 0u32); num_cars];

    for i in (0..recording.ticks.len().saturating_sub(stride + 1)).step_by(stride) {
        let from_tick = &recording.ticks[i];
        let to_tick = &recording.ticks[i + stride];
        for (j, car_record) in to_tick.car_records.iter().enumerate() {
            controls_buf[j] = car_record.prev_controls.into();
        }
        set_state_to_record_tick(&mut arena, &car_idcs, from_tick, &controls_buf);
        arena.step_tick();

        for (j, &car_idx) in car_idcs.iter().enumerate() {
            let to_car = &to_tick.car_records[j];
            if to_car.is_demoed {
                continue;
            }
            // Throttling, not boosting, not airborne: the pure-throttle regime.
            let prev = &to_car.prev_controls;
            if prev.throttle <= 0.0 || prev.boost || !to_car.is_on_ground {
                continue;
            }
            let game_speed = glam::Vec3A::from(to_car.phys.lin_vel).length();
            let sim_speed = arena.get_car_state(car_idx).phys.vel.length();
            let a = &mut acc[j];
            a.0 += game_speed;
            a.1 += sim_speed;
            a.2 = a.2.max(game_speed);
            a.3 = a.3.max(sim_speed);
            a.4 += 1;
        }
    }

    println!("[{}] CAR SPEED AUDIT (throttle, no boost, grounded): car | game speed | sim speed | game max | sim max | n", recording.name);
    for (j, (gs, ss, gmx, smx, n)) in acc.iter().enumerate() {
        if *n == 0 {
            continue;
        }
        let nf = *n as f32;
        println!(
            "  car{j} | {:>8.1} | {:>8.1} | {:>8.1} | {:>8.1} | n={}",
            gs / nf, ss / nf, gmx, smx, n
        );
    }
}

// ── grounded car z-velocity audit (RLRESID=8) ────────────────────────────────
//
// The residual z-bias on grounded cars is the dominant compounding error. This
// prints the game's vs the sim's per-tick z-velocity on grounded ticks, so the
// target (game) and the current (sim) are both visible.

pub fn analyze_car_z(recording: &Recording) {
    let num_cars = recording.info.num_cars as usize;
    let stride = recording.stride;
    let (mut arena, car_idcs) = make_arena(num_cars);
    let mut controls_buf: Vec<CarControls> = vec![CarControls::DEFAULT; num_cars];

    // Per-car accumulators: (game_z_vel, sim_z_vel, game_z_dv, sim_z_dv, n)
    let mut acc = vec![(0.0f32, 0.0f32, 0.0f32, 0.0f32, 0u32); num_cars];

    for i in (0..recording.ticks.len().saturating_sub(stride + 1)).step_by(stride) {
        let from_tick = &recording.ticks[i];
        let to_tick = &recording.ticks[i + stride];
        for (j, car_record) in to_tick.car_records.iter().enumerate() {
            controls_buf[j] = car_record.prev_controls.into();
        }
        set_state_to_record_tick(&mut arena, &car_idcs, from_tick, &controls_buf);
        arena.step_tick();

        for (j, &car_idx) in car_idcs.iter().enumerate() {
            let to_car = &to_tick.car_records[j];
            if to_car.is_demoed || !to_car.is_on_ground {
                continue;
            }
            let game_z = to_car.phys.lin_vel.z;
            let game_prev_z = from_tick.car_records[j].phys.lin_vel.z;
            let sim_z = arena.get_car_state(car_idx).phys.vel.z;
            let a = &mut acc[j];
            a.0 += game_z;
            a.1 += sim_z;
            a.2 += game_z - game_prev_z;
            a.3 += sim_z - game_prev_z;
            a.4 += 1;
        }
    }

    println!("[{}] CAR Z AUDIT (grounded ticks): car | game z_vel | sim z_vel | game z_dv/tick | sim z_dv/tick | n", recording.name);
    for (j, (gz, sz, gd, sd, n)) in acc.iter().enumerate() {
        if *n == 0 {
            continue;
        }
        let nf = *n as f32;
        println!(
            "  car{j} | {:>8.3} | {:>8.3} | {:>+8.3} | {:>+8.3} | n={}",
            gz / nf, sz / nf, gd / nf, sd / nf, n
        );
    }
}

// ── impulse calibration (RLRESID=4) ──────────────────────────────────────────
//
// Uses the v3 hit records to measure the game's ACTUAL extra-hit impulse per
// contact: the ball's velocity change in the hit tick (minus gravity) is the
// game's applied impulse. The effective factor = |impulse| / rel_vel_mag from
// the hit record. This is the ground truth the `BALL_CAR_EXTRA_IMPULSE_FACTOR`
// curve must reproduce — compare it against the sim's current curve and fit.
//
// RLRESID=4 prints a per-hit table; RLRESID=5 prints only the curve fit.
// RLRESID=6 buckets by closing_speed ((v_car - v_ball)·hit_normal) instead of
// rel_vel_mag, since the impulse physically scales with the normal component.

pub fn analyze_impulse(recording: &Recording) {
    let stride = recording.stride;
    let verbose = std::env::var("RLRESID").unwrap_or_default() != "5";
    let by_closing = std::env::var("RLRESID").unwrap_or_default() == "6";

    // (rel_speed, game_impulse_mag, same_tick)
    let mut samples: Vec<(f32, f32, bool)> = Vec::new();

    for i in (0..recording.ticks.len().saturating_sub(stride + 1)).step_by(stride) {
        for (_j, car) in recording.ticks[i].car_records.iter().enumerate() {
            if !car.hit.has_hit {
                continue;
            }
            let before = Vec3A::from(car.hit.ball_vel_before);
            let same_vel = Vec3A::from(recording.ticks[i].ball_record.lin_vel);
            let mut dv = same_vel - before;
            dv.z -= GRAVITY_PER_TICK;
            let mag = dv.length();
            let speed = if by_closing {
                car.hit.closing_speed.abs()
            } else {
                car.hit.rel_vel_mag
            };
            samples.push((speed, mag, mag > 1.0));
        }
    }

    if samples.is_empty() {
        println!("[{}] IMPULSE CALIBRATION: no v3 hit records", recording.name);
        return;
    }

    // Bucket by speed and report the mean game factor against the sim curve.
    let curve = rocketsim::consts::curves::BALL_CAR_EXTRA_IMPULSE_FACTOR;
    let input = if by_closing { "closing_speed" } else { "rel_speed" };
    println!(
        "[{}] IMPULSE CALIBRATION (n={}, input={input}): bucket | game factor (mean±σ) | sim curve | n",
        recording.name,
        samples.len()
    );
    for (lo, hi) in [(0., 500.), (500., 1000.), (1000., 1500.), (1500., 2000.),
                     (2000., 2500.), (2500., 3000.), (3000., 4000.), (4000., 6000.)] {
        let bucket: Vec<&(f32, f32, bool)> = samples
            .iter()
            .filter(|(s, _, _)| *s >= lo && *s < hi)
            .collect();
        if bucket.is_empty() {
            continue;
        }
        let n = bucket.len() as f32;
        let mean: f32 = bucket.iter().map(|(s, m, _)| m / s).sum::<f32>() / n;
        let var: f32 = bucket
            .iter()
            .map(|(s, m, _)| (m / s - mean).powi(2))
            .sum::<f32>()
            / n;
        let mid = (lo + hi) / 2.0;
        println!(
            "  [{lo:>4},{hi:>5})  game={mean:>6.3}±{:.3}  sim={:>6.3}  n={}",
            var.sqrt(),
            curve.get_output(mid),
            bucket.len()
        );
    }

    if verbose {
        println!("[{}] IMPULSE CALIBRATION per-hit ({} , game_factor, same_tick):", recording.name, input);
        for (s, m, same) in samples.iter().take(40) {
            println!(
                "  {input}={s:>7.1}  game_factor={:>6.3}  impulse={m:>7.1}  same_tick={same}",
                if *s > 0.0 { m / s } else { 0.0 }
            );
        }
    }
}

// ── sim-vs-game impulse gap (RLRESID=7) ──────────────────────────────────────
//
// Runs the sim per tick (baseline — extra impulse neutralized) and, on the
// ticks where the v3 hit records say a car hit the ball, compares:
//   game_total  = the game's same-tick ball Δv (from the hit record)  [minus gravity]
//   sim_total   = the sim's Δv on the same tick (bullet-only, extra disabled)
//   gap         = game_total - sim_total  → the extra impulse the sim must add
//
// This is the direct calibration target: if the gap ≈ 0, the sim's bullet
// collision already reproduces the game and the extra impulse is redundant.

pub fn analyze_impulse_gap(recording: &Recording) {
    let num_cars = recording.info.num_cars as usize;
    let stride = recording.stride;
    let (mut arena, car_idcs) = make_arena(num_cars);
    let mut controls_buf: Vec<CarControls> = vec![CarControls::DEFAULT; num_cars];

    let mut gap_sum = Vec3A::ZERO;
    let mut game_sum = Vec3A::ZERO;
    let mut sim_sum = Vec3A::ZERO;
    let mut n = 0u32;

    for i in (0..recording.ticks.len().saturating_sub(stride + 1)).step_by(stride) {
        // Only ticks with a hit event produce a gap sample.
        let has_hit = recording.ticks[i]
            .car_records
            .iter()
            .any(|c| c.hit.has_hit);
        if !has_hit {
            continue;
        }

        let from_tick = &recording.ticks[i];
        let to_tick = &recording.ticks[i + stride];
        for (j, car_record) in to_tick.car_records.iter().enumerate() {
            controls_buf[j] = car_record.prev_controls.into();
        }
        set_state_to_record_tick(&mut arena, &car_idcs, from_tick, &controls_buf);
        arena.step_tick();

        let bs = *arena.get_ball_state();
        let from_vel = Vec3A::from(from_tick.ball_record.lin_vel);
        let sim_dv = bs.phys.vel - from_vel;
        let to_vel = Vec3A::from(to_tick.ball_record.lin_vel);
        let mut game_dv = to_vel - from_vel;
        game_dv.z -= GRAVITY_PER_TICK;
        let mut sim_dv_g = sim_dv;
        sim_dv_g.z -= GRAVITY_PER_TICK;

        let gap = game_dv - sim_dv_g;
        gap_sum += gap;
        game_sum += game_dv;
        sim_sum += sim_dv_g;
        n += 1;
    }

    if n == 0 {
        println!("[{}] IMPULSE GAP: no v3 hit records", recording.name);
        return;
    }
    let nf = n as f32;
    let g = game_sum / nf;
    let s = sim_sum / nf;
    let gap = gap_sum / nf;
    println!(
        "[{}] IMPULSE GAP (n={n}): mean game Δv=({:>7.1},{:>7.1},{:>7.1}) | sim Δv=({:>7.1},{:>7.1},{:>7.1}) | gap(game-sim)=({:>7.1},{:>7.1},{:>7.1}) | |gap|={:.1}",
        recording.name,
        g.x, g.y, g.z,
        s.x, s.y, s.z,
        gap.x, gap.y, gap.z,
        gap.length()
    );
}