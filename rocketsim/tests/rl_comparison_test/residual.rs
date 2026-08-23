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
use rocketsim::{ArenaEvent, CarControls, consts::TICK_TIME};

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
pub(super) fn ball_touches_car(ball_pos: Vec3A, car_pos: Vec3A, car_rot: Mat3A) -> bool {
    ball_touches_car_with_slack(ball_pos, car_pos, car_rot, CONTACT_SLACK)
}

pub(super) fn ball_touches_car_exact(ball_pos: Vec3A, car_pos: Vec3A, car_rot: Mat3A) -> bool {
    ball_touches_car_with_slack(ball_pos, car_pos, car_rot, 0.0)
}

pub(super) fn ball_touches_car_with_slack(
    ball_pos: Vec3A,
    car_pos: Vec3A,
    car_rot: Mat3A,
    slack: f32,
) -> bool {
    ball_car_gap(ball_pos, car_pos, car_rot) <= slack
}

fn ball_car_gap(ball_pos: Vec3A, car_pos: Vec3A, car_rot: Mat3A) -> f32 {
    ball_car_gap_normal(ball_pos, car_pos, car_rot).0
}

fn ball_car_gap_normal(ball_pos: Vec3A, car_pos: Vec3A, car_rot: Mat3A) -> (f32, Vec3A) {
    let local = car_rot.transpose() * (ball_pos - car_pos) - HITBOX_OFFSET;
    let closest = local.clamp(-HITBOX_HALF, HITBOX_HALF);
    let delta = local - closest;
    let distance = delta.length();
    (
        distance - BALL_RADIUS,
        car_rot * delta.normalize_or(Vec3A::Z),
    )
}

fn swept_ball_car_min_gap(
    ball_pos: Vec3A,
    ball_vel: Vec3A,
    car_pos: Vec3A,
    car_vel: Vec3A,
    car_rot: Mat3A,
) -> f32 {
    let start = car_rot.transpose() * (ball_pos - car_pos) - HITBOX_OFFSET;
    let delta = car_rot.transpose() * (ball_vel - car_vel) * TICK_TIME;
    let gap_at = |t: f32| {
        let local = start + delta * t;
        (local - local.clamp(-HITBOX_HALF, HITBOX_HALF)).length() - BALL_RADIUS
    };

    // Distance to a convex box along a line segment is convex, so ternary
    // search isolates the within-tick closest approach without changing sim.
    let (mut lo, mut hi) = (0.0, 1.0);
    for _ in 0..24 {
        let third = (hi - lo) / 3.0;
        let m1 = lo + third;
        let m2 = hi - third;
        if gap_at(m1) <= gap_at(m2) {
            hi = m2;
        } else {
            lo = m1;
        }
    }
    gap_at((lo + hi) * 0.5).min(gap_at(0.0)).min(gap_at(1.0))
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

        set_state_to_record_tick(
            &mut arena,
            &car_idcs,
            from_tick,
            i.checked_sub(stride).map(|i| &recording.ticks[i]),
            &controls_buf,
        );
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
            let game_dv = Vec3A::from(to_tick.ball_record.lin_vel)
                - Vec3A::from(from_tick.ball_record.lin_vel);
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
        println!(
            "[{}] RESIDUAL ball-touch trace (tick, gameΔv, simΔv):",
            recording.name
        );
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
            let next = recording
                .ticks
                .get(i + stride)
                .map(|t| Vec3A::from(t.ball_record.lin_vel));
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
        println!(
            "[{}] HIT CADENCE: no v3 hit records (re-record with the v3 logger)",
            recording.name
        );
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
            before.x,
            before.y,
            before.z,
            dv_same.x,
            dv_same.y,
            dv_same.z,
            dv_next.x,
            dv_next.y,
            dv_next.z
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
        set_state_to_record_tick(
            &mut arena,
            &car_idcs,
            from_tick,
            i.checked_sub(stride).map(|i| &recording.ticks[i]),
            &controls_buf,
        );
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
            let game_dv =
                glam::Vec3A::from(to_car.phys.lin_vel) - glam::Vec3A::from(from_car.phys.lin_vel);
            let game_fwd = game_dv.dot(fwd);
            let sim_dv =
                arena.get_car_state(car_idx).phys.vel - glam::Vec3A::from(from_car.phys.lin_vel);
            let sim_fwd = sim_dv.dot(fwd);
            let speed = glam::Vec3A::from(from_car.phys.lin_vel).length();
            let b = ((speed / 500.0) as usize).min(7);
            let t = &mut buckets[b];
            t.0 += game_fwd;
            t.1 += sim_fwd;
            t.2 += 1;
        }
    }

    println!(
        "[{}] BOOST AUDIT (grounded, per-tick fwd Δv): speed | game | sim | n",
        recording.name
    );
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
        set_state_to_record_tick(
            &mut arena,
            &car_idcs,
            from_tick,
            i.checked_sub(stride).map(|i| &recording.ticks[i]),
            &controls_buf,
        );
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

    println!(
        "[{}] CAR SPEED AUDIT (throttle, no boost, grounded): car | game speed | sim speed | game max | sim max | n",
        recording.name
    );
    for (j, (gs, ss, gmx, smx, n)) in acc.iter().enumerate() {
        if *n == 0 {
            continue;
        }
        let nf = *n as f32;
        println!(
            "  car{j} | {:>8.1} | {:>8.1} | {:>8.1} | {:>8.1} | n={}",
            gs / nf,
            ss / nf,
            gmx,
            smx,
            n
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
        set_state_to_record_tick(
            &mut arena,
            &car_idcs,
            from_tick,
            i.checked_sub(stride).map(|i| &recording.ticks[i]),
            &controls_buf,
        );
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

    println!(
        "[{}] CAR Z AUDIT (grounded ticks): car | game z_vel | sim z_vel | game z_dv/tick | sim z_dv/tick | n",
        recording.name
    );
    for (j, (gz, sz, gd, sd, n)) in acc.iter().enumerate() {
        if *n == 0 {
            continue;
        }
        let nf = *n as f32;
        println!(
            "  car{j} | {:>8.3} | {:>8.3} | {:>+8.3} | {:>+8.3} | n={}",
            gz / nf,
            sz / nf,
            gd / nf,
            sd / nf,
            n
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
        println!(
            "[{}] IMPULSE CALIBRATION: no v3 hit records",
            recording.name
        );
        return;
    }

    // Bucket by speed and report the mean game factor against the sim curve.
    let curve = rocketsim::consts::curves::BALL_CAR_EXTRA_IMPULSE_FACTOR;
    let input = if by_closing {
        "closing_speed"
    } else {
        "rel_speed"
    };
    println!(
        "[{}] IMPULSE CALIBRATION (n={}, input={input}): bucket | game factor (mean±σ) | sim curve | n",
        recording.name,
        samples.len()
    );
    for (lo, hi) in [
        (0., 500.),
        (500., 1000.),
        (1000., 1500.),
        (1500., 2000.),
        (2000., 2500.),
        (2500., 3000.),
        (3000., 4000.),
        (4000., 6000.),
    ] {
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
        println!(
            "[{}] IMPULSE CALIBRATION per-hit ({} , game_factor, same_tick):",
            recording.name, input
        );
        for (s, m, same) in samples.iter().take(40) {
            println!(
                "  {input}={s:>7.1}  game_factor={:>6.3}  impulse={m:>7.1}  same_tick={same}",
                if *s > 0.0 { m / s } else { 0.0 }
            );
        }
    }
}

// ── sim-vs-game hit episode gap (RLRESID=7) ──────────────────────────────────
//
// Consecutive v3 OnHitBall records form one hit episode. RL commonly spreads
// the resulting velocity change into the following frame, so comparing only
// the event tick mostly measures frame phase. Replay from two ticks before each
// episode (enough to close a sub-frame contact gap) through one tick after it
// and compare the complete velocity change.

pub fn analyze_impulse_gap(recording: &Recording) {
    let num_cars = recording.info.num_cars as usize;
    let stride = recording.stride;
    let mut gap_sum = Vec3A::ZERO;
    let mut game_sum = Vec3A::ZERO;
    let mut sim_sum = Vec3A::ZERO;
    let mut gap_mag_sum = 0.0f32;
    let mut n = 0u32;
    let mut unmeasurable = 0u32;

    let has_hit = |i: usize| {
        recording.ticks[i]
            .car_records
            .iter()
            .any(|car| car.hit.has_hit)
    };
    let touching = |i: usize| {
        let tick = &recording.ticks[i];
        let ball_pos = Vec3A::from(tick.ball_record.pos);
        tick.car_records.iter().any(|car| {
            ball_touches_car_exact(
                ball_pos,
                car.phys.pos.into(),
                Mat3A::from_cols(
                    car.phys.rot.rows[0].into(),
                    car.phys.rot.rows[1].into(),
                    car.phys.rot.rows[2].into(),
                ),
            )
        })
    };
    let mut onset = 0;
    while onset + stride < recording.ticks.len() {
        if !has_hit(onset) {
            onset += stride;
            continue;
        }

        let onset_start = onset;
        let mut contact_start = onset;
        let mut contact_end = onset;
        if touching(onset) {
            while contact_start >= stride
                && (touching(contact_start - stride) || has_hit(contact_start - stride))
            {
                contact_start -= stride;
            }
            while contact_end + stride < recording.ticks.len()
                && (touching(contact_end + stride) || has_hit(contact_end + stride))
            {
                contact_end += stride;
            }
        } else {
            while contact_end + stride < recording.ticks.len()
                && (has_hit(contact_end + stride) || touching(contact_end + stride))
            {
                contact_end += stride;
            }
        }
        let mut last_hit = onset_start;
        for i in (onset_start..=contact_end).step_by(stride) {
            if has_hit(i) {
                last_hit = i;
            }
        }
        let start = contact_start.saturating_sub(2 * stride);
        let end = (contact_end + stride).min(recording.ticks.len() - 1);
        onset = contact_end + stride;

        if contact_start < 2 * stride {
            unmeasurable += 1;
            continue;
        }

        if (start..end)
            .step_by(stride)
            .any(|i| super::runner::has_discontinuity(recording, i, stride))
        {
            continue;
        }

        let (mut arena, car_idcs) = make_arena(num_cars);
        let mut controls_buf: Vec<CarControls> = vec![CarControls::DEFAULT; num_cars];
        for (j, car_record) in recording.ticks[start + stride]
            .car_records
            .iter()
            .enumerate()
        {
            controls_buf[j] = car_record.prev_controls.into();
        }
        let previous_tick = start.checked_sub(stride).map(|i| &recording.ticks[i]);
        for _ in 0..super::runner::SHARD_WARMUP_TICKS {
            set_state_to_record_tick(
                &mut arena,
                &car_idcs,
                &recording.ticks[start],
                previous_tick,
                &controls_buf,
            );
            arena.step_tick();
        }
        set_state_to_record_tick(
            &mut arena,
            &car_idcs,
            &recording.ticks[start],
            previous_tick,
            &controls_buf,
        );
        let mut sim_contact_ticks = Vec::new();
        let mut sim_fire_ticks = Vec::new();
        let mut sim_gaps = Vec::new();
        for target in ((start + stride)..=end).step_by(stride) {
            for (j, &car_idx) in car_idcs.iter().enumerate() {
                arena.set_car_controls(
                    car_idx,
                    recording.ticks[target].car_records[j].prev_controls.into(),
                );
            }
            arena.step_tick();
            for event in arena.get_last_step_events() {
                if let ArenaEvent::CarHitBall(hit) = event {
                    sim_contact_ticks.push(target);
                    if hit.extra_hit_vel != Vec3A::ZERO {
                        sim_fire_ticks.push(target);
                    }
                }
            }
            let ball_pos = arena.get_ball_state().phys.pos;
            let (min_gap, closing_speed) = car_idcs
                .iter()
                .map(|car_idx| {
                    let car = arena.get_car_state(*car_idx);
                    let (gap, normal) =
                        ball_car_gap_normal(ball_pos, car.phys.pos, car.phys.rot_mat);
                    let closing = (car.phys.vel - arena.get_ball_state().phys.vel).dot(normal);
                    (gap, closing)
                })
                .min_by(|a, b| a.0.total_cmp(&b.0))
                .unwrap_or((f32::INFINITY, 0.0));
            sim_gaps.push((target, min_gap, closing_speed));
        }

        let bs = *arena.get_ball_state();
        let from_vel = Vec3A::from(recording.ticks[start].ball_record.lin_vel);
        let sim_dv = bs.phys.vel - from_vel;
        let game_dv = Vec3A::from(recording.ticks[end].ball_record.lin_vel) - from_vel;

        let gap = game_dv - sim_dv;
        println!(
            "  episode {onset_start}..={last_hit} window {start}->{end}: game Δv=({:>7.1},{:>7.1},{:>7.1}) sim Δv=({:>7.1},{:>7.1},{:>7.1}) gap=({:>7.1},{:>7.1},{:>7.1}) |gap|={:.1} contact={sim_contact_ticks:?} fire={sim_fire_ticks:?} sim_gap={sim_gaps:?}",
            game_dv.x,
            game_dv.y,
            game_dv.z,
            sim_dv.x,
            sim_dv.y,
            sim_dv.z,
            gap.x,
            gap.y,
            gap.z,
            gap.length(),
        );
        gap_sum += gap;
        gap_mag_sum += gap.length();
        game_sum += game_dv;
        sim_sum += sim_dv;
        n += 1;
    }

    if n == 0 {
        println!(
            "[{}] HIT EPISODE GAP: no measurable episodes (unmeasurable={unmeasurable})",
            recording.name
        );
        return;
    }
    let nf = n as f32;
    let g = game_sum / nf;
    let s = sim_sum / nf;
    let gap = gap_sum / nf;
    println!(
        "[{}] HIT EPISODE GAP (n={n}, unmeasurable={unmeasurable}): mean game Δv=({:>7.1},{:>7.1},{:>7.1}) | sim Δv=({:>7.1},{:>7.1},{:>7.1}) | mean gap=({:>7.1},{:>7.1},{:>7.1}) | mean |gap|={:.1}",
        recording.name,
        g.x,
        g.y,
        g.z,
        s.x,
        s.y,
        s.z,
        gap.x,
        gap.y,
        gap.z,
        gap_mag_sum / nf
    );
}

// -- hit-trigger audit (RLRESID=11) -------------------------------------------
//
// Compare the game's OnHitBall labels with the sim's geometric contact events
// and with the subset that actually fire an extra impulse. This deliberately
// changes no production behavior: it separates narrowphase disagreement from
// cadence suppression before either is tuned.

pub fn analyze_hit_trigger(recording: &Recording) {
    let num_cars = recording.info.num_cars as usize;
    let stride = recording.stride;
    let (mut arena, car_idcs) = make_arena(num_cars);
    let mut controls_buf = vec![CarControls::DEFAULT; num_cars];

    let mut game_hits = 0usize;
    let mut sim_contacts = 0usize;
    let mut sim_fires = 0usize;
    let mut solver_contacts = 0usize;
    let mut solver_impulses = 0usize;
    let mut contact_tp = 0usize;
    let mut fire_tp = 0usize;
    let mut contact_fp = 0usize;
    let mut fire_fp = 0usize;
    let mut solver_tp = 0usize;
    let mut solver_fp = 0usize;
    let mut impulse_tp = 0usize;
    let mut impulse_fp = 0usize;
    let mut matched_solver_hits = 0usize;
    let mut normal_dot_sum = 0.0f32;
    let mut location_error_sum = 0.0f32;
    let mut location_normal_offset_sum = 0.0f32;
    let mut location_tangent_error_sum = 0.0f32;
    let mut solver_distance_sum = 0.0f32;
    let mut solver_pre_normal_sum = 0.0f32;
    let mut solver_post_normal_sum = 0.0f32;
    let mut solver_impulse_sum = 0.0f32;
    let mut sweep_tp = [0usize; 4];
    let mut sweep_fp = [0usize; 4];
    let mut game_sweep_gap_sum = 0.0f32;
    let mut closing_sum = 0.0f32;
    let mut rel_speed_sum = 0.0f32;
    let mut low_closing = [0usize; 3];
    let mut run_count = 0usize;
    let mut run_ticks = 0usize;
    let mut max_run = 0usize;
    let mut current_run = vec![0usize; num_cars];
    let mut run_start = vec![0usize; num_cars];
    let mut game_episodes: Vec<(usize, usize, usize)> = Vec::new();
    let mut sim_contact_ticks: Vec<Vec<usize>> = vec![Vec::new(); num_cars];
    let mut sim_fire_ticks: Vec<Vec<usize>> = vec![Vec::new(); num_cars];
    let mut solver_ticks: Vec<Vec<(usize, rocketsim::CarBallContactInfo)>> =
        vec![Vec::new(); num_cars];

    for i in (0..recording.ticks.len()).step_by(stride) {
        for (car_idx, car) in recording.ticks[i].car_records.iter().enumerate() {
            if car.hit.has_hit {
                game_hits += 1;
                closing_sum += car.hit.closing_speed.abs();
                rel_speed_sum += car.hit.rel_vel_mag;
                for (bucket, threshold) in [25.0, 100.0, 250.0].into_iter().enumerate() {
                    low_closing[bucket] += usize::from(car.hit.closing_speed.abs() < threshold);
                }
                if current_run[car_idx] == 0 {
                    run_count += 1;
                    run_start[car_idx] = i;
                }
                current_run[car_idx] += 1;
            } else if current_run[car_idx] != 0 {
                run_ticks += current_run[car_idx];
                max_run = max_run.max(current_run[car_idx]);
                game_episodes.push((car_idx, run_start[car_idx], i - stride));
                current_run[car_idx] = 0;
            }
        }
    }
    for (car_idx, run) in current_run.into_iter().enumerate() {
        if run != 0 {
            run_ticks += run;
            max_run = max_run.max(run);
            game_episodes.push((
                car_idx,
                run_start[car_idx],
                recording.ticks.len().saturating_sub(stride),
            ));
        }
    }

    for i in (0..recording.ticks.len().saturating_sub(stride)).step_by(stride) {
        let target = i + stride;
        for (j, car) in recording.ticks[target].car_records.iter().enumerate() {
            controls_buf[j] = car.prev_controls.into();
        }
        set_state_to_record_tick(
            &mut arena,
            &car_idcs,
            &recording.ticks[i],
            i.checked_sub(stride).map(|p| &recording.ticks[p]),
            &controls_buf,
        );
        arena.step_tick();

        let mut contacted = vec![false; num_cars];
        let mut fired = vec![false; num_cars];
        for event in arena.get_last_step_events() {
            if let ArenaEvent::CarHitBall(hit) = event {
                contacted[hit.car_idx] = true;
                fired[hit.car_idx] |= hit.extra_hit_vel != Vec3A::ZERO;
            }
        }
        let mut solved = vec![None; num_cars];
        for contact in arena.get_last_step_car_ball_contacts() {
            let slot = &mut solved[contact.car_idx];
            if slot.is_none_or(|current: rocketsim::CarBallContactInfo| {
                contact.normal_impulse > current.normal_impulse
            }) {
                *slot = Some(*contact);
            }
        }

        for car_idx in 0..num_cars {
            let game_hit = recording.ticks[target].car_records[car_idx].hit;
            let game = game_hit.has_hit;
            let from_car = &recording.ticks[i].car_records[car_idx].phys;
            let from_ball = &recording.ticks[i].ball_record;
            let sweep_gap = swept_ball_car_min_gap(
                from_ball.pos.into(),
                from_ball.lin_vel.into(),
                from_car.pos.into(),
                from_car.lin_vel.into(),
                Mat3A::from_cols(
                    from_car.rot.rows[0].into(),
                    from_car.rot.rows[1].into(),
                    from_car.rot.rows[2].into(),
                ),
            );
            if game {
                game_sweep_gap_sum += sweep_gap;
            }
            for (bucket, threshold) in [0.0, 1.0, 2.0, 3.0].into_iter().enumerate() {
                let swept_contact = sweep_gap <= threshold;
                sweep_tp[bucket] += usize::from(game && swept_contact);
                sweep_fp[bucket] += usize::from(!game && swept_contact);
            }
            if contacted[car_idx] {
                sim_contact_ticks[car_idx].push(target);
            }
            if fired[car_idx] {
                sim_fire_ticks[car_idx].push(target);
            }
            sim_contacts += usize::from(contacted[car_idx]);
            sim_fires += usize::from(fired[car_idx]);
            contact_tp += usize::from(game && contacted[car_idx]);
            fire_tp += usize::from(game && fired[car_idx]);
            contact_fp += usize::from(!game && contacted[car_idx]);
            fire_fp += usize::from(!game && fired[car_idx]);
            if let Some(contact) = solved[car_idx] {
                let has_impulse = contact.normal_impulse > 1e-6 || contact.push_impulse > 1e-6;
                solver_ticks[car_idx].push((target, contact));
                solver_contacts += 1;
                solver_impulses += usize::from(has_impulse);
                solver_tp += usize::from(game);
                solver_fp += usize::from(!game);
                impulse_tp += usize::from(game && has_impulse);
                impulse_fp += usize::from(!game && has_impulse);
                if game {
                    let game_normal = Vec3A::from(game_hit.hit_normal).normalize_or_zero();
                    normal_dot_sum += contact.contact_normal.dot(game_normal);
                    let location_delta = contact.contact_point - Vec3A::from(game_hit.hit_location);
                    let normal_offset = location_delta.dot(game_normal);
                    location_error_sum += location_delta.length();
                    location_normal_offset_sum += normal_offset;
                    location_tangent_error_sum +=
                        (location_delta - game_normal * normal_offset).length();
                    solver_distance_sum += contact.distance;
                    solver_pre_normal_sum +=
                        contact.relative_velocity_before.dot(contact.contact_normal);
                    solver_post_normal_sum +=
                        contact.relative_velocity_after.dot(contact.contact_normal);
                    solver_impulse_sum += contact.normal_impulse;
                    matched_solver_hits += 1;
                }
            }
        }
    }

    if game_hits == 0 {
        println!("[{}] HIT TRIGGER: no v3 hit records", recording.name);
        return;
    }

    let recall = |tp: usize| tp as f32 / game_hits as f32;
    let precision = |tp: usize, fp: usize| {
        if tp + fp == 0 {
            0.0
        } else {
            tp as f32 / (tp + fp) as f32
        }
    };
    let episode_matches = |ticks: &[Vec<usize>]| {
        game_episodes
            .iter()
            .filter(|(car, start, end)| {
                let lo = start.saturating_sub(stride);
                let hi = end.saturating_add(stride);
                ticks[*car].iter().any(|tick| *tick >= lo && *tick <= hi)
            })
            .count()
    };
    let contact_episode_matches = episode_matches(&sim_contact_ticks);
    let fire_episode_matches = episode_matches(&sim_fire_ticks);
    println!(
        "[{}] HIT TRIGGER: game={} runs={} mean_run={:.2} max_run={} | exact contact={} P={:.3} R={:.3} fire={} P={:.3} R={:.3} | episode±1 contact={}/{} fire={}/{} | mean closing={:.1} rel={:.1} | closing<25/100/250={}/{}/{}",
        recording.name,
        game_hits,
        run_count,
        run_ticks as f32 / run_count.max(1) as f32,
        max_run,
        sim_contacts,
        precision(contact_tp, contact_fp),
        recall(contact_tp),
        sim_fires,
        precision(fire_tp, fire_fp),
        recall(fire_tp),
        contact_episode_matches,
        game_episodes.len(),
        fire_episode_matches,
        game_episodes.len(),
        closing_sum / game_hits as f32,
        rel_speed_sum / game_hits as f32,
        low_closing[0],
        low_closing[1],
        low_closing[2],
    );
    let matched = matched_solver_hits.max(1) as f32;
    println!(
        "[{}] SOLVED CONTACT: contact={} P={:.3} R={:.3} impulse={} P={:.3} R={:.3} | matched={} mean normal_dot={:.3} location_err={:.2} normal_offset={:+.2} tangent_err={:.2} distance={:+.3} pre_n={:+.1} post_n={:+.1} impulse={:.4} | hitbox_bt min={:?} max={:?}",
        recording.name,
        solver_contacts,
        precision(solver_tp, solver_fp),
        recall(solver_tp),
        solver_impulses,
        precision(impulse_tp, impulse_fp),
        recall(impulse_tp),
        matched_solver_hits,
        normal_dot_sum / matched,
        location_error_sum / matched,
        location_normal_offset_sum / matched,
        location_tangent_error_sum / matched,
        solver_distance_sum / matched,
        solver_pre_normal_sum / matched,
        solver_post_normal_sum / matched,
        solver_impulse_sum / matched,
        Vec3A::from(recording.info.hitbox_rel_min_bt),
        Vec3A::from(recording.info.hitbox_rel_max_bt),
    );
    println!(
        "[{}] SWEPT CONTACT: mean game min_gap={:+.2} | threshold 0/1/2/3 P={:.3}/{:.3}/{:.3}/{:.3} R={:.3}/{:.3}/{:.3}/{:.3}",
        recording.name,
        game_sweep_gap_sum / game_hits as f32,
        precision(sweep_tp[0], sweep_fp[0]),
        precision(sweep_tp[1], sweep_fp[1]),
        precision(sweep_tp[2], sweep_fp[2]),
        precision(sweep_tp[3], sweep_fp[3]),
        recall(sweep_tp[0]),
        recall(sweep_tp[1]),
        recall(sweep_tp[2]),
        recall(sweep_tp[3]),
    );

    if std::env::var("RLTRIGGER_VERBOSE").is_ok() {
        for (car_idx, start, end) in game_episodes {
            let hit_ticks: Vec<usize> = (start..=end).step_by(stride).collect();
            let mean_closing = hit_ticks
                .iter()
                .map(|tick| {
                    recording.ticks[*tick].car_records[car_idx]
                        .hit
                        .closing_speed
                        .abs()
                })
                .sum::<f32>()
                / hit_ticks.len() as f32;
            let mean_rel = hit_ticks
                .iter()
                .map(|tick| recording.ticks[*tick].car_records[car_idx].hit.rel_vel_mag)
                .sum::<f32>()
                / hit_ticks.len() as f32;
            let lo = start.saturating_sub(stride);
            let hi = end.saturating_add(stride);
            let contacts: Vec<usize> = sim_contact_ticks[car_idx]
                .iter()
                .copied()
                .filter(|tick| *tick >= lo && *tick <= hi)
                .collect();
            let fires: Vec<usize> = sim_fire_ticks[car_idx]
                .iter()
                .copied()
                .filter(|tick| *tick >= lo && *tick <= hi)
                .collect();
            let solved: Vec<(usize, f32, f32, f32, f32)> = solver_ticks[car_idx]
                .iter()
                .filter(|(tick, _)| *tick >= lo && *tick <= hi)
                .map(|(tick, contact)| {
                    (
                        *tick,
                        contact.distance,
                        contact.normal_impulse,
                        contact.relative_velocity_before.dot(contact.contact_normal),
                        contact.relative_velocity_after.dot(contact.contact_normal),
                    )
                })
                .collect();
            let geometry: Vec<(usize, f32)> = (lo..=hi)
                .step_by(stride)
                .map(|tick| {
                    let car = &recording.ticks[tick].car_records[car_idx].phys;
                    (
                        tick,
                        ball_car_gap(
                            recording.ticks[tick].ball_record.pos.into(),
                            car.pos.into(),
                            Mat3A::from_cols(
                                car.rot.rows[0].into(),
                                car.rot.rows[1].into(),
                                car.rot.rows[2].into(),
                            ),
                        ),
                    )
                })
                .collect();
            println!(
                "  car{car_idx} game={start}..={end} closing={mean_closing:.1} rel={mean_rel:.1} | sim contact={contacts:?} fire={fires:?} solved(tick,distance,impulse,pre_n,post_n)={solved:?} | game gap={geometry:?}"
            );
        }
    }
}
