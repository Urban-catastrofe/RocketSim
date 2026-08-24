//! Per-tick impulse ledger accounting (`RLLEDGER=1|2`).
//!
//! Every velocity change the sim makes in a tick is recorded by name in
//! `RigidBody::dbg_tick_impulse_history`. This pass restores the recorded
//! state, steps once, and reads the ledger back, so a tick's velocity change
//! arrives decomposed by *source* instead of as a single number. Every other
//! pass in this harness reports how wrong the sim is; this one reports which
//! force it was wrong about.
//!
//! The accounting identity is
//!
//! ```text
//! v_after - v_before  ==  sum of ledger entries  +  closure
//! ```
//!
//! `closure` is the constraint solver. Contact and friction constraints write
//! back in aggregate (`body.set_lin_vel(solver.lin_vel + external_force_impulse)`
//! in `seq_impulse_constraint_solver`), so they are the one channel the ledger
//! cannot name. Defining them as the remainder makes them *exactly* measurable
//! without tagging individual constraints — but only while every other velocity
//! change is named, which is what `RLLEDGER=1` checks.
//!
//! The check is a 2x2 of "did the sim report a contact for this car" against
//! "is the closure nonzero":
//!
//! | | closure == 0 | closure != 0 |
//! |---|---|---|
//! | **no contact event** | solver idle, fully accounted | **SUSPECT** |
//! | **contact event** | contact with no velocity effect | solver contact, measured |
//!
//! Only the SUSPECT cell matters. It means *either* a velocity change no ledger
//! entry names — in which case every attribution built on the ledger is wrong
//! until it is found — *or* a sustained contact whose manifold points were not
//! re-added this tick, so no event fired while the solver still acted. The two
//! are not separable from here, which is why this reports a count and the worst
//! offenders rather than asserting. A suspect count that tracks contact ticks
//! is the second explanation; suspects on airborne, far-from-everything ticks
//! are the first.
//!
//! `RLLEDGER=2` dumps one row per car-tick — Rocket League's own velocity
//! change, ours, every named source, and the state regressors — for the offline
//! fit that turns those into a per-source scale factor.

use glam::Vec3A;
use rocketsim::{ArenaEvent, CarControls, consts::BT_TO_UU};

use super::recording::Recording;
use super::runner::{is_car_sentinel, make_arena, set_state_to_record_tick};

/// Ledger entries whose name starts with this are informational decompositions
/// of an impulse already counted under its own name (`~AirTorque` splits
/// `AirControl`). Summing them alongside the real entries double-counts, so the
/// identity skips them and only the per-source table uses them.
const INFO_PREFIX: u8 = b'~';

/// A closure below this is treated as exactly zero. The identity is built from
/// f32 sums in different orders, so a genuinely idle solver still leaves a few
/// ULPs behind; at car speeds one ULP is ~1e-4 UU/s, and the smallest real
/// contact impulse in the suite is orders of magnitude above this.
const CLOSURE_EPS: f32 = 1e-3;

#[derive(Default, Clone, Copy)]
struct Acc {
    n: u64,
    sum: f64,
    max: f32,
}

impl Acc {
    fn add(&mut self, v: f32) {
        self.n += 1;
        self.sum += f64::from(v);
        if v > self.max {
            self.max = v;
        }
    }

    fn mean(&self) -> f64 {
        if self.n == 0 {
            0.0
        } else {
            self.sum / self.n as f64
        }
    }
}

/// Per-source accumulator: how big this force's own contribution is, and how
/// often it fires at all.
#[derive(Default, Clone, Copy)]
struct Source {
    lin: Acc,
    ang: Acc,
}

pub fn analyze(recording: &Recording) {
    let dump = matches!(std::env::var("RLLEDGER").as_deref(), Ok("2"));
    let num_cars = recording.info.num_cars as usize;
    let stride = recording.stride;
    let (mut arena, car_idcs) = make_arena(num_cars);
    let mut controls_buf: Vec<CarControls> = vec![CarControls::DEFAULT; num_cars];

    let mut sources: std::collections::BTreeMap<String, Source> = Default::default();
    // The 2x2 of contact-reported against closure-nonzero.
    let mut idle_accounted = 0u64;
    let mut idle_suspect = Acc::default();
    let mut idle_suspect_ang = Acc::default();
    let mut contact_zero = 0u64;
    let mut contact_closure = Acc::default();
    // Suspect ticks split by whether the car could plausibly be touching
    // anything: a suspect while airborne cannot be a missed contact event.
    let mut suspect_airborne = 0u64;
    // What Rocket League did that we did not.
    let mut missing = Acc::default();
    let mut missing_ang = Acc::default();
    let mut worst_suspect: Vec<(f32, usize, usize, bool)> = Vec::new();

    let n = recording.ticks.len().saturating_sub(stride + 1);
    let mut contacted: Vec<bool> = vec![false; num_cars];

    for i in (0..=n).step_by(stride) {
        if super::runner::has_discontinuity(recording, i, stride) {
            continue;
        }
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

        // Copy the contact set out immediately: `step_tick` returns a borrow of
        // the arena and everything below needs it again.
        contacted.iter_mut().for_each(|c| *c = false);
        for event in arena.step_tick() {
            match event {
                ArenaEvent::CarHitWorld(e) => contacted[e.car_idx] = true,
                ArenaEvent::CarHitBall(e) => contacted[e.car_idx] = true,
                ArenaEvent::CarHitCar(e) => {
                    contacted[e.bumper_car_idx] = true;
                    contacted[e.victim_car_idx] = true;
                }
                _ => {}
            }
        }

        for (j, &car_idx) in car_idcs.iter().enumerate() {
            let from_car = &from_tick.car_records[j];
            let to_car = &to_tick.car_records[j];
            if to_car.is_demoed || is_car_sentinel(&from_car.phys) {
                continue;
            }

            let cs = *arena.get_car_state(car_idx);
            let v_before = Vec3A::from(from_car.phys.lin_vel);
            let w_before = Vec3A::from(from_car.phys.ang_vel);
            let sim_dv = cs.phys.vel - v_before;
            let sim_dw = cs.phys.ang_vel - w_before;
            let game_dv = Vec3A::from(to_car.phys.lin_vel) - v_before;
            let game_dw = Vec3A::from(to_car.phys.ang_vel) - w_before;

            // Sum the ledger. Real entries close the identity; `~` entries are
            // decompositions of entries already counted, so they only feed the
            // per-source table.
            let mut ledger_lin = Vec3A::ZERO;
            let mut ledger_ang = Vec3A::ZERO;
            let mut wheels_touched = false;
            let mut row: Vec<(&'static str, Vec3A, Vec3A)> = Vec::new();
            for (&(name, _accum), &(lin, ang)) in arena.get_car_impulse_history(car_idx) {
                let lin_uu = lin * BT_TO_UU;
                if name.as_bytes()[0] != INFO_PREFIX {
                    ledger_lin += lin_uu;
                    ledger_ang += ang;
                }
                if name == "WheelsSuspension" || name == "WheelsFriction" {
                    wheels_touched = true;
                }
                let e = sources.entry(name.to_string()).or_default();
                e.lin.add(lin_uu.length());
                e.ang.add(ang.length());
                if dump {
                    row.push((name, lin_uu, ang));
                }
            }

            let closure = sim_dv - ledger_lin;
            let closure_ang = sim_dw - ledger_ang;
            let nonzero = closure.length() > CLOSURE_EPS || closure_ang.length() > CLOSURE_EPS;

            match (contacted[j], nonzero) {
                (false, false) => idle_accounted += 1,
                (false, true) => {
                    idle_suspect.add(closure.length());
                    idle_suspect_ang.add(closure_ang.length());
                    // No wheel ray hit either, so the car is not resting on
                    // anything: a missed contact event is the less likely
                    // explanation here.
                    if !wheels_touched {
                        suspect_airborne += 1;
                    }
                    worst_suspect.push((
                        closure.length().max(closure_ang.length()),
                        i,
                        j,
                        wheels_touched,
                    ));
                }
                (true, false) => contact_zero += 1,
                (true, true) => contact_closure.add(closure.length()),
            }

            missing.add((game_dv - sim_dv).length());
            missing_ang.add((game_dw - sim_dw).length());

            if dump {
                let ctrl = from_car.prev_controls;
                let mut s = format!(
                    "LEDGER {} t{} car{} contact={} wheels={} \
                     pos={:.2},{:.2},{:.2} vel={:.3},{:.3},{:.3} spd={:.3} \
                     fwd={:.4},{:.4},{:.4} up={:.4},{:.4},{:.4} \
                     ctrl={:.3},{:.3},{:.3},{:.3},{},{},{} \
                     gdv={:.5},{:.5},{:.5} sdv={:.5},{:.5},{:.5} \
                     gdw={:.5},{:.5},{:.5} sdw={:.5},{:.5},{:.5} \
                     closure={:.5},{:.5},{:.5} closure_ang={:.5},{:.5},{:.5}",
                    recording.name,
                    i,
                    j,
                    u8::from(contacted[j]),
                    u8::from(wheels_touched),
                    from_car.phys.pos.x,
                    from_car.phys.pos.y,
                    from_car.phys.pos.z,
                    v_before.x,
                    v_before.y,
                    v_before.z,
                    v_before.length(),
                    from_car.phys.rot.rows[0].x,
                    from_car.phys.rot.rows[0].y,
                    from_car.phys.rot.rows[0].z,
                    from_car.phys.rot.rows[2].x,
                    from_car.phys.rot.rows[2].y,
                    from_car.phys.rot.rows[2].z,
                    ctrl.throttle,
                    ctrl.steer,
                    ctrl.pitch,
                    ctrl.yaw,
                    u8::from(ctrl.boost),
                    u8::from(ctrl.jump),
                    u8::from(ctrl.handbrake),
                    game_dv.x,
                    game_dv.y,
                    game_dv.z,
                    sim_dv.x,
                    sim_dv.y,
                    sim_dv.z,
                    game_dw.x,
                    game_dw.y,
                    game_dw.z,
                    sim_dw.x,
                    sim_dw.y,
                    sim_dw.z,
                    closure.x,
                    closure.y,
                    closure.z,
                    closure_ang.x,
                    closure_ang.y,
                    closure_ang.z,
                );
                for (name, lin, ang) in &row {
                    s.push_str(&format!(
                        " {}={:.5},{:.5},{:.5}/{:.5},{:.5},{:.5}",
                        name, lin.x, lin.y, lin.z, ang.x, ang.y, ang.z
                    ));
                }
                println!("{s}");
            }
        }
    }

    worst_suspect.sort_by(|a, b| b.0.total_cmp(&a.0));

    println!(
        "[{}] RLLEDGER ACCOUNTING idle+accounted={} idle+SUSPECT={} (of which no wheel contact: {}) contact+zero={} contact+closure={}",
        recording.name,
        idle_accounted,
        idle_suspect.n,
        suspect_airborne,
        contact_zero,
        contact_closure.n,
    );
    println!(
        "[{}] RLLEDGER suspect closure lin mean={:.5} max={:.5} UU/s | ang mean={:.5} max={:.5} rad/s | solver contact closure mean={:.4} max={:.4} UU/s",
        recording.name,
        idle_suspect.mean(),
        idle_suspect.max,
        idle_suspect_ang.mean(),
        idle_suspect_ang.max,
        contact_closure.mean(),
        contact_closure.max,
    );
    println!(
        "[{}] RLLEDGER vs RL: unexplained dv mean={:.4} UU/s, dw mean={:.6} rad/s over {} car-ticks",
        recording.name,
        missing.mean(),
        missing_ang.mean(),
        missing.n,
    );
    for (name, s) in &sources {
        println!(
            "[{}] RLLEDGER   {:<22} n={:<7} lin mean={:.5} max={:.5} | ang mean={:.5} max={:.5}",
            recording.name,
            name,
            s.lin.n,
            s.lin.mean(),
            s.lin.max,
            s.ang.mean(),
            s.ang.max,
        );
    }
    for &(mag, tick, car, wheels) in worst_suspect.iter().take(5) {
        println!(
            "[{}] RLLEDGER   SUSPECT t={} car{} closure={:.5} UU/s, wheel contact={}",
            recording.name,
            tick,
            car,
            mag,
            u8::from(wheels),
        );
    }
}
