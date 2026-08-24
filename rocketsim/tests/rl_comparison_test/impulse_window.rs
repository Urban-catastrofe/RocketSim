//! Gate button-triggered impulses over a two-step window.
//!
//! Rocket League applies a jump, double-jump or flip impulse at the instant of
//! the press, which does not land on the 120 Hz physics grid. The impulse is
//! therefore split across the two recorded frames straddling the press, in
//! proportion (1−φ):φ for a sub-frame phase φ the recording never stores. The
//! per-tick gate cannot judge either of those steps — it would be measuring φ —
//! so [`super::runner`] skips them.
//!
//! What *is* well defined is the sum across the pair: however φ falls, the total
//! is one whole impulse. Measuring the window `onset−1 → onset+1` therefore
//! tests the impulse physics (magnitude *and* direction) with φ divided out.
//!
//! The sim is restored once, at `onset−1`, and then free-runs both steps with
//! the recorded controls. Restoring in between would re-impose the recording's
//! arbitrary split and defeat the whole point.
//!
//! Car-car bumps get the same treatment for the same reason, and the free-run is
//! what makes them measurable at all. A bump fires on geometric contact, and
//! Bullet detects contact from the positions at the *start* of a step: on
//! `car_car_basic_bump` the restored frame still leaves a 5.6 UU gap that the
//! step then closes, so a single restored step cannot produce the bump however
//! correct the impulse is. Free-running two steps lets the first close the gap
//! and the second fire, which tests the magnitude with the timing divided out.
//! A grounded jump overlapping car-ball contact gets one additional step because
//! Rocket League spreads that collision response across the following frame.

use glam::{Mat3A, Vec3A};
use rocketsim::CarControls;

use super::config::HarnessConfig;
use super::recording::Recording;
use super::runner::{has_discontinuity, is_car_sentinel, make_arena, set_state_to_record_tick};

fn ball_touches_car(recording: &Recording, tick: usize, car: usize) -> bool {
    let tick = &recording.ticks[tick];
    let car = &tick.car_records[car].phys;
    super::residual::ball_touches_car(
        tick.ball_record.pos.into(),
        car.pos.into(),
        Mat3A::from_cols(
            car.rot.rows[0].into(),
            car.rot.rows[1].into(),
            car.rot.rows[2].into(),
        ),
    )
}

/// Which mechanic produced the impulse, for reporting. A bump is identified by
/// the onset mark rather than by a flag — the observer records no bump event.
fn classify(rec: &super::recording::cpp_records::CarRecord, is_bump: bool) -> &'static str {
    if is_bump {
        "bump"
    } else if rec.is_flipping {
        "flip"
    } else if rec.is_on_ground {
        "jump"
    } else {
        "double_jump"
    }
}

#[derive(Debug, Clone)]
pub struct ImpulseEvent {
    pub car: usize,
    pub onset_tick: usize,
    pub kind: &'static str,
    /// Velocity change the recording shows across the window.
    pub game_dvel: Vec3A,
    /// Velocity change the sim produced across the same window.
    pub sim_dvel: Vec3A,
    pub vel_err: f32,
    pub pos_err: f32,
    /// Speed of each side at the end of the window, so a demo mismatch (one
    /// side parked at spawn while the other is still moving) is separable from
    /// an impulse-magnitude error.
    pub game_end_speed: f32,
    pub sim_end_speed: f32,
    /// The sim demoed inside the window. The recording's end state is checked
    /// before measuring, so this can only be a *disagreement*.
    pub sim_demoed: bool,
}

#[derive(Debug, Clone, Default)]
pub struct ImpulseReport {
    pub name: String,
    pub events: Vec<ImpulseEvent>,
    /// Onsets that could not be measured (too close to either end of the
    /// recording, or a discontinuity inside the window).
    pub unmeasurable: usize,
}

impl ImpulseReport {
    pub fn worst(&self) -> Option<&ImpulseEvent> {
        self.events
            .iter()
            .max_by(|a, b| a.vel_err.total_cmp(&b.vel_err))
    }
}

/// Replay every impulse onset in the recording over its two-step window.
pub fn measure_impulses(recording: &Recording, cfg: &HarnessConfig) -> ImpulseReport {
    let stride = recording.stride;
    let num_cars = recording.info.num_cars as usize;
    let mut report = ImpulseReport {
        name: recording.name.clone(),
        ..Default::default()
    };
    if recording.ticks.len() < 2 * stride + 1 {
        return report;
    }

    let (mut arena, car_idcs) = make_arena(num_cars);
    let mut controls: Vec<CarControls> = vec![CarControls::DEFAULT; num_cars];

    // Collect (onset tick, car) pairs in tick order so the arena walks forward.
    for onset in 0..recording.ticks.len() {
        for car in 0..num_cars {
            if !recording.is_any_onset(onset, car) {
                continue;
            }
            // A bump onset that coincides with a press is reported as the press:
            // the sim reproduces a press from the recorded controls, so that is
            // the stronger classification.
            let is_bump =
                recording.is_bump_onset(onset, car) && !recording.is_impulse_onset(onset, car);
            if cfg.car_focus.is_some_and(|c| c != car) {
                continue;
            }
            // The window needs a frame either side of the press.
            if onset < stride || onset + stride >= recording.ticks.len() {
                report.unmeasurable += 1;
                continue;
            }
            let start = onset - stride;
            let end = onset + stride;
            if has_discontinuity(recording, start, stride)
                || has_discontinuity(recording, onset, stride)
            {
                report.unmeasurable += 1;
                continue;
            }
            let kind = classify(&recording.ticks[onset].car_records[car], is_bump);
            // RLIMP_BUMPWIN=3 runs bump windows one step longer. Bullet detects
            // contact from the positions at the *start* of a step, so a window
            // that begins with the cars still apart cannot produce the bump
            // however correct the impulse is -- on `car_car_below_demo_speed`
            // the restored frame leaves a 15 UU gap that two steps at 1410 UU/s
            // do not close. This separates "the sim never fires" from "the sim
            // fires a tick later", which are different defects: only the first
            // one loses a bot the event.
            let bump_win_3 = kind == "bump"
                && matches!(std::env::var("RLIMP_BUMPWIN").as_deref(), Ok("3"))
                && end + stride < recording.ticks.len()
                && !has_discontinuity(recording, end, stride);
            let extended_end = end + stride;
            let extend_for_ball_contact = kind == "jump"
                && extended_end < recording.ticks.len()
                && !has_discontinuity(recording, end, stride)
                && [onset, end, extended_end]
                    .into_iter()
                    .any(|tick| ball_touches_car(recording, tick, car));
            let measured_end = if extend_for_ball_contact || bump_win_3 {
                extended_end
            } else {
                end
            };
            let from = &recording.ticks[start];
            let real_end = &recording.ticks[measured_end].car_records[car];
            if real_end.is_demoed || is_car_sentinel(&real_end.phys) {
                report.unmeasurable += 1;
                continue;
            }

            // Step 1: restore the frame before the press and run it.
            for (j, cr) in recording.ticks[onset].car_records.iter().enumerate() {
                controls[j] = cr.prev_controls.into();
            }
            set_state_to_record_tick(
                &mut arena,
                &car_idcs,
                from,
                start.checked_sub(stride).map(|i| &recording.ticks[i]),
                &controls,
            );
            arena.step_tick();

            // Step 2: only the controls advance — no restore, so the sim carries
            // its own post-impulse state into the second step exactly as it
            // would in free running.
            for (j, &car_idx) in car_idcs.iter().enumerate() {
                let c: CarControls = recording.ticks[end].car_records[j].prev_controls.into();
                arena.set_car_controls(car_idx, c);
            }
            arena.step_tick();

            if extend_for_ball_contact || bump_win_3 {
                for (j, &car_idx) in car_idcs.iter().enumerate() {
                    let c: CarControls = recording.ticks[extended_end].car_records[j]
                        .prev_controls
                        .into();
                    arena.set_car_controls(car_idx, c);
                }
                arena.step_tick();
            }
            let sim = *arena.get_car_state(car_idcs[car]);

            let v0: Vec3A = from.car_records[car].phys.lin_vel.into();
            let v_end: Vec3A = real_end.phys.lin_vel.into();
            let p_end: Vec3A = real_end.phys.pos.into();

            report.events.push(ImpulseEvent {
                car,
                onset_tick: onset,
                kind,
                game_dvel: v_end - v0,
                sim_dvel: sim.phys.vel - v0,
                vel_err: (sim.phys.vel - v_end).length(),
                pos_err: (sim.phys.pos - p_end).length(),
                game_end_speed: v_end.length(),
                sim_end_speed: sim.phys.vel.length(),
                sim_demoed: sim.is_demoed,
            });
        }
    }

    report
}

/// Report lines: a per-kind rollup plus the worst few individual events.
pub fn impulse_lines(report: &ImpulseReport, cfg: &HarnessConfig) -> Vec<String> {
    let mut lines = Vec::new();
    if report.events.is_empty() {
        if report.unmeasurable > 0 {
            lines.push(format!(
                "[{}] IMPULSE none measurable ({} onset(s) too close to an end or inside a discontinuity)",
                report.name, report.unmeasurable,
            ));
        }
        return lines;
    }

    let budget = cfg.tolerance.impulse_vel;
    for kind in ["jump", "double_jump", "flip", "bump"] {
        let of_kind: Vec<&ImpulseEvent> = report.events.iter().filter(|e| e.kind == kind).collect();
        if of_kind.is_empty() {
            continue;
        }
        let n = of_kind.len();
        let mean = of_kind.iter().map(|e| e.vel_err as f64).sum::<f64>() / n as f64;
        let worst = of_kind
            .iter()
            .max_by(|a, b| a.vel_err.total_cmp(&b.vel_err))
            .unwrap();
        let over = of_kind.iter().filter(|e| e.vel_err > budget).count();
        lines.push(format!(
            "[{}] IMPULSE {kind:11} n={n:3} mean={mean:8.4} max={:8.4}@t{} over={}/{} budget={budget:.3}",
            report.name, worst.vel_err, worst.onset_tick, over, n,
        ));
    }

    // RLIMP=1: one machine-readable row per event, so the error *distribution*
    // can be read instead of the worst three. The rollup above hides whether a
    // kind's mean comes from every event or from a handful of outliers.
    if matches!(std::env::var("RLIMP").as_deref(), Ok("1") | Ok("true")) {
        for e in &report.events {
            lines.push(format!(
                "IMPROW {} car{} {} t{} gdv={:.4},{:.4},{:.4} sdv={:.4},{:.4},{:.4}                  gmag={:.4} smag={:.4} vel_err={:.4} pos_err={:.4}                  gspd={:.4} sspd={:.4} sdemo={}",
                report.name,
                e.car,
                e.kind,
                e.onset_tick,
                e.game_dvel.x,
                e.game_dvel.y,
                e.game_dvel.z,
                e.sim_dvel.x,
                e.sim_dvel.y,
                e.sim_dvel.z,
                e.game_dvel.length(),
                e.sim_dvel.length(),
                e.vel_err,
                e.pos_err,
                e.game_end_speed,
                e.sim_end_speed,
                u8::from(e.sim_demoed),
            ));
        }
    }

    let mut worst: Vec<&ImpulseEvent> = report.events.iter().collect();
    worst.sort_by(|a, b| b.vel_err.total_cmp(&a.vel_err));
    for e in worst.iter().take(3).filter(|e| e.vel_err > budget) {
        lines.push(format!(
            "[{}] IMPULSE   worst car{} {} @t{}: game dv=({:+.1},{:+.1},{:+.1}) |{:.1}| \
             sim dv=({:+.1},{:+.1},{:+.1}) |{:.1}| vel_err={:.4} pos_err={:.4}",
            report.name,
            e.car,
            e.kind,
            e.onset_tick,
            e.game_dvel.x,
            e.game_dvel.y,
            e.game_dvel.z,
            e.game_dvel.length(),
            e.sim_dvel.x,
            e.sim_dvel.y,
            e.sim_dvel.z,
            e.sim_dvel.length(),
            e.vel_err,
            e.pos_err,
        ));
    }
    if report.unmeasurable > 0 {
        lines.push(format!(
            "[{}] IMPULSE {} onset(s) unmeasurable (recording end or discontinuity)",
            report.name, report.unmeasurable,
        ));
    }
    lines
}

/// Gate violations: the worst window error must stay inside the budget.
pub fn violations(report: &ImpulseReport, cfg: &HarnessConfig) -> Vec<String> {
    let budget = cfg.tolerance.impulse_vel;
    match report.worst() {
        Some(e) if e.vel_err > budget => vec![format!(
            "impulse {} car{} @t{}: two-step vel error {:.4} > {:.3} \
             (game |dv|={:.1}, sim |dv|={:.1})",
            e.kind,
            e.car,
            e.onset_tick,
            e.vel_err,
            budget,
            e.game_dvel.length(),
            e.sim_dvel.length(),
        )],
        _ => Vec::new(),
    }
}
