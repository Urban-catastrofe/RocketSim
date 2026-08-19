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

use glam::Vec3A;
use rocketsim::CarControls;

use super::config::HarnessConfig;
use super::recording::Recording;
use super::runner::{has_discontinuity, is_car_sentinel, make_arena, set_state_to_record_tick};

/// Which mechanic produced the impulse, for reporting.
fn classify(rec: &super::recording::cpp_records::CarRecord) -> &'static str {
    if rec.is_flipping {
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
            if !recording.is_impulse_onset(onset, car) {
                continue;
            }
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
            let from = &recording.ticks[start];
            let real_end = &recording.ticks[end].car_records[car];
            if real_end.is_demoed || is_car_sentinel(&real_end.phys) {
                report.unmeasurable += 1;
                continue;
            }

            // Step 1: restore the frame before the press and run it.
            for (j, cr) in recording.ticks[onset].car_records.iter().enumerate() {
                controls[j] = cr.prev_controls.into();
            }
            set_state_to_record_tick(&mut arena, &car_idcs, from, &controls);
            arena.step_tick();

            // Step 2: only the controls advance — no restore, so the sim carries
            // its own post-impulse state into the second step exactly as it
            // would in free running.
            for (j, &car_idx) in car_idcs.iter().enumerate() {
                let c: CarControls = recording.ticks[end].car_records[j].prev_controls.into();
                arena.set_car_controls(car_idx, c);
            }
            arena.step_tick();

            let sim = arena.get_car_state(car_idcs[car]);
            let v0: Vec3A = from.car_records[car].phys.lin_vel.into();
            let v_end: Vec3A = real_end.phys.lin_vel.into();
            let p_end: Vec3A = real_end.phys.pos.into();

            report.events.push(ImpulseEvent {
                car,
                onset_tick: onset,
                kind: classify(&recording.ticks[onset].car_records[car]),
                game_dvel: v_end - v0,
                sim_dvel: sim.phys.vel - v0,
                vel_err: (sim.phys.vel - v_end).length(),
                pos_err: (sim.phys.pos - p_end).length(),
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
    for kind in ["jump", "double_jump", "flip"] {
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
