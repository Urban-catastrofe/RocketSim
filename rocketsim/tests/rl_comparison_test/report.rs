//! Aggregation, gating, and human/LLM-readable reporting.
//!
//! Turns the per-tick deltas collected by the runner into:
//! - a physics **gate** (hard fail when a budgeted percentile is exceeded),
//! - per-field bias/rms/percentile stats, overall and per situation,
//! - a state-machine mismatch-rate report,
//! - a deep-dive center tick to hand to [`super::deep_dive`].

use super::config::{GateMode, HarnessConfig};
use super::measure::{field_index, for_each_segment};
use super::recording::cpp_records::CarRecord;
use super::state::{StateFlag, StateStats};
use super::stats::{Field, FieldStats, RunningStats, Segment};

fn segment_idx(seg: Segment) -> usize {
    match seg {
        Segment::Grounded => 0,
        Segment::Airborne => 1,
        Segment::Boosting => 2,
        Segment::Jumping => 3,
        Segment::Flipping => 4,
        Segment::Supersonic => 5,
    }
}

#[derive(Debug, Clone)]
pub struct FieldReport {
    pub overall: FieldStats,
    pub by_segment: [RunningStats; Segment::BREAKDOWN.len()],
}

impl FieldReport {
    fn new(cap: usize) -> Self {
        Self {
            overall: FieldStats::new_with_capacity(cap),
            by_segment: [RunningStats::default(); Segment::BREAKDOWN.len()],
        }
    }

    pub fn add(&mut self, mag: f32, signed: glam::DVec3, tick: usize, car: Option<&CarRecord>) {
        self.overall.add(mag, signed, tick);
        if let Some(car) = car {
            for_each_segment(car, |seg| {
                self.by_segment[segment_idx(seg)].add(mag, signed, tick);
            });
        }
    }

    pub fn merge(&mut self, other: FieldReport) {
        self.overall.merge(other.overall);
        for i in 0..self.by_segment.len() {
            self.by_segment[i].merge(other.by_segment[i]);
        }
    }
}

#[derive(Debug, Clone)]
pub struct EntityReport {
    pub label: String,
    pub is_car: bool,
    pub fields: Vec<FieldReport>,
    pub state: StateStats,
}

impl EntityReport {
    fn new(label: String, is_car: bool, cap: usize) -> Self {
        Self {
            label,
            is_car,
            fields: Field::ALL.iter().map(|_| FieldReport::new(cap)).collect(),
            state: StateStats::default(),
        }
    }

    pub fn field(&self, f: Field) -> &FieldReport {
        &self.fields[field_index(f)]
    }
    pub fn field_mut(&mut self, f: Field) -> &mut FieldReport {
        &mut self.fields[field_index(f)]
    }

    pub fn merge(&mut self, other: EntityReport) {
        for (mine, theirs) in self.fields.iter_mut().zip(other.fields.into_iter()) {
            mine.merge(theirs);
        }
        self.state.merge(other.state);
    }
}

#[derive(Debug, Clone)]
pub struct Report {
    pub name: String,
    pub stride: usize,
    pub ticks_measured: u64,
    /// Ticks skipped because the recording has a non-physical discontinuity
    /// (goal reset, demo, logger car-index swap).
    pub ticks_voided: u64,
    /// Per-car steps skipped because they straddle a button-triggered impulse
    /// press, whose sub-frame phase the recording does not capture. Counted
    /// rather than silently dropped: these are the only steps the per-tick bar
    /// cannot judge, and their number is the size of that blind spot.
    pub impulse_steps_skipped: u64,
    pub num_cars: usize,
    /// Cars first (0..num_cars), then the ball at index num_cars.
    pub entities: Vec<EntityReport>,
}

impl Report {
    pub fn new(name: String, stride: usize, num_cars: usize, cap: usize) -> Self {
        let mut entities = Vec::with_capacity(num_cars + 1);
        for i in 0..num_cars {
            entities.push(EntityReport::new(format!("car_{i}"), true, cap));
        }
        entities.push(EntityReport::new("ball".to_string(), false, cap));
        Self {
            name,
            stride,
            ticks_measured: 0,
            ticks_voided: 0,
            impulse_steps_skipped: 0,
            num_cars,
            entities,
        }
    }

    /// Entity indices in scope for gating/reporting given the car focus.
    pub fn focused_indices(&self, cfg: &HarnessConfig) -> Vec<usize> {
        match cfg.car_focus {
            Some(c) if c < self.num_cars => vec![c],
            Some(_) => vec![],
            None => (0..self.entities.len()).collect(),
        }
    }

    pub fn focused_cars(&self, cfg: &HarnessConfig) -> Vec<usize> {
        self.focused_indices(cfg)
            .into_iter()
            .filter(|&i| self.entities[i].is_car)
            .collect()
    }

    /// Combine another report into this one (parallel shard merge).
    pub fn merge(&mut self, other: Report) {
        self.ticks_measured += other.ticks_measured;
        self.ticks_voided += other.ticks_voided;
        self.impulse_steps_skipped += other.impulse_steps_skipped;
        for (mine, theirs) in self.entities.iter_mut().zip(other.entities.into_iter()) {
            mine.merge(theirs);
        }
    }
}

/// Result of evaluating the physics gate.
#[derive(Debug, Clone, Default)]
pub struct GateOutcome {
    pub passed: bool,
    pub violations: Vec<String>,
    /// Suggested deep-dive center tick (worst hard-gated error), if any data.
    pub deep_dive_tick: Option<usize>,
    pub deep_dive_entity: usize,
}

fn budget_for(field: Field, cfg: &HarnessConfig) -> f32 {
    match field {
        Field::Pos => cfg.tolerance.pos,
        Field::Vel => cfg.tolerance.vel,
        Field::AngVel => cfg.tolerance.ang_vel,
        Field::RotFwd | Field::RotUp => cfg.tolerance.rot,
    }
}

/// Evaluate the gate and produce the report text lines (without trailing
/// newlines). Kept separate from printing so tests could assert on it.
pub fn evaluate(report: &mut Report, cfg: &HarnessConfig) -> (GateOutcome, Vec<String>) {
    let tol = cfg.tolerance;
    let focused = report.focused_indices(cfg);
    let mut outcome = GateOutcome {
        passed: true,
        ..Default::default()
    };
    let mut lines: Vec<String> = Vec::new();

    let gate_strict = cfg.gate == GateMode::Strict;
    let gate_label = if gate_strict { "STRICT" } else { "OFF" };
    let impulse_note = if report.impulse_steps_skipped > 0 {
        format!(" impulse_steps_skipped={}", report.impulse_steps_skipped)
    } else {
        String::new()
    };
    lines.push(format!(
        "==== RLPR {} | ticks={} stride={} gate={} percentile=p{:0>2}{} ====",
        report.name,
        report.ticks_measured,
        report.stride,
        gate_label,
        (tol.percentile * 100.0) as i32,
        impulse_note,
    ));

    // A case that compared nothing proves nothing: every field reports 0.0 and
    // would sail through the gate. Truncated or fully-discontinuous recordings
    // must fail loudly rather than inflate the pass count.
    if report.ticks_measured == 0 {
        outcome.passed = false;
        outcome.violations.push(format!(
            "no ticks measured ({} voided) - nothing was verified",
            report.ticks_voided,
        ));
    }

    // Worst hard-gated error across focused entities, for deep-dive targeting.
    let mut worst_mag: f32 = -1.0;

    for &ei in &focused {
        let ent = &mut report.entities[ei];
        for field in Field::ALL {
            let fr = ent.field_mut(field);
            let p = fr.overall.percentile(tol.percentile);
            let run = fr.overall.run;
            let budget = budget_for(field, cfg);
            let is_hard = Field::HARD_GATED.contains(&field);
            let frac_over = fr.overall.fraction_over(budget);
            let bias = run.bias_mean();
            let bias_mag = bias.length();

            let verdict = if run.count == 0 {
                "n/a"
            } else if is_hard && p > budget {
                "FAIL"
            } else if is_hard {
                "pass"
            } else {
                "soft"
            };
            if is_hard && p > budget {
                outcome.passed = false;
                outcome.violations.push(format!(
                    "{} {} p{:0>2}={:.4}{} > budget {:.4} (max={:.2}@t{}, {:.2}% over)",
                    ent.label,
                    field.label(),
                    (tol.percentile * 100.0) as i32,
                    p,
                    field.unit(),
                    budget,
                    run.max,
                    run.max_tick,
                    frac_over * 100.0,
                ));
            }

            // Track worst hard-gated tick for the deep dive.
            if is_hard && run.max > worst_mag {
                worst_mag = run.max;
                outcome.deep_dive_tick = Some(run.max_tick);
                outcome.deep_dive_entity = ei;
            }

            lines.push(format!(
                "[{}] {:>7} {:<7}: p={:.4} max={:.3}@t{} mean={:.4} rms={:.4} bias={:.4} over={:5.2}% budget={:.3} -> {}",
                report.name,
                ent.label,
                field.label(),
                p,
                run.max,
                run.max_tick,
                run.mean(),
                run.rms(),
                bias_mag,
                frac_over * 100.0,
                budget,
                verdict,
            ));
            // Directional bias detail (which way the sim consistently misses).
            if bias_mag > 1e-4 {
                lines.push(format!(
                    "[{}] {:>7} {:<7}  bias_vec=({:+.3}, {:+.3}, {:+.3})",
                    report.name,
                    ent.label,
                    field.label(),
                    bias.x,
                    bias.y,
                    bias.z,
                ));
            }
        }
    }

    // Gate verdict line.
    if !gate_strict {
        lines.push(format!("[{}] GATE: OFF (measurement only)", report.name));
    } else if outcome.passed {
        lines.push(format!("[{}] GATE: PASS", report.name));
    } else {
        lines.push(format!(
            "[{}] GATE: FAIL ({} violation{})",
            report.name,
            outcome.violations.len(),
            if outcome.violations.len() == 1 { "" } else { "s" },
        ));
        for v in &outcome.violations {
            lines.push(format!("[{}]   VIOLATION {}", report.name, v));
        }
    }

    // Honor an explicit tick focus for the deep dive.
    if let Some(t) = cfg.tick_focus {
        outcome.deep_dive_tick = Some(t);
    }

    (outcome, lines)
}

/// Per-situation breakdown lines (only when `RLSEG=1`).
pub fn segment_lines(report: &Report, cfg: &HarnessConfig) -> Vec<String> {
    let mut lines = Vec::new();
    for ei in report.focused_indices(cfg) {
        let ent = &report.entities[ei];
        for field in Field::ALL {
            let fr = ent.field(field);
            for (si, seg) in Segment::BREAKDOWN.iter().enumerate() {
                let s = fr.by_segment[si];
                if s.count == 0 {
                    continue;
                }
                lines.push(format!(
                    "[{}] SEG {:>7} {:<7} {:<10}: n={} mean={:.4} rms={:.4} max={:.3}@t{}",
                    report.name,
                    ent.label,
                    field.label(),
                    seg.label(),
                    s.count,
                    s.mean(),
                    s.rms(),
                    s.max,
                    s.max_tick,
                ));
            }
        }
    }
    lines
}

/// State-machine mismatch-rate lines, per focused car, worst flags first.
pub fn state_lines(report: &Report, cfg: &HarnessConfig) -> Vec<String> {
    let mut lines = Vec::new();
    for ei in report.focused_cars(cfg) {
        let ent = &report.entities[ei];
        let mut rows: Vec<(f64, &str)> = StateFlag::ALL
            .iter()
            .map(|f| (ent.state.mismatch_rate(*f), f.label()))
            .collect();
        rows.sort_by(|a, b| b.0.partial_cmp(&a.0).unwrap());
        let nonzero: Vec<String> = rows
            .iter()
            .filter(|(r, _)| *r > 0.0)
            .map(|(r, l)| format!("{l}={:.2}%", r * 100.0))
            .collect();
        let detail = if nonzero.is_empty() {
            "all flags match".to_string()
        } else {
            nonzero.join(" ")
        };
        lines.push(format!(
            "[{}] STATE {:>7} (n={}): {}",
            report.name, ent.label, ent.state.total, detail
        ));
        let st = &ent.state;
        let mut timers = Vec::new();
        if st.air_time_err.count > 0 {
            timers.push(format!(
                "air_time mean={:.4}s max={:.4}s (n={})",
                st.air_time_err.mean(),
                st.air_time_err.max,
                st.air_time_err.count
            ));
        }
        if st.jump_time_err.count > 0 {
            timers.push(format!(
                "jump_time mean={:.4}s max={:.4}s (n={})",
                st.jump_time_err.mean(),
                st.jump_time_err.max,
                st.jump_time_err.count
            ));
        }
        if st.wheel_count_err.max > 0.0 {
            timers.push(format!(
                "wheel_count |dn| mean={:.4} max={:.0}",
                st.wheel_count_err.mean(),
                st.wheel_count_err.max
            ));
        }
        if !timers.is_empty() {
            lines.push(format!(
                "[{}] STATE {:>7} timers: {}",
                report.name,
                ent.label,
                timers.join(" | ")
            ));
        }
    }
    lines
}
