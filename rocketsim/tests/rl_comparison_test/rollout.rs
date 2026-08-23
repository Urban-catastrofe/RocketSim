//! Rollout mode (`RLROLL=1s` / `RLROLL=2s` / `RLROLL=<ticks>`).
//!
//! The per-tick pass restores the full recorded state every tick, so it only
//! ever measures a *single* physics step in isolation. Some bugs only surface
//! once errors compound: a slightly-wrong landing pose, a state flag that
//! drifts, a friction term that bleeds speed over a second of driving.
//!
//! Rollout mode closes that gap. It samples start ticks every
//! [`HarnessConfig::rollout_stride`] ticks, restores the recorded state there,
//! then free-runs the sim for [`HarnessConfig::rollout_ticks`] steps using the
//! recording's own controls — measuring the divergence at every step of the
//! rollout. The result is an error-vs-time *growth curve* per entity+field:
//! flat ≈ the sim is stable, climbing ≈ a compounding bug. Informational
//! only (never gated).

use rocketsim::{BallState, CarControls, CarState};

use super::config::HarnessConfig;
use super::measure::{compute_delta, field_index};
use super::recording::Recording;
use super::runner;
use super::state::{StateFlag, StateStats};
use super::stats::{Field, RunningStats};

/// Per-field rollout stats: one [`RunningStats`] per rollout step, so the
/// error growth curve can be read off at any horizon.
#[derive(Debug, Clone)]
pub struct RolloutFieldReport {
    /// Index `k - 1` holds the stats across all rollouts at step `k`.
    pub per_step: Vec<RunningStats>,
    /// Largest error any rollout reached at its final measured step, and the
    /// start tick of that rollout — a deep-dive target.
    pub worst_end_mag: f32,
    pub worst_end_start: usize,
}

impl RolloutFieldReport {
    fn new(rollout_ticks: usize) -> Self {
        Self {
            per_step: vec![RunningStats::default(); rollout_ticks],
            worst_end_mag: 0.0,
            worst_end_start: 0,
        }
    }

    fn merge(&mut self, other: RolloutFieldReport) {
        for (mine, theirs) in self.per_step.iter_mut().zip(other.per_step) {
            mine.merge(theirs);
        }
        if other.worst_end_mag > self.worst_end_mag {
            self.worst_end_mag = other.worst_end_mag;
            self.worst_end_start = other.worst_end_start;
        }
    }
}

#[derive(Debug, Clone)]
pub struct RolloutEntityReport {
    pub label: String,
    pub is_car: bool,
    pub fields: Vec<RolloutFieldReport>,
    /// State-flag mismatch + timer-error accumulation across all rollout steps
    /// (cars only; the ball leaves it empty).
    pub state: StateStats,
}

impl RolloutEntityReport {
    fn new(label: String, is_car: bool, rollout_ticks: usize) -> Self {
        Self {
            label,
            is_car,
            fields: Field::ALL
                .iter()
                .map(|_| RolloutFieldReport::new(rollout_ticks))
                .collect(),
            state: StateStats::default(),
        }
    }

    pub fn field_mut(&mut self, f: Field) -> &mut RolloutFieldReport {
        &mut self.fields[field_index(f)]
    }

    fn merge(&mut self, other: RolloutEntityReport) {
        for (mine, theirs) in self.fields.iter_mut().zip(other.fields) {
            mine.merge(theirs);
        }
        self.state.merge(other.state);
    }
}

#[derive(Debug, Clone)]
pub struct RolloutReport {
    pub name: String,
    pub stride: usize,
    pub rollout_ticks: usize,
    pub rollout_stride: usize,
    pub num_cars: usize,
    pub rollouts_measured: u64,
    /// Rollouts cut short by a recording discontinuity or the end of the file.
    pub rollouts_truncated: u64,
    pub entities: Vec<RolloutEntityReport>,
}

impl RolloutReport {
    pub fn new(name: String, recording: &Recording, cfg: &HarnessConfig) -> Self {
        let num_cars = recording.info.num_cars as usize;
        let mut entities = Vec::with_capacity(num_cars + 1);
        for i in 0..num_cars {
            entities.push(RolloutEntityReport::new(
                format!("car_{i}"),
                true,
                cfg.rollout_ticks,
            ));
        }
        entities.push(RolloutEntityReport::new(
            "ball".to_string(),
            false,
            cfg.rollout_ticks,
        ));
        Self {
            name,
            stride: recording.stride,
            rollout_ticks: cfg.rollout_ticks,
            rollout_stride: cfg.rollout_stride,
            num_cars,
            rollouts_measured: 0,
            rollouts_truncated: 0,
            entities,
        }
    }

    fn merge(&mut self, other: RolloutReport) {
        self.rollouts_measured += other.rollouts_measured;
        self.rollouts_truncated += other.rollouts_truncated;
        for (mine, theirs) in self.entities.iter_mut().zip(other.entities) {
            mine.merge(theirs);
        }
    }
}

/// Write the controls that drive the step *into* `to_tick` (its recorded
/// `prev_controls`) into `buf`.
fn controls_for(to_tick: &super::recording::tick_record::TickRecord, buf: &mut Vec<CarControls>) {
    for (j, car_record) in to_tick.car_records.iter().enumerate() {
        buf[j] = car_record.prev_controls.into();
    }
}

/// Rollout worker for one shard of start ticks. Owns its arena so shards can
/// run on separate threads; each start tick restores full state, so rollouts
/// are independent.
fn run_rollout_shard(recording: &Recording, shard: &[usize], cfg: &HarnessConfig) -> RolloutReport {
    let num_cars = recording.info.num_cars as usize;
    let stride = recording.stride;
    let rollout_ticks = cfg.rollout_ticks;
    let (mut arena, car_idcs) = runner::make_arena(num_cars);
    let mut report = RolloutReport::new(recording.name.clone(), recording, cfg);
    let mut controls_buf: Vec<CarControls> = vec![CarControls::DEFAULT; num_cars];
    // Last measured per-entity field magnitudes for the current rollout, used
    // to update the worst-end tracker once the rollout's length is known.
    let mut last_mag: Vec<[f32; Field::ALL.len()]> = vec![[0.0; Field::ALL.len()]; num_cars + 1];

    // Warm the shard's arena once (cold Bullet manifolds diverge on the first
    // steps) using the first start tick, discarding the result.
    let mut warmed = false;

    for &i in shard {
        let bound = recording.ticks.len();
        if i + stride >= bound {
            continue;
        }

        let from_tick = &recording.ticks[i];
        controls_for(&recording.ticks[i + stride], &mut controls_buf);
        let previous_tick = i.checked_sub(stride).map(|i| &recording.ticks[i]);
        runner::set_state_to_record_tick(
            &mut arena,
            &car_idcs,
            from_tick,
            previous_tick,
            &controls_buf,
        );

        if !warmed {
            for _ in 0..runner::SHARD_WARMUP_TICKS {
                arena.step_tick();
            }
            // Re-restore the exact start pose now that manifolds are warm.
            runner::set_state_to_record_tick(
                &mut arena,
                &car_idcs,
                from_tick,
                previous_tick,
                &controls_buf,
            );
            warmed = true;
        }

        let mut last_step = 0usize;
        for k in 1..=rollout_ticks {
            let target = i + k * stride;
            if target >= bound {
                break;
            }
            // Don't step across a non-physical discontinuity (goal reset, demo,
            // index swap) — the sim cannot reproduce a teleport.
            if runner::has_discontinuity(recording, target - stride, stride) {
                break;
            }

            arena.step_tick();
            let to_tick = &recording.ticks[target];

            for (j, &car_idx) in car_idcs.iter().enumerate() {
                let real = &to_tick.car_records[j];
                if real.is_demoed || runner::is_car_sentinel(&real.phys) {
                    continue;
                }
                let cs: CarState = *arena.get_car_state(car_idx);
                let delta = compute_delta(&cs.phys, &real.phys);
                let ent = &mut report.entities[j];
                for field in Field::ALL {
                    let s = *delta.get(field);
                    let fr = ent.field_mut(field);
                    fr.per_step[k - 1].add(s.mag, s.signed, i);
                    last_mag[j][field_index(field)] = s.mag;
                }
                ent.state.record(&cs, real);
            }

            if !runner::is_ball_sentinel(&to_tick.ball_record) {
                let ball_state: &BallState = arena.get_ball_state();
                let delta = compute_delta(&ball_state.phys, &to_tick.ball_record);
                let ball_ent = &mut report.entities[report.num_cars];
                for field in Field::ALL {
                    let s = *delta.get(field);
                    let fr = ball_ent.field_mut(field);
                    fr.per_step[k - 1].add(s.mag, s.signed, i);
                    last_mag[report.num_cars][field_index(field)] = s.mag;
                }
            }

            last_step = k;

            // Controls for the *next* step (into target + stride).
            let next = target + stride;
            if next < bound {
                controls_for(&recording.ticks[next], &mut controls_buf);
                for (j, &car_idx) in car_idcs.iter().enumerate() {
                    arena.set_car_controls(car_idx, controls_buf[j]);
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
                    let fi = field_index(field);
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

/// Rollout mode, sharded across `cfg.threads` workers.
pub fn run_rollout(recording: &Recording, cfg: &HarnessConfig) -> RolloutReport {
    let stride = recording.stride;
    let n = runner::loop_bound(recording, None);
    let starts: Vec<usize> = (0..=n).step_by(cfg.rollout_stride * stride).collect();

    if starts.is_empty() || cfg.rollout_ticks == 0 {
        return RolloutReport::new(recording.name.clone(), recording, cfg);
    }

    let threads = if starts.len() < 4 {
        1
    } else {
        cfg.threads.clamp(1, starts.len())
    };
    if threads <= 1 {
        return run_rollout_shard(recording, &starts, cfg);
    }

    let mut merged = RolloutReport::new(recording.name.clone(), recording, cfg);
    let chunk = starts.len().div_ceil(threads);
    std::thread::scope(|s| {
        let handles: Vec<_> = starts
            .chunks(chunk)
            .map(|shard| s.spawn(move || run_rollout_shard(recording, shard, cfg)))
            .collect();
        for handle in handles {
            merged.merge(handle.join().expect("rollout shard panicked"));
        }
    });
    merged
}

/// Horizons (in rollout steps) to print on the growth curve. Always includes
/// the final step. 60/120/180/240 are the half-second marks, so a `RLROLL=2s`
/// run reads off 0.5s/1.0s/1.5s/2.0s directly.
fn sample_horizons(rollout_ticks: usize) -> Vec<usize> {
    let mut hs: Vec<usize> = [1usize, 5, 15, 30, 60, 120, 180, 240, 360, 480]
        .iter()
        .copied()
        .filter(|&h| h <= rollout_ticks)
        .collect();
    if hs.last().copied() != Some(rollout_ticks) {
        hs.push(rollout_ticks);
    }
    hs.dedup();
    hs
}

/// Human/LLM-readable growth-curve lines.
pub fn rollout_lines(report: &RolloutReport, cfg: &HarnessConfig) -> Vec<String> {
    let mut lines = Vec::new();
    let horizons = sample_horizons(report.rollout_ticks);

    lines.push(format!(
        "==== ROLLOUT {} | free-run {}t ({:.2}s) from {} starts (stride {}) | rollouts={} truncated={} ====",
        report.name,
        report.rollout_ticks,
        report.rollout_ticks as f64 / 120.0,
        report.rollouts_measured,
        report.rollout_stride,
        report.rollouts_measured,
        report.rollouts_truncated,
    ));

    let focused: Vec<usize> = match cfg.car_focus {
        Some(c) if c < report.num_cars => vec![c],
        Some(_) => vec![],
        None => (0..report.entities.len()).collect(),
    };

    for ei in focused {
        let ent = &report.entities[ei];
        for field in Field::ALL {
            let fr = &ent.fields[field_index(field)];
            let curve: Vec<String> = horizons
                .iter()
                .filter_map(|&h| {
                    let s = fr.per_step.get(h - 1)?;
                    if s.count == 0 {
                        return None;
                    }
                    Some(format!("t{}={:.3}/n{}", h, s.mean(), s.count))
                })
                .collect();
            if curve.is_empty() {
                continue;
            }
            lines.push(format!(
                "[{}] ROLL {:>7} {:<7}: {} | worst_end={:.2}@start{}",
                report.name,
                ent.label,
                field.label(),
                curve.join(" "),
                fr.worst_end_mag,
                fr.worst_end_start,
            ));
            // Systematic drift vs decorrelation. The mean *magnitude* above
            // grows either way, so it cannot tell a wrong force from two
            // trajectories that merely stopped agreeing. The mean *signed*
            // error can: it only stays large when every rollout misses the
            // same way, so the printed fraction bias/mag near 1 is a force
            // error and near 0 is decorrelation.
            let bias: Vec<String> = horizons
                .iter()
                .filter_map(|&h| {
                    let s = fr.per_step.get(h - 1)?;
                    if s.count == 0 {
                        return None;
                    }
                    let (b, m) = (s.bias_mean().length(), s.mean());
                    Some(format!(
                        "t{}={:.3}/f{:.2}",
                        h,
                        b,
                        if m > 0.0 { b / m } else { 0.0 }
                    ))
                })
                .collect();
            lines.push(format!(
                "[{}] ROLLBIAS {:>7} {:<7}: {}",
                report.name,
                ent.label,
                field.label(),
                bias.join(" "),
            ));
        }
        if ent.is_car && ent.state.total > 0 {
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
                "[{}] ROLLSTATE {:>7} (n={}): {}",
                report.name, ent.label, ent.state.total, detail
            ));
            let st = &ent.state;
            let mut timers = Vec::new();
            if st.air_time_err.count > 0 {
                timers.push(format!(
                    "air_time mean={:.4}s max={:.4}s",
                    st.air_time_err.mean(),
                    st.air_time_err.max
                ));
            }
            if st.jump_time_err.count > 0 {
                timers.push(format!(
                    "jump_time mean={:.4}s max={:.4}s",
                    st.jump_time_err.mean(),
                    st.jump_time_err.max
                ));
            }
            if !timers.is_empty() {
                lines.push(format!(
                    "[{}] ROLLSTATE {:>7} timers: {}",
                    report.name,
                    ent.label,
                    timers.join(" | ")
                ));
            }
        }
    }

    lines
}
