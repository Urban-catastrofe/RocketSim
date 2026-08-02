//! Runtime harness options, parsed from environment variables.
//!
//! These let a human or an LLM drive the harness at a specific case without
//! editing code. Every knob is optional; defaults give a full, gated run.
//!
//! | Env var          | Meaning                                                    | Default |
//! |------------------|------------------------------------------------------------|---------|
//! | `RLDEEP`         | `fail` \| `always` \| `never` — when to emit a deep dive   | `fail`  |
//! | `RLDEEP_RADIUS`  | context ticks on *each side* of the deep-dive center       | `15`    |
//! | `RLCAR`          | focus a single car index (e.g. `0`), or `all`             | `all`   |
//! | `RLTICK`         | deep-dive a specific tick instead of the worst             | worst   |
//! | `RLSEG`          | `1` to print per-segment (situation) breakdowns            | off     |
//! | `RLGATE`         | `strict` \| `off` — enforce the physics gate               | `strict`|
//! | `RL_*_TOL`       | tolerance overrides (see [`PhysicsTolerance`])             | —       |

use super::tolerance::{PhysicsTolerance, env_usize};

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum DeepDiveMode {
    /// Emit a deep dive only when the physics gate fails.
    OnFail,
    /// Always emit a deep dive (for exploration).
    Always,
    /// Never emit a deep dive.
    Never,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum GateMode {
    /// Fail the test when a hard field exceeds its budget.
    Strict,
    /// Measure and report only; never fail.
    Off,
}

#[derive(Debug, Clone)]
pub struct HarnessConfig {
    pub deep_dive: DeepDiveMode,
    /// Context ticks on each side of the deep-dive center tick.
    pub deep_dive_radius: usize,
    /// Restrict measurement + gate to one car index. `None` = all cars.
    pub car_focus: Option<usize>,
    /// Force the deep-dive center to this tick. `None` = worst tick.
    pub tick_focus: Option<usize>,
    /// Print the per-segment (situation) breakdown.
    pub emit_segments: bool,
    pub gate: GateMode,
    pub tolerance: PhysicsTolerance,
    /// Worker threads for the (embarrassingly parallel) per-tick pass.
    pub threads: usize,
    /// Also run the sequential continuous (compounding) pass. Off by default
    /// because it doubles runtime on long replays and is informational only.
    pub continuous: bool,
}

impl Default for HarnessConfig {
    fn default() -> Self {
        Self {
            deep_dive: DeepDiveMode::OnFail,
            deep_dive_radius: 15,
            car_focus: None,
            tick_focus: None,
            emit_segments: false,
            gate: GateMode::Strict,
            tolerance: PhysicsTolerance::default(),
            threads: default_threads(),
            continuous: false,
        }
    }
}

fn default_threads() -> usize {
    std::thread::available_parallelism()
        .map(|n| n.get())
        .unwrap_or(4)
        .clamp(1, 16)
}

impl HarnessConfig {
    pub fn from_env() -> Self {
        let mut cfg = Self::default();

        match std::env::var("RLDEEP").as_deref() {
            Ok("always") | Ok("1") | Ok("true") => cfg.deep_dive = DeepDiveMode::Always,
            Ok("never") | Ok("0") | Ok("false") => cfg.deep_dive = DeepDiveMode::Never,
            _ => cfg.deep_dive = DeepDiveMode::OnFail,
        }

        if let Ok(r) = env_usize("RLDEEP_RADIUS") {
            cfg.deep_dive_radius = r.clamp(1, 240);
        }

        cfg.car_focus = match std::env::var("RLCAR").ok().as_deref() {
            Some("all") | None | Some("") => None,
            Some(s) => s.trim().parse::<usize>().ok(),
        };

        cfg.tick_focus = std::env::var("RLTICK")
            .ok()
            .and_then(|s| s.trim().parse::<usize>().ok());

        cfg.emit_segments = matches!(std::env::var("RLSEG").as_deref(), Ok("1") | Ok("true"));

        cfg.gate = match std::env::var("RLGATE").as_deref() {
            Ok("off") | Ok("0") | Ok("false") => GateMode::Off,
            _ => GateMode::Strict,
        };

        if let Ok(t) = env_usize("RLTHREADS") {
            cfg.threads = t.clamp(1, 64);
        }
        cfg.continuous = matches!(
            std::env::var("RLCONT").as_deref(),
            Ok("1") | Ok("true") | Ok("always")
        );

        cfg.tolerance = cfg.tolerance.from_env();
        cfg
    }
}
