//! Measurement vocabulary and statistics accumulators.
//!
//! Pure math + the enums that name *what* we measure. No arena/sim driving
//! lives here, so it stays trivially testable and allocation-light.

use glam::DVec3;

/// A physical quantity compared between sim and recording each tick.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum Field {
    Pos,
    Vel,
    AngVel,
    RotFwd,
    RotUp,
}

impl Field {
    pub const ALL: [Field; 5] = [
        Field::Pos,
        Field::Vel,
        Field::AngVel,
        Field::RotFwd,
        Field::RotUp,
    ];

    /// Fields that hard-gate the test when their percentile exceeds budget.
    pub const HARD_GATED: [Field; 2] = [Field::Pos, Field::Vel];

    pub fn label(&self) -> &'static str {
        match self {
            Field::Pos => "pos",
            Field::Vel => "vel",
            Field::AngVel => "ang_vel",
            Field::RotFwd => "rot_fwd",
            Field::RotUp => "rot_up",
        }
    }

    pub fn unit(&self) -> &'static str {
        match self {
            Field::Pos => "UU",
            Field::Vel => "UU/s",
            Field::AngVel => "rad/s",
            Field::RotFwd | Field::RotUp => "dir",
        }
    }
}

/// A situation a tick is tagged with, derived from the recording's own flags.
///
/// Segmenting error by situation localises a bug to a subsystem: e.g. large
/// vel error only while `Grounded` + boosting points at ground friction/boost,
/// while error only while `Airborne` points at air control. The overall rollup
/// (all ticks) is kept separately in [`FieldStats`], not as a segment here.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum Segment {
    Grounded,
    Airborne,
    Boosting,
    Jumping,
    Flipping,
    Supersonic,
}

impl Segment {
    pub const BREAKDOWN: [Segment; 6] = [
        Segment::Grounded,
        Segment::Airborne,
        Segment::Boosting,
        Segment::Jumping,
        Segment::Flipping,
        Segment::Supersonic,
    ];

    pub fn label(&self) -> &'static str {
        match self {
            Segment::Grounded => "grounded",
            Segment::Airborne => "airborne",
            Segment::Boosting => "boosting",
            Segment::Jumping => "jumping",
            Segment::Flipping => "flipping",
            Segment::Supersonic => "supersonic",
        }
    }
}

/// Constant-memory running statistics (no per-tick storage).
#[derive(Debug, Clone, Copy, Default)]
pub struct RunningStats {
    pub count: u64,
    pub sum: f64,
    pub sum_sq: f64,
    /// Sum of signed (pred - real) vectors, for directional bias.
    pub bias: DVec3,
    pub max: f32,
    pub max_tick: usize,
}

impl RunningStats {
    pub fn add(&mut self, mag: f32, signed: DVec3, tick: usize) {
        self.count += 1;
        self.sum += mag as f64;
        self.sum_sq += (mag as f64) * (mag as f64);
        self.bias += signed;
        if mag > self.max {
            self.max = mag;
            self.max_tick = tick;
        }
    }

    pub fn mean(&self) -> f64 {
        if self.count == 0 {
            0.0
        } else {
            self.sum / self.count as f64
        }
    }

    pub fn rms(&self) -> f64 {
        if self.count == 0 {
            0.0
        } else {
            (self.sum_sq / self.count as f64).sqrt()
        }
    }

    /// Average signed error vector: magnitude = systematic bias, direction =
    /// which way the sim consistently misses.
    pub fn bias_mean(&self) -> DVec3 {
        if self.count == 0 {
            DVec3::ZERO
        } else {
            self.bias / self.count as f64
        }
    }

    /// Combine another accumulator into this one (parallel shard merge).
    pub fn merge(&mut self, other: RunningStats) {
        self.count += other.count;
        self.sum += other.sum;
        self.sum_sq += other.sum_sq;
        self.bias += other.bias;
        if other.max > self.max {
            self.max = other.max;
            self.max_tick = other.max_tick;
        }
    }
}

/// Per-field stats. Carries the per-tick magnitudes so an exact percentile can
/// be computed for gating. Only the overall rollup stores magnitudes; segment
/// breakdowns use [`RunningStats`] only to keep memory bounded on long replays.
#[derive(Debug, Clone, Default)]
pub struct FieldStats {
    pub run: RunningStats,
    pub mags: Vec<f32>,
    sorted: bool,
}

impl FieldStats {
    pub fn new_with_capacity(cap: usize) -> Self {
        Self {
            run: RunningStats::default(),
            mags: Vec::with_capacity(cap),
            sorted: false,
        }
    }

    pub fn add(&mut self, mag: f32, signed: DVec3, tick: usize) {
        self.run.add(mag, signed, tick);
        self.mags.push(mag);
        self.sorted = false;
    }

    /// Exact percentile (nearest-rank). Sorts lazily on first call.
    pub fn percentile(&mut self, p: f32) -> f32 {
        if self.mags.is_empty() {
            return 0.0;
        }
        if !self.sorted {
            self.mags.sort_by(|a, b| a.partial_cmp(b).unwrap());
            self.sorted = true;
        }
        percentile_sorted(&self.mags, p)
    }

    /// Fraction of samples exceeding `threshold`.
    pub fn fraction_over(&mut self, threshold: f32) -> f64 {
        if self.mags.is_empty() {
            return 0.0;
        }
        if !self.sorted {
            self.mags.sort_by(|a, b| a.partial_cmp(b).unwrap());
            self.sorted = true;
        }
        // First index with value > threshold.
        let idx = self.mags.partition_point(|v| *v <= threshold);
        (self.mags.len() - idx) as f64 / self.mags.len() as f64
    }

    /// Combine another accumulator into this one (parallel shard merge).
    pub fn merge(&mut self, other: FieldStats) {
        self.run.merge(other.run);
        self.mags.extend(other.mags);
        self.sorted = false;
    }
}

fn percentile_sorted(sorted: &[f32], p: f32) -> f32 {
    let n = sorted.len();
    if n == 0 {
        return 0.0;
    }
    let rank = (p.clamp(0.0, 1.0) * n as f32).ceil() as usize;
    let idx = rank.clamp(1, n) - 1;
    sorted[idx]
}
