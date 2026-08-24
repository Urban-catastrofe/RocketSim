//! State-machine comparison channel.
//!
//! Discrete flags (is_jumping, has_flipped, …) are reported separately from
//! physics: they reveal state-machine *timing* bugs, not integration errors,
//! and need a different fix. The metric here is mismatch rate (% of ticks the
//! sim and recording disagree), never a hard gate.

use rocketsim::CarState;

use super::recording::cpp_records::CarRecord;

#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum StateFlag {
    OnGround,
    Jumping,
    HasJumped,
    Flipping,
    HasDoubleJumped,
    HasFlipped,
    Boosting,
    Supersonic,
    Demoed,
    WheelsContact,
}

impl StateFlag {
    pub const ALL: [StateFlag; 10] = [
        StateFlag::OnGround,
        StateFlag::Jumping,
        StateFlag::HasJumped,
        StateFlag::Flipping,
        StateFlag::HasDoubleJumped,
        StateFlag::HasFlipped,
        StateFlag::Boosting,
        StateFlag::Supersonic,
        StateFlag::Demoed,
        StateFlag::WheelsContact,
    ];

    pub fn label(&self) -> &'static str {
        match self {
            StateFlag::OnGround => "is_on_ground",
            StateFlag::Jumping => "is_jumping",
            StateFlag::HasJumped => "has_jumped",
            StateFlag::Flipping => "is_flipping",
            StateFlag::HasDoubleJumped => "has_double_jumped",
            StateFlag::HasFlipped => "has_flipped",
            StateFlag::Boosting => "is_boosting",
            StateFlag::Supersonic => "is_supersonic",
            StateFlag::Demoed => "is_demoed",
            StateFlag::WheelsContact => "wheels_contact",
        }
    }
}

/// The recording's view of the two derived flags, matching `From<CarRecord>`.
fn record_double_jumped(real: &CarRecord) -> bool {
    real.double_jumped_or_flipped && !real.is_flipping
}
fn record_flipped(real: &CarRecord) -> bool {
    real.double_jumped_or_flipped && real.is_flipping
}

/// Wheels in contact, split front pair / back pair.
///
/// Compared per pair rather than as an exact four-bit mask because the
/// recording's left/right order within a pair cannot be pinned down from the
/// data: `{0,2}` and `{1,3}` are the two common side-lift masks in near-equal
/// numbers (3148 vs 3108 across the suite), `steer_amount` is never populated
/// so the front pair cannot be identified that way, and the sim's own
/// `right_dir_2d` naming contradicts the right-handed forward/up convention.
/// The front/back grouping *is* certain — indices 0,1 share a suspension length
/// and 2,3 share another whenever the car is level — and a per-pair count is
/// what `is_on_ground`'s `n >= 3` rule actually turns on, so nothing actionable
/// is lost. A left/right lift swap is the one blind spot.
fn pred_wheel_pairs(pred: &CarState) -> (u8, u8) {
    let w = &pred.wheels_with_contact;
    (
        u8::from(w[0]) + u8::from(w[1]),
        u8::from(w[2]) + u8::from(w[3]),
    )
}

fn record_wheel_pairs(real: &CarRecord) -> (u8, u8) {
    let w = &real.wheels;
    (
        u8::from(w[0].has_contact) + u8::from(w[1].has_contact),
        u8::from(w[2].has_contact) + u8::from(w[3].has_contact),
    )
}

fn flag_value(flag: StateFlag, pred: &CarState, real: &CarRecord) -> (bool, bool) {
    match flag {
        StateFlag::OnGround => (pred.is_on_ground, real.is_on_ground),
        StateFlag::Jumping => (pred.is_jumping, real.is_jumping),
        StateFlag::HasJumped => (pred.has_jumped, real.has_jumped),
        StateFlag::Flipping => (pred.is_flipping, real.is_flipping),
        StateFlag::HasDoubleJumped => (pred.has_double_jumped, record_double_jumped(real)),
        StateFlag::HasFlipped => (pred.has_flipped, record_flipped(real)),
        StateFlag::Boosting => (pred.is_boosting, real.is_boosting),
        StateFlag::Supersonic => (pred.is_supersonic, real.is_supersonic),
        StateFlag::Demoed => (pred.is_demoed, real.is_demoed),
        // Handled by `pred_wheel_pairs` / `record_wheel_pairs` in `record`;
        // it is a pair of counts rather than a single boolean.
        StateFlag::WheelsContact => (false, false),
    }
}

/// Running absolute-error stats for a float timer (air_time, jump_time).
#[derive(Debug, Clone, Default)]
pub struct TimerStats {
    pub count: u64,
    pub sum_abs: f64,
    pub max: f32,
}

impl TimerStats {
    fn add(&mut self, err: f32) {
        self.count += 1;
        self.sum_abs += err.abs() as f64;
        self.max = self.max.max(err.abs());
    }

    pub fn mean(&self) -> f64 {
        if self.count == 0 {
            0.0
        } else {
            self.sum_abs / self.count as f64
        }
    }

    fn merge(&mut self, other: TimerStats) {
        self.count += other.count;
        self.sum_abs += other.sum_abs;
        self.max = self.max.max(other.max);
    }
}

#[derive(Debug, Clone, Default)]
pub struct StateStats {
    pub total: u64,
    pub mismatches: [u64; StateFlag::ALL.len()],
    /// air_time error, counted while either side is airborne.
    pub air_time_err: TimerStats,
    /// jump_time error, counted while either side is in the jump phase.
    pub jump_time_err: TimerStats,
    /// Absolute difference in the number of wheels in contact, counted on every
    /// tick. `TimerStats` is just a running abs-error accumulator; the name is
    /// historical.
    pub wheel_count_err: TimerStats,
}

impl StateStats {
    pub fn record(&mut self, pred: &CarState, real: &CarRecord) {
        self.total += 1;
        let pred_pairs = pred_wheel_pairs(pred);
        let real_pairs = record_wheel_pairs(real);
        for (i, flag) in StateFlag::ALL.iter().enumerate() {
            let mismatched = if matches!(flag, StateFlag::WheelsContact) {
                pred_pairs != real_pairs
            } else {
                let (p, r) = flag_value(*flag, pred, real);
                p != r
            };
            if mismatched {
                self.mismatches[i] += 1;
            }
        }
        let pred_n = i16::from(pred_pairs.0 + pred_pairs.1);
        let real_n = i16::from(real_pairs.0 + real_pairs.1);
        self.wheel_count_err.add(f32::from(pred_n - real_n));
        if !pred.is_on_ground || !real.is_on_ground {
            self.air_time_err.add(pred.air_time - real.air_time);
        }
        if pred.is_jumping || real.is_jumping {
            self.jump_time_err.add(pred.jump_time() - real.jump_time);
        }
    }

    pub fn mismatch_rate(&self, flag: StateFlag) -> f64 {
        if self.total == 0 {
            return 0.0;
        }
        let idx = StateFlag::ALL.iter().position(|f| *f == flag).unwrap();
        self.mismatches[idx] as f64 / self.total as f64
    }

    /// Combine another accumulator into this one (parallel shard merge).
    pub fn merge(&mut self, other: StateStats) {
        self.total += other.total;
        for i in 0..self.mismatches.len() {
            self.mismatches[i] += other.mismatches[i];
        }
        self.air_time_err.merge(other.air_time_err);
        self.jump_time_err.merge(other.jump_time_err);
        self.wheel_count_err.merge(other.wheel_count_err);
    }
}
