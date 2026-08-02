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
}

impl StateFlag {
    pub const ALL: [StateFlag; 9] = [
        StateFlag::OnGround,
        StateFlag::Jumping,
        StateFlag::HasJumped,
        StateFlag::Flipping,
        StateFlag::HasDoubleJumped,
        StateFlag::HasFlipped,
        StateFlag::Boosting,
        StateFlag::Supersonic,
        StateFlag::Demoed,
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
    }
}

#[derive(Debug, Clone, Default)]
pub struct StateStats {
    pub total: u64,
    pub mismatches: [u64; StateFlag::ALL.len()],
}

impl StateStats {
    pub fn record(&mut self, pred: &CarState, real: &CarRecord) {
        self.total += 1;
        for (i, flag) in StateFlag::ALL.iter().enumerate() {
            let (p, r) = flag_value(*flag, pred, real);
            if p != r {
                self.mismatches[i] += 1;
            }
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
    }
}
