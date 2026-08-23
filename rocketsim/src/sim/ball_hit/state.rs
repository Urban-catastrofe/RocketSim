//! Per-ball state the impulse subsystem needs to enforce its cadence.

/// Stateful bookkeeping for the extra hit impulse.
#[derive(Clone, Copy, Debug)]
pub struct BallHitState {
    /// The tick the impulse last fired on, if ever.
    pub last_impulse_tick: Option<u64>,
    /// The tick the ball was last in contact with a car, if ever.
    pub last_contact_tick: Option<u64>,
}

impl Default for BallHitState {
    fn default() -> Self {
        Self::DEFAULT
    }
}

impl BallHitState {
    pub const DEFAULT: Self = Self {
        last_impulse_tick: None,
        last_contact_tick: None,
    };
}
