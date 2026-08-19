//! Translate the RLPR observer's state-machine semantics into the sim's.
//!
//! The observer and the sim record some flags with genuinely different meanings.
//! Restoring such a field verbatim does not add noise — it drives the sim into a
//! state the recording never described, so the measured divergence is the
//! harness's fault rather than the physics'. Normalising once at load time keeps
//! every consumer (per-tick gate, residual decomposition, rollout, deep dive and
//! the C++ side-by-side) on one consistent interpretation.

use rocketsim::consts::car::jump;

use super::tick_record::TickRecord;

/// Mark, per `[tick][car]`, the ticks on which Rocket League *began* applying a
/// button-triggered impulse: a jump, a double jump or a flip.
///
/// Such an impulse is applied at the moment of the press, which is not aligned
/// to the 120 Hz physics grid. The press lands at some sub-frame phase φ and the
/// impulse is split across the two recorded frames that straddle it, in
/// proportion (1−φ):φ. φ is not recorded anywhere, so neither of those two
/// frames can be reproduced from a frame-aligned restore — the sim applies the
/// whole impulse on whichever step carries the control's rising edge. Only the
/// *total* across the pair is well defined, and it is exact: every jump onset
/// sums to `jump::IMMEDIATE_FORCE` = 291.67 UU/s however φ falls, while the
/// single-frame share ranges from 15.2 to 295.6 UU/s across recordings of the
/// identical mechanic.
///
/// Both the observer's `is_jumping` and `is_flipping` are one-tick pulses on the
/// activation tick (1626 and 1314 events in the suite, every run length exactly
/// 1), which is precisely the marker needed. They coincide on 65 ticks, where
/// one event is simply marked once.
///
/// Must run on the **raw** records, before [`normalize_jump_active`] rewrites
/// `is_jumping` — that pass deliberately clears the airborne double-jump pulses,
/// which are onsets all the same.
pub fn detect_impulse_onsets(ticks: &[TickRecord], num_cars: usize) -> Vec<Vec<bool>> {
    ticks
        .iter()
        .map(|tick| {
            (0..num_cars)
                .map(|car| {
                    tick.car_records
                        .get(car)
                        .is_some_and(|rec| rec.is_jumping || rec.is_flipping)
                })
                .collect()
        })
        .collect()
}

/// Rewrite `is_jumping` from the observer's activation *pulse* into the sim's
/// sustained flag, and report how many (tick, car) pairs changed.
///
/// The observer sets `is_jumping` for exactly one tick — the activation tick —
/// then clears it while `jump_time` keeps counting up and the car keeps
/// accelerating upward. Every one of the 1626 `is_jumping` runs in the recording
/// suite has length exactly 1. The sim instead holds `is_jumping` for as long as
/// the jump produces thrust, and applies `jump::ACCEL` only while it is set.
///
/// Restoring the pulse verbatim therefore switched the sustained jump accel off
/// on every tick of an ascent but the first, losing a flat
/// `jump::ACCEL * TICK_TIME` ≈ 12.15 UU/s of vertical velocity per tick — which
/// is exactly the 12.167 UU/s error the gate reported on the interior ticks of
/// `car_jump_after_turning_left`.
///
/// This has to be a whole-recording pass rather than a per-tick rule: telling
/// "this jump is still running" apart from "the button was pressed again while
/// `jump_time` happens to still be small" needs the control history. A local
/// rule using only the current tick disagrees with the replayed truth on 6086
/// ticks of the suite, every one of them a jump that had already ended.
///
/// The activation tick stays recoverable as `is_jumping && jump_time == 0.0`,
/// which is how the immediate-force guards identify it.
pub fn normalize_jump_active(ticks: &mut [TickRecord], num_cars: usize) -> usize {
    let mut active = vec![false; num_cars];
    let mut changed = 0;

    for tick in ticks.iter_mut() {
        for (car, is_active) in active.iter_mut().enumerate() {
            let Some(rec) = tick.car_records.get_mut(car) else {
                *is_active = false;
                continue;
            };

            if rec.is_jumping && rec.is_on_ground {
                // A grounded activation pulse arms the jump. The `is_on_ground`
                // guard matters: the observer also pulses `is_jumping` on an
                // airborne *double* jump (71 of the suite's 1626 pulses), but
                // Rocket League only ever starts a sustained jump from the
                // ground — a double jump is impulse-only and gets no thrust —
                // and the sim likewise arms `is_jumping` only in
                // `update_jump`'s grounded branch. Arming on those pulses would
                // invent sustained thrust that neither the game nor the sim has.
                *is_active = true;
            } else if *is_active {
                // Disarm with the sim's own continuation rule (`update_jump`):
                // a jump survives while it is inside the minimum-jump window,
                // or while the button is held and the maximum time has not
                // elapsed. `prev_controls` is the input that was applied over
                // the step *into* this tick, so it is what kept the jump alive.
                let held = rec.prev_controls.jump;
                if rec.jump_time >= jump::MAX_TIME || (rec.jump_time >= jump::MIN_TIME && !held) {
                    *is_active = false;
                }
            }

            if *is_active != rec.is_jumping {
                rec.is_jumping = *is_active;
                changed += 1;
            }
        }
    }

    changed
}

#[cfg(test)]
mod tests {
    use super::super::cpp_records::{CarRecord, PhysRecord};
    use super::*;

    /// `CarRecord`/`PhysRecord` are `#[repr(C)]` plain-old-data (the parser
    /// reads them straight out of the file), so zeroing is a valid way to get a
    /// blank record to fill in the few fields these tests care about.
    fn tick(cars: &[(bool, bool, f32, bool)]) -> TickRecord {
        let car_records = cars
            .iter()
            .map(|&(is_jumping, is_on_ground, jump_time, held)| {
                let mut rec: CarRecord = unsafe { std::mem::zeroed() };
                rec.is_jumping = is_jumping;
                rec.is_on_ground = is_on_ground;
                rec.jump_time = jump_time;
                rec.prev_controls.jump = held;
                rec
            })
            .collect();
        TickRecord {
            car_records,
            ball_record: unsafe { std::mem::zeroed::<PhysRecord>() },
        }
    }

    fn run(rows: &[(bool, bool, f32, bool)]) -> Vec<bool> {
        let mut ticks: Vec<TickRecord> = rows.iter().map(|r| tick(&[*r])).collect();
        normalize_jump_active(&mut ticks, 1);
        ticks.iter().map(|t| t.car_records[0].is_jumping).collect()
    }

    /// A held jump: the observer's single pulse must become a sustained flag
    /// that survives until `MAX_TIME`, so the sim keeps applying jump accel.
    #[test]
    fn held_jump_stays_active_until_max_time() {
        // (pulse, grounded, jump_time, button held)
        let got = run(&[
            (false, true, 0.0, false), // pre-jump
            (true, true, 0.0, true),   // observer pulse
            (false, true, 0.0083, true),
            (false, false, 0.05, true),
            (false, false, 0.19, true),
            (false, false, 0.20, true), // MAX_TIME reached -> ends
            (false, false, 0.21, true),
        ]);
        assert_eq!(
            got,
            vec![false, true, true, true, true, false, false],
            "a held jump must stay active from the pulse until MAX_TIME"
        );
    }

    /// Releasing the button ends the jump, but only once the minimum-jump
    /// window has elapsed — that window is why a tapped jump still launches.
    #[test]
    fn released_jump_survives_min_window_then_ends() {
        let got = run(&[
            (true, true, 0.0, true),      // pulse
            (false, true, 0.0083, false), // released, still inside MIN_TIME
            (false, true, 0.0167, false), // still inside MIN_TIME
            (false, true, 0.025, false),  // MIN_TIME reached, not held -> ends
            (false, true, 0.0333, false),
        ]);
        assert_eq!(got, vec![true, true, true, false, false]);
    }

    /// The observer also pulses `is_jumping` on an airborne double jump, which
    /// produces an impulse but no sustained thrust. Arming there would invent
    /// acceleration that neither the game nor the sim applies.
    #[test]
    fn airborne_pulse_does_not_arm() {
        let got = run(&[
            (true, false, 0.0, true), // double-jump pulse, airborne
            (false, false, 0.0083, true),
            (false, false, 0.0167, true),
        ]);
        assert_eq!(got, vec![false, false, false]);
    }

    /// A second press while `jump_time` is still below `MAX_TIME` must not
    /// revive the finished jump — the case a per-tick rule gets wrong.
    #[test]
    fn re_press_after_release_does_not_revive_jump() {
        let got = run(&[
            (true, true, 0.0, true),     // pulse
            (false, true, 0.03, false),  // released past MIN_TIME -> ends
            (false, false, 0.05, true),  // pressed again, airborne
            (false, false, 0.075, true), // still below MAX_TIME
        ]);
        assert_eq!(
            got,
            vec![true, false, false, false],
            "only a grounded pulse may arm the sustained flag"
        );
    }

    /// Cars are tracked independently.
    #[test]
    fn cars_are_independent() {
        let mut ticks = vec![
            tick(&[(true, true, 0.0, true), (false, true, 0.0, false)]),
            tick(&[(false, true, 0.0083, true), (false, true, 0.0, false)]),
        ];
        normalize_jump_active(&mut ticks, 2);
        assert!(ticks[1].car_records[0].is_jumping);
        assert!(!ticks[1].car_records[1].is_jumping);
    }
}
