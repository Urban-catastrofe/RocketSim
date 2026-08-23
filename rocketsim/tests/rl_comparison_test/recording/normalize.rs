//! Translate the RLPR observer's state-machine semantics into the sim's.
//!
//! The observer and the sim record some flags with genuinely different meanings.
//! Restoring such a field verbatim does not add noise — it drives the sim into a
//! state the recording never described, so the measured divergence is the
//! harness's fault rather than the physics'. Normalising once at load time keeps
//! every consumer (per-tick gate, residual decomposition, rollout, deep dive and
//! the C++ side-by-side) on one consistent interpretation.

use glam::Vec3A;
use rocketsim::consts::car::{drive, jump};
use rocketsim::consts::{TICK_RATE, TICK_TIME};

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

/// A car-car bump lands at a sub-frame time just like a jump press, so the
/// recorded tick that carries it is not reproducible from a frame-aligned
/// restore. Unlike a jump there is no flag to read: the observer records no
/// bump event, so the onset has to be recovered from the kinematics.
///
/// The marker is Rocket League's own position/velocity *self*-consistency. On
/// an ordinary tick the recording satisfies `pos[t] - pos[t-1] == vel[t] * dt`
/// to within float noise — measured over 477939 non-teleport car/ticks of the
/// suite, that residual runs p50 = 0.004, p90 = 0.006, p99 = 0.21 UU. When an
/// impulse is applied partway through a step the identity has to break, because
/// part of the step was travelled at the pre-impulse velocity. On
/// `car_car_basic_bump` the bump tick shows 1.90 UU (bumper) and 4.29 UU
/// (victim) — two to three orders of magnitude above the p90 floor. Solving
/// `dpos = [v_pre*phi + v_post*(1-phi)] * dt` for the victim gives phi = 0.41,
/// i.e. the impulse landed 41% into the tick.
///
/// The residual is exactly the right criterion rather than a proxy: a bump that
/// happened to land at phi = 0 leaves the identity intact, and that tick really
/// is reproducible by a frame-aligned restore. Only ticks that are provably
/// unmeasurable get marked.
///
/// Two confounders also break the identity and are excluded first:
///
/// - **Teleports** (demo respawn, kickoff reset) move a car hundreds of UU in
///   one tick. Caught by the displacement bound.
/// - **Dropped logger frames**, which are real: `2v2_1` cars 1 and 3 hold
///   `|dpos|` = 38.4 UU for 18 consecutive ticks at exactly 2300 UU/s, which is
///   precisely `2 * vel * dt`. Caught by requiring `physics_frame` to advance by
///   exactly `stride`.
///
/// Proximity to another car is required on top, so that sub-frame events from
/// other subsystems (ball hits, world contacts) are not swept up as bumps.
pub fn detect_bump_onsets(ticks: &[TickRecord], num_cars: usize, stride: usize) -> Vec<Vec<bool>> {
    /// Position/velocity inconsistency (UU) above which a step cannot have been
    /// a single frame-aligned integration. 40x the measured p90 of 0.006 and
    /// still ~8x below the smallest bump residual observed (1.90).
    const SUBFRAME_POS_RESIDUAL: f32 = 0.25;

    /// Centre separation (UU) within which two cars can be in contact. The
    /// Octane hitbox (120.507 x 86.6994 x 38.6591, offset (13.8757, 0, 20.755))
    /// reaches 94.8 UU from the car origin at its furthest corner, so two of
    /// them touch at up to 189.5 UU apart; one tick of maximum closing travel
    /// (2 * 2300 / 120 = 38.3) is added on top.
    const BUMP_PROXIMITY: f32 = 240.0;

    /// One tick can carry at most `MAX_SPEED / TICK_RATE` = 19.2 UU. Anything
    /// well beyond that is a teleport, not a physics step.
    const MAX_STEP_DISPLACEMENT: f32 = 40.0;

    let dt = TICK_TIME * stride as f32;
    let mut onsets = vec![vec![false; num_cars]; ticks.len()];

    for t in stride..ticks.len() {
        let (from, to) = (&ticks[t - stride], &ticks[t]);
        for (car, mark) in onsets[t].iter_mut().enumerate() {
            let (Some(a), Some(b)) = (from.car_records.get(car), to.car_records.get(car)) else {
                continue;
            };
            if a.is_demoed || b.is_demoed {
                continue;
            }
            if b.phys.physics_frame != a.phys.physics_frame + stride as u32 {
                continue;
            }

            let p0: Vec3A = a.phys.pos.into();
            let p1: Vec3A = b.phys.pos.into();
            let v1: Vec3A = b.phys.lin_vel.into();
            let step = p1 - p0;
            if step.length() > MAX_STEP_DISPLACEMENT {
                continue;
            }
            if (step - v1 * dt).length() <= SUBFRAME_POS_RESIDUAL {
                continue;
            }

            let near_other_car = (0..num_cars).filter(|&o| o != car).any(|o| {
                to.car_records.get(o).is_some_and(|other| {
                    !other.is_demoed && (Vec3A::from(other.phys.pos) - p1).length() < BUMP_PROXIMITY
                })
            });
            *mark = near_other_car;
        }
    }

    onsets
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
                let jump_ticks = (rec.jump_time * TICK_RATE).round() as u32;
                if jump_ticks >= jump::MAX_TICKS || (jump_ticks >= jump::MIN_TICKS && !held) {
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

/// Rebuild `handbrake_val` from the handbrake *button*, and report how many
/// (tick, car) pairs changed.
///
/// The observer stores this field as the button state snapped to 0 or 1 - in
/// `car_drift_powerslide_turn` it drops from 1 to 0 within a single tick - but
/// Rocket League ramps it, exactly as the sim does in `Car::update_wheels`, at
/// `POWERSLIDE_RISE_RATE` up and `POWERSLIDE_FALL_RATE` down.
///
/// The ramp is directly measurable. On the closed lateral axis (see
/// `latdump`) the lateral friction factor Rocket League
/// applies after a release climbs by 0.0150 per tick for the full 60 ticks,
/// against a predicted `0.9 * POWERSLIDE_FALL_RATE / 120` = 0.0150; fitting the
/// factor itself off the rebuilt ramp returns 0.1023 against the shipped
/// `HANDBRAKE_LAT_FRICTION_FACTOR` of 0.1.
///
/// Restoring the snapped value drives the sim to full lateral grip where the game
/// still had almost none, and it is the largest single error on that channel:
/// mean `|sim - RL|` over the affected steps is 6.34 uu/s on the release side and
/// 6.45 on the press side, against 0.77 and 0.68 once the ramp is rebuilt. Where
/// the ramp is settled the snapped and rebuilt values agree exactly and the
/// channel is already accurate to 0.006 uu/s - so the ramp is the whole defect,
/// and the friction constants underneath it were never wrong.
///
/// This also explains why `RLCENSUS=3` never produced an `hb-ramp` band: the
/// field it bands on only ever held 0 or 1. That was a property of the log rather
/// than of Rocket League, and it had been read as evidence that
/// `POWERSLIDE_RISE_RATE` was not involved in anything.
///
/// `prev_controls` is the input applied over the step *into* a tick, and
/// `update_wheels` increments before it reads, so the value stored at tick `t` is
/// the state the sim must be restored to before stepping out of `t`. Only 120 Hz
/// recordings get this far, so one array entry is one tick.
pub fn normalize_handbrake_val(ticks: &mut [TickRecord], num_cars: usize) -> usize {
    // A recording can open mid-slide, and the snapped log is the only evidence
    // available for where the ramp already was.
    let mut val: Vec<f32> = (0..num_cars)
        .map(|car| {
            ticks
                .first()
                .and_then(|tick| tick.car_records.get(car))
                .map_or(0.0, |rec| rec.handbrake_val)
        })
        .collect();
    let mut changed = 0;

    for (t, tick) in ticks.iter_mut().enumerate() {
        for (car, value) in val.iter_mut().enumerate() {
            let Some(rec) = tick.car_records.get_mut(car) else {
                continue;
            };
            if t > 0 {
                let rate = if rec.prev_controls.handbrake {
                    drive::POWERSLIDE_RISE_RATE
                } else {
                    -drive::POWERSLIDE_FALL_RATE
                };
                *value = (*value + rate * TICK_TIME).clamp(0.0, 1.0);
            }
            if *value != rec.handbrake_val {
                rec.handbrake_val = *value;
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

    /// Build a one-car tick from `(handbrake button, logged handbrake_val)`.
    fn hb_tick(cars: &[(bool, f32)]) -> TickRecord {
        let car_records = cars
            .iter()
            .map(|&(held, logged)| {
                let mut rec: CarRecord = unsafe { std::mem::zeroed() };
                rec.prev_controls.handbrake = held;
                rec.handbrake_val = logged;
                rec
            })
            .collect();
        TickRecord {
            car_records,
            ball_record: unsafe { std::mem::zeroed::<PhysRecord>() },
        }
    }

    fn run_hb(rows: &[(bool, f32)]) -> Vec<f32> {
        let mut ticks: Vec<TickRecord> = rows.iter().map(|r| hb_tick(&[*r])).collect();
        normalize_handbrake_val(&mut ticks, 1);
        ticks
            .iter()
            .map(|t| t.car_records[0].handbrake_val)
            .collect()
    }

    /// The log snaps to 0 on release; the rebuilt value has to decay at
    /// `POWERSLIDE_FALL_RATE`, which is what Rocket League measurably does.
    #[test]
    fn release_decays_at_fall_rate() {
        let got = run_hb(&[(true, 1.0), (false, 0.0), (false, 0.0), (false, 0.0)]);
        let step = drive::POWERSLIDE_FALL_RATE * TICK_TIME;
        assert_eq!(got[0], 1.0, "the seed is the logged value at tick 0");
        for (i, v) in got.iter().enumerate().skip(1) {
            let want = 1.0 - step * i as f32;
            assert!((v - want).abs() < 1e-6, "tick {i}: {v} != {want}");
        }
    }

    /// The log snaps to 1 on press; the rebuilt value has to rise at
    /// `POWERSLIDE_RISE_RATE` and clamp there.
    #[test]
    fn press_rises_at_rise_rate_and_clamps() {
        let rows: Vec<(bool, f32)> = std::iter::once((false, 0.0))
            .chain(std::iter::repeat_n((true, 1.0), 40))
            .collect();
        let got = run_hb(&rows);
        let step = drive::POWERSLIDE_RISE_RATE * TICK_TIME;
        assert!(
            (got[1] - step).abs() < 1e-6,
            "first held tick is one step up"
        );
        assert!(got[10] < 1.0, "still ramping after 10 ticks");
        assert_eq!(got[40], 1.0, "clamped at 1 once the ramp completes");
    }

    /// A tap shorter than the rise time never reaches full powerslide, which is
    /// the whole reason the snapped log is wrong for real play.
    #[test]
    fn short_tap_never_reaches_full() {
        let got = run_hb(&[
            (false, 0.0),
            (true, 1.0),
            (true, 1.0),
            (false, 0.0),
            (false, 0.0),
        ]);
        let up = drive::POWERSLIDE_RISE_RATE * TICK_TIME;
        let down = drive::POWERSLIDE_FALL_RATE * TICK_TIME;
        assert!((got[2] - 2.0 * up).abs() < 1e-6, "two ticks of rise only");
        assert!((got[4] - (2.0 * up - 2.0 * down)).abs() < 1e-6);
        assert!(
            got.iter().all(|&v| v < 1.0),
            "a 2-tick tap cannot reach 1.0"
        );
    }

    /// Never leaves the valid range, however long the button is held either way.
    #[test]
    fn stays_within_range() {
        let rows: Vec<(bool, f32)> = std::iter::repeat_n((false, 0.0), 200).collect();
        assert!(run_hb(&rows).iter().all(|&v| (0.0..=1.0).contains(&v)));
    }

    /// Cars ramp independently.
    #[test]
    fn handbrake_cars_are_independent() {
        let mut ticks = vec![
            hb_tick(&[(true, 1.0), (false, 0.0)]),
            hb_tick(&[(true, 1.0), (false, 0.0)]),
        ];
        normalize_handbrake_val(&mut ticks, 2);
        assert_eq!(ticks[1].car_records[0].handbrake_val, 1.0);
        assert_eq!(ticks[1].car_records[1].handbrake_val, 0.0);
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
