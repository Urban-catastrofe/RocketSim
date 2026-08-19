//! Ground-truth sanity checks on a recording, run before it is measured.
//!
//! A recording is the *reference* the sim is graded against, so a physically
//! impossible one does not merely add noise — it inverts the metric, penalising
//! the sim for being right. These checks reject such recordings loudly instead
//! of letting them silently set the accuracy bar.
//!
//! The check that motivated this module: a large share of the original `car_*`
//! recordings hold the jump button while the car is flagged `is_on_ground`, set
//! `has_jumped`, count `jump_time` up — and never move the car at all (velocity
//! along the car's own up-axis peaks at 0.1–0.4 UU/s, where a real Rocket League
//! jump reaches ~300). Those cars were pinned by a state-set loop while
//! recording, so the inputs reached the state machine but the physics was
//! overridden. Grading a correct launch against them costs 300–450 UU/s and
//! made flips the worst-scoring mechanism in the suite.

use glam::Vec3A;

use super::recording::Recording;
use super::recording::cpp_records::CarRecord;

/// Ticks of recording that must remain after a grounded jump press for the
/// launch to be observable. RL applies the impulse on the press tick, so a
/// couple of ticks is plenty; requiring more would let a press at the very end
/// of a recording masquerade as a failure.
const RESPONSE_WINDOW: usize = 3;

/// Peak velocity along the car's own up-axis that any genuine jump exceeds.
/// Real recordings reach ~301–307 UU/s; the broken ones peak below 0.5. The
/// threshold sits far from both so it never has to arbitrate a close call.
const MIN_JUMP_UP_VEL: f32 = 50.0;

/// A ground-truth defect found in a recording, phrased for whoever has to act
/// on it.
#[derive(Debug, Clone)]
pub struct GroundTruthError {
    pub car: usize,
    pub detail: String,
}

/// Velocity along the car's own up-axis, so a car on a wall, on the ceiling or
/// upside-down is judged by the direction it would actually launch, not world Z.
fn up_axis_speed(car: &CarRecord) -> f32 {
    let up: Vec3A = car.phys.rot.rows[2].into();
    let vel: Vec3A = car.phys.lin_vel.into();
    vel.dot(up)
}

/// Reject recordings whose own state is not physically reachable in Rocket
/// League. Returns one entry per offending car.
///
/// Checks grounded jump presses: RL launches a grounded car at ~300 UU/s along
/// its up-axis on the press tick, so a press that produces no up-axis velocity
/// within [`RESPONSE_WINDOW`] ticks did not happen in physics. A recording is
/// only rejected when **every** observable press fails — a single failure could
/// be a legitimate edge case (a car wedged under the ball, a press on the tick
/// the car leaves the ground), but a recording where none of them ever launches
/// is a pinned car.
pub fn check_ground_truth(recording: &Recording) -> Vec<GroundTruthError> {
    let num_cars = recording.info.num_cars as usize;
    let ticks = &recording.ticks;
    let mut errors = Vec::new();

    for car in 0..num_cars {
        let mut evaluable = 0usize;
        let mut launched = 0usize;
        let mut best_response = f32::NEG_INFINITY;

        for i in 0..ticks.len() {
            let Some(rec) = ticks[i].car_records.get(car) else {
                continue;
            };
            // A demoed car is parked by the logger; its pose is not physics.
            if rec.is_demoed || !rec.prev_controls.jump || !rec.is_on_ground {
                continue;
            }
            // Only judge presses whose launch would still be in the recording.
            if i + RESPONSE_WINDOW >= ticks.len() {
                continue;
            }

            evaluable += 1;
            let mut peak = f32::NEG_INFINITY;
            for t in &ticks[i..=i + RESPONSE_WINDOW] {
                if let Some(r) = t.car_records.get(car) {
                    peak = peak.max(up_axis_speed(r));
                }
            }
            best_response = best_response.max(peak);
            if peak >= MIN_JUMP_UP_VEL {
                launched += 1;
            }
        }

        if evaluable > 0 && launched == 0 {
            errors.push(GroundTruthError {
                car,
                detail: format!(
                    "{evaluable} grounded jump press(es) produced no launch — the best velocity \
                     along its own up-axis within {RESPONSE_WINDOW} ticks of any press was \
                     {best_response:.2} UU/s, where a real grounded jump reaches ~300. The \
                     recorded car was pinned by a state-set loop: the inputs reached the state \
                     machine but the physics was overridden, so this recording cannot grade jump \
                     or flip behaviour."
                ),
            });
        }
    }

    errors
}

/// Panic with an actionable message if the recording cannot serve as ground
/// truth. Called before measurement so a bad recording can never quietly set
/// the accuracy bar.
pub fn assert_usable_ground_truth(recording: &Recording) {
    let errors = check_ground_truth(recording);
    if errors.is_empty() {
        return;
    }

    let mut msg = format!(
        "[{}] UNUSABLE GROUND TRUTH — this recording is not physically reachable in Rocket \
         League, so measuring the sim against it is meaningless:\n",
        recording.name,
    );
    for e in &errors {
        msg.push_str(&format!("  car_{}: {}\n", e.car, e.detail));
    }
    msg.push_str(
        "Move the .rlpr to tests/rl_comparison_test/test_recordings_invalid/ and re-record it. \
         See how-to-test.md \"Ground-Truth Validation\".",
    );
    panic!("{msg}");
}
