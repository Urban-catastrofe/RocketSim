//! Recording event diagnostic.
//!
//! Scans a recording for discrete, non-physics events that a pure per-tick
//! physics comparison cannot reproduce: cars teleporting (goal resets, demos,
//! respawns) and car-index reordering by the logger. Run with `RLDIAG=1` to
//! dump these for a case instead of the normal measurement.

use glam::Vec3A;

use super::recording::Recording;

/// A car cannot physically move more than this in one recorded step
/// (max speed ~2300 UU/s at 120 Hz ≈ 19 UU/tick); anything larger is a
/// game-state event, not physics.
const TELEPORT_DIST: f32 = 100.0;
/// The ball is past the goal line (a goal was scored) beyond this |y|.
const GOAL_LINE_Y: f32 = 5120.0;

pub fn dump_events(recording: &Recording) {
    let stride = recording.stride;
    let num_cars = recording.info.num_cars as usize;
    let n = recording.ticks.len().saturating_sub(stride);

    println!(
        "[{}] RLDIAG num_cars={} ticks={} stride={}",
        recording.name,
        num_cars,
        recording.ticks.len(),
        stride
    );

    let mut teleport_events = 0usize;
    let mut affected_ticks = 0usize;
    let mut demo_ticks = 0usize;
    let mut goal_ticks = 0usize;
    // `is_demoed` is never set by the observer -- across the whole suite it
    // reads false on every car-tick. What actually marks a demolished car is
    // the logger parking it at the arena origin, which `is_car_sentinel`
    // detects. A live -> parked transition is therefore the only demolition
    // signal the ground truth carries, and it is the only thing a demo
    // detector can be scored against.
    let mut park_onsets = 0usize;
    let mut parked_ticks = 0usize;
    // The other shape a demolition could take in the log: RL removes the car
    // for `RESPAWN_TIME` (3 s = 360 ticks) and the observer keeps emitting its
    // last pose, so the car would sit at *exactly* one position for a long run
    // and then teleport to a spawn point. Anything at least half a respawn long
    // is reported; a car legitimately parked by a player still drifts on its
    // suspension, so exact equality over hundreds of ticks is not a pose a live
    // car reaches.
    const FROZEN_MIN_TICKS: usize = 180;
    let mut frozen_since = vec![0usize; num_cars];
    let mut frozen_spans = 0usize;

    for i in (0..n).step_by(stride) {
        let from = &recording.ticks[i];
        let to = &recording.ticks[i + stride];

        let ball_y: f32 = to.ball_record.pos.y;
        if ball_y.abs() > GOAL_LINE_Y {
            goal_ticks += 1;
        }

        let mut tick_has_teleport = false;
        for c in 0..num_cars {
            let fc = &from.car_records[c];
            let tc = &to.car_records[c];

            if tc.is_demoed {
                demo_ticks += 1;
            }
            let was_parked = super::runner::is_car_sentinel(&fc.phys);
            let is_parked = super::runner::is_car_sentinel(&tc.phys);
            if is_parked {
                parked_ticks += 1;
            }
            if is_parked && !was_parked {
                park_onsets += 1;
                println!(
                    "[{}]   t={:>5} car{} PARKED (demolition) from {:?} at |v|={:.0}",
                    recording.name,
                    i,
                    c,
                    [fc.phys.pos.x, fc.phys.pos.y, fc.phys.pos.z],
                    Vec3A::from(fc.phys.lin_vel).length(),
                );
            }

            let a: Vec3A = fc.phys.pos.into();
            let b: Vec3A = tc.phys.pos.into();
            let dist = (b - a).length();
            if a == b {
                frozen_since[c] += 1;
            } else {
                if frozen_since[c] >= FROZEN_MIN_TICKS {
                    frozen_spans += 1;
                    println!(
                        "[{}]   t={:>5} car{} FROZEN for {} ticks at {:?}, then moved {:.0}",
                        recording.name,
                        i,
                        c,
                        frozen_since[c],
                        [a.x, a.y, a.z],
                        dist,
                    );
                }
                frozen_since[c] = 0;
            }
            if dist > TELEPORT_DIST {
                teleport_events += 1;
                tick_has_teleport = true;
                println!(
                    "[{}]   t={:>5} car{} TELEPORT dist={:7.0} demo {}->{} | {:?} -> {:?}",
                    recording.name,
                    i,
                    c,
                    dist,
                    fc.is_demoed as u8,
                    tc.is_demoed as u8,
                    [a.x, a.y, a.z],
                    [b.x, b.y, b.z],
                );
            }
        }
        if tick_has_teleport {
            affected_ticks += 1;
        }
    }

    let total_steps = n / stride.max(1);
    println!(
        "[{}] RLDIAG summary: {} teleport events across {} steps ({} distinct ticks, {:.1}%), {} ball-in-goal ticks, {} car-ticks is_demoed, {} origin-parked car-ticks in {} parkings, {} frozen spans",
        recording.name,
        teleport_events,
        total_steps,
        affected_ticks,
        affected_ticks as f64 / total_steps.max(1) as f64 * 100.0,
        goal_ticks,
        demo_ticks,
        parked_ticks,
        park_onsets,
        frozen_spans,
    );
}
