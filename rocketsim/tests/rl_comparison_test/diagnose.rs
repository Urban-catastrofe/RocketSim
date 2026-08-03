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

            let a: Vec3A = fc.phys.pos.into();
            let b: Vec3A = tc.phys.pos.into();
            let dist = (b - a).length();
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
        "[{}] RLDIAG summary: {} teleport events across {} steps ({} distinct ticks, {:.1}%), {} ball-in-goal ticks, {} car-ticks is_demoed",
        recording.name,
        teleport_events,
        total_steps,
        affected_ticks,
        affected_ticks as f64 / total_steps.max(1) as f64 * 100.0,
        goal_ticks,
        demo_ticks,
    );
}
