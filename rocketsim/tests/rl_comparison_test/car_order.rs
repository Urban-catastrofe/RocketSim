//! Stabilize car identity across a recording.
//!
//! The logger's per-tick car array order is not reliable — it can rotate when
//! cars get close (and the game's player IDs are not stable enough to sort by),
//! so car index `j` is not always the same physical car. Left alone, that makes
//! index-based restore/compare fabricate huge divergence.
//!
//! This pass reorders every tick's car array into a *canonical* order tracked
//! across ticks, so downstream code sees a stable car `j`. It runs on the
//! parsed recording in memory, so it fixes existing recordings too.
//!
//! Matching uses each track's **velocity-predicted position** (where the car
//! should be this tick), not just its last position. That is what keeps two
//! close cars that are crossing from being swapped — position alone is
//! ambiguous there, but their predicted positions diverge along their
//! velocities.

use glam::Vec3A;

use super::recording::Recording;

#[derive(Clone, Copy)]
struct Track {
    pos: Vec3A,
    vel: Vec3A,
}

/// Reorder each tick's `car_records` so index `j` is consistently the same
/// physical car, tracked across ticks by velocity-predicted position.
pub fn reorder_cars(recording: &mut Recording) {
    let n = recording.info.num_cars as usize;
    if n <= 1 {
        return;
    }

    let dt = rocketsim::consts::TICK_TIME / recording.stride.max(1) as f32;
    let mut tracks: Vec<Option<Track>> = vec![None; n];

    // Zip the onset marks so they follow the same permutation; they are indexed
    // by canonical car and would otherwise silently point at the wrong car.
    let mut onsets = recording.impulse_onsets.iter_mut();

    for tick in recording.ticks.iter_mut() {
        let onset_row = onsets.next();
        if tick.car_records.len() != n {
            continue;
        }

        let pos: Vec<Vec3A> = tick
            .car_records
            .iter()
            .map(|c| -> Vec3A { c.phys.pos.into() })
            .collect();
        let vel: Vec<Vec3A> = tick
            .car_records
            .iter()
            .map(|c| -> Vec3A { c.phys.lin_vel.into() })
            .collect();

        // assignment[current_recorded_index] = canonical_track_index
        let assignment = match_to_tracks(&pos, &tracks, n, dt);

        // Place each recorded car into its canonical slot.
        let old = tick.car_records.clone();
        for (current, &canon) in assignment.iter().enumerate() {
            tick.car_records[canon] = old[current];
        }
        if let Some(row) = onset_row.filter(|r| r.len() == n) {
            let old_row = row.clone();
            for (current, &canon) in assignment.iter().enumerate() {
                row[canon] = old_row[current];
            }
        }

        // Advance the tracks (in canonical order).
        for (current, &canon) in assignment.iter().enumerate() {
            tracks[canon] = Some(Track {
                pos: pos[current],
                vel: vel[current],
            });
        }
    }
}

/// Bijective match of this tick's cars to the canonical tracks. Each track is
/// projected forward by its last velocity (`pos + vel * dt` = where that car
/// should be now) and cars are matched to those predictions, closest pair
/// first. Accurate for the small car counts we have (<= 8).
fn match_to_tracks(pos: &[Vec3A], tracks: &[Option<Track>], n: usize, dt: f32) -> Vec<usize> {
    let mut assignment = vec![0usize; n];

    // First tick (nothing tracked yet): establish the canonical order as-is.
    if tracks.iter().all(|t| t.is_none()) {
        for i in 0..n {
            assignment[i] = i;
        }
        return assignment;
    }

    let mut car_used = vec![false; n];
    let mut track_used = vec![false; n];

    for _ in 0..n {
        let mut best: Option<(f32, usize, usize)> = None;
        for c in 0..n {
            if car_used[c] {
                continue;
            }
            for t in 0..n {
                if track_used[t] {
                    continue;
                }
                let d = match tracks[t] {
                    Some(tr) => (tr.pos + tr.vel * dt - pos[c]).length_squared(),
                    None => f32::MAX, // unseen track: last resort
                };
                if best.map_or(true, |(bd, _, _)| d < bd) {
                    best = Some((d, c, t));
                }
            }
        }
        if let Some((_, c, t)) = best {
            assignment[c] = t;
            car_used[c] = true;
            track_used[t] = true;
        }
    }

    assignment
}
