//! Flip z-damping measurement (`RLFLIP=1`).
//!
//! `RLCENSUS=10` bands every bucket by the recorded jump/flip state and turns up
//! one band carrying **6.6% of the whole suite's error mass** on its own:
//! `air_boost/djf/ft.15-.4` at n = 6157, mean 28.04, p50 0.56, p90 146.5, p99
//! 266, with a signed bias of -13.2 forward and -15.7 up. `air_free` over the
//! same window adds another 3.7%. The neighbouring flip-time bands
//! (`ft0-.15`, `ft.4-.8`, `ft.8+`) all sit between 1.2 and 2.5, so it is that
//! window specifically, and the shape is bimodal -- most steps are exact and
//! about an eighth are wrong by 150 to 270 uu/s.
//!
//! The window is not arbitrary: `flip::Z_DAMP_START` is 0.15 and
//! `flip::Z_DAMP_END` is 0.21. Inside `update_double_jump_or_flip` the sim does
//!
//! ```text
//! if is_flipping && flip_time <= TORQUE_TIME (0.65)
//!                && flip_time >= Z_DAMP_START (0.15)
//!                && (lin_vel.z < 0.0 || flip_time < Z_DAMP_END (0.21))
//! {
//!     lin_vel.z *= 1.0 - Z_DAMP_120;   // *= 0.65
//! }
//! ```
//!
//! a 35% cut of vertical velocity *per tick*, unconditional below 0.21 and
//! whenever falling from 0.21 up to 0.65. On a car climbing at 1000 uu/s that is
//! a 350 uu/s change in one tick, which is the scale the p90 and p99 show.
//!
//! Whether it fires hinges entirely on `is_flipping`, and the recording does not
//! carry that as a sustained flag -- the observer emits a one-tick pulse on the
//! activation tick. So `runner::set_state_to_record_tick` *reconstructs* it, from
//! a heuristic on angular velocity and the up-axis:
//!
//! ```text
//! hard_tumble = |ang_vel.xy| > 2.0
//! active      = flip_time <= Z_DAMP_END
//! ... and only re-opens is_flipping when
//!     Z_DAMP_START..=Z_DAMP_END contains flip_time && 0.0 < up.z < 0.9
//! ```
//!
//! Two consequences worth measuring rather than assuming. The reconstruction can
//! only ever re-open the flag *inside* [0.15, 0.21], so for `flip_time` in
//! (0.21, 0.65] the sim never damps -- even though the sim's own rule says it
//! should whenever the car is falling. And inside the window the decision rests
//! on `up.z < 0.9` and a tumble threshold, neither of which is what the game
//! keys off.
//!
//! # The closed channel
//!
//! An airborne car with no wheel contact, no ball and no other car nearby has a
//! fully determined vertical velocity. Ordering inside `pre_tick_update` is
//! z-damp (in `update_double_jump_or_flip`), then `update_boost`, then Bullet
//! integrates gravity. So
//!
//! ```text
//! no damp:  vz_to == vz_from            + boost_z - g*dt
//! damped:   vz_to == vz_from * 0.65     + boost_z - g*dt
//! ```
//!
//! and the two differ by `0.35 * vz_from`. Picking whichever the recording
//! matches reads Rocket League's own damp decision off the data with no
//! simulation involved. Steps with no boost are the cleanest, since `boost_z`
//! drops out and nothing is assumed about where RL applies boost relative to the
//! damp; boosting steps are emitted too, with both orderings, so the ordering can
//! be settled rather than guessed.
//!
//! # Stepping
//!
//! Every tick is stepped whether it is a candidate or not.
//! `set_state_to_record_tick` does not restore Bullet's persistent contact
//! manifolds, which are warm-started from the previous step, so skipping ticks
//! leaves the solver warm on unrelated geometry. Ignoring that in the suspension
//! study once produced a phantom +49 uu/s of force.

use glam::Vec3A;
use rocketsim::CarControls;

use super::census::{ball_touches_car, is_sentinel};
use super::recording::Recording;
use super::recording::cpp_records::CarRecord;
use super::runner::{has_discontinuity, is_car_sentinel, make_arena, set_state_to_record_tick};

/// Gravity's contribution to vz over one tick (uu/s).
const GRAV_DV: f32 = 650.0 / 120.0;
/// Airborne boost acceleration over one tick (uu/s), along the car's forward.
const BOOST_DV: f32 = (3175.0 / 3.0) / 120.0;
/// The sim's per-tick vertical retention while damping.
const DAMP: f32 = 1.0 - 0.35;
/// Another car this close (UU, centre to centre) can be touching.
const CAR_PROXIMITY: f32 = 240.0;

fn wheels_in_contact(car: &CarRecord) -> usize {
    car.wheels.iter().filter(|w| w.has_contact).count()
}

pub fn analyze(recording: &Recording) {
    let num_cars = recording.info.num_cars as usize;
    if num_cars == 0 {
        return;
    }
    let stride = recording.stride;
    let (mut arena, car_idcs) = make_arena(num_cars);
    let mut controls_buf: Vec<CarControls> = vec![CarControls::DEFAULT; num_cars];

    let last = recording.ticks.len().saturating_sub(stride + 1);
    for i in (0..=last).step_by(stride) {
        if has_discontinuity(recording, i, stride) {
            continue;
        }
        let from_tick = &recording.ticks[i];
        let to_tick = &recording.ticks[i + stride];

        for (j, car_record) in to_tick.car_records.iter().enumerate() {
            controls_buf[j] = car_record.prev_controls.into();
        }
        set_state_to_record_tick(&mut arena, &car_idcs, from_tick, &controls_buf);
        arena.step_tick();

        for (j, &car_idx) in car_idcs.iter().enumerate() {
            let from = &from_tick.car_records[j];
            let to = &to_tick.car_records[j];
            if to.is_demoed || from.is_demoed {
                continue;
            }
            if is_sentinel(&from.phys) || is_car_sentinel(&from.phys) {
                continue;
            }
            // Airborne at both ends: no suspension, no wheel friction, no
            // sticky force, so vz has exactly three contributors.
            if wheels_in_contact(from) != 0 || wheels_in_contact(to) != 0 {
                continue;
            }
            // Only flip-interior steps can damp at all.
            if !from.double_jumped_or_flipped || from.flip_time <= 0.0 {
                continue;
            }
            // A jump impulse or a fresh press lands inside the step and its
            // split is unrecorded; those are the impulse bucket's business.
            if recording.step_straddles_impulse(i, stride, j) {
                continue;
            }
            if from.is_jumping || from.prev_controls.jump {
                continue;
            }
            let ball = &from_tick.ball_record;
            if !is_sentinel(ball) && ball_touches_car(Vec3A::from(ball.pos), &from.phys) {
                continue;
            }
            let here = Vec3A::from(from.phys.pos);
            let car_near = from_tick.car_records.iter().enumerate().any(|(k, c)| {
                k != j
                    && !c.is_demoed
                    && !is_sentinel(&c.phys)
                    && (Vec3A::from(c.phys.pos) - here).length() < CAR_PROXIMITY
            });
            if car_near {
                continue;
            }

            let vz_from = from.phys.lin_vel.z;
            let vz_to = to.phys.lin_vel.z;
            let fwd_z = from.phys.rot.rows[0].z;
            let boosting = from.is_boosting;
            let boost_z = if boosting { BOOST_DV * fwd_z } else { 0.0 };

            // Four candidates: damp or not, crossed with whether RL applies
            // boost before or after the damp. Without boost the pairs collapse.
            let pred_none = vz_from + boost_z - GRAV_DV;
            let pred_damp_then_boost = vz_from * DAMP + boost_z - GRAV_DV;
            let pred_boost_then_damp = (vz_from + boost_z) * DAMP - GRAV_DV;

            let sim_vz = arena.get_car_state(car_idx).phys.vel.z;
            let up_z = from.phys.rot.rows[2].z;
            let av = from.phys.ang_vel;
            let tumble = (av.x * av.x + av.y * av.y).sqrt();

            println!(
                "[{}] FLIPZ {} {} ft={:.4} vzF={:.2} vzT={:.2} fwdz={:.3} upz={:.3} tumble={:.3} boost={} frt={:.3} hasflip={} isflip={} rN={:.3} rD={:.3} rBD={:.3} simvz={:.2} simres={:.3}",
                recording.name,
                i,
                j,
                from.flip_time,
                vz_from,
                vz_to,
                fwd_z,
                up_z,
                tumble,
                u8::from(boosting),
                // A real flip carries a relative torque; a double jump does not.
                // `double_jumped_or_flipped` cannot tell them apart, and the
                // z-damp only belongs to the flip.
                Vec3A::new(
                    from.flip_rel_torque.x,
                    from.flip_rel_torque.y,
                    from.flip_rel_torque.z,
                )
                .length(),
                u8::from(from.has_flip),
                u8::from(from.is_flipping),
                vz_to - pred_none,
                vz_to - pred_damp_then_boost,
                vz_to - pred_boost_then_damp,
                sim_vz,
                sim_vz - vz_to,
            );
        }
    }
}
