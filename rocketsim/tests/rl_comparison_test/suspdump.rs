//! Suspension force measurement (`RLSUSPDUMP=1`).
//!
//! On a flat floor with an upright car and its wheels down, the *vertical*
//! equation of motion is closed and scalar. The only vertical impulses are
//! gravity, the sticky force, boost (through `forward.z`) and the suspension,
//! because the wheel friction impulse lies in the contact plane and so has no
//! vertical component. That makes Rocket League's own suspension impulse for a
//! step recoverable from the recording alone, with no simulation involved:
//!
//! ```text
//! dv_susp = (vel[i+1].z - vel[i].z) + 1.5 * g * dt - boost_z
//! ```
//!
//! (1.5 g because the sticky force adds half a gravity on the floor.) The same
//! expression applied to the sim post-step velocity gives the sim suspension
//! impulse, so the two are directly comparable - and unlike the `RLCENSUS`
//! banding, this is an *absolute* measurement of RL suspension force rather
//! than a "required multiplier" read off a probe sweep. Rebuilding
//! `SUSPENSION_STIFFNESS` / `WHEELS_DAMPING_*` from it needs no probe at all.
//!
//! The step gate and the pose-derived wheel geometry live in [`super::flat_floor`],
//! shared with [`super::latdump`], which closes the lateral axis the same way.
//!
//! Each row is one usable step:
//!
//! ```text
//! SUSPROW <recording> <tick> <car> (<compression_uu> <rate_uu_per_s>) x4 <rl_dv> <sim_dv>
//! ```
//!
//! Compression is positive when the suspension is compressed; rate is positive
//! when it is extending. Rows are per recording, so aggregating across the suite
//! is an external step, as it is for `RLCENSUS`.
//!
//! To rebuild a coefficient from the rows, regress `rl_dv` on the sim own
//! basis, which follows `WheelInfo::update_suspension` term for term (uu/s of
//! velocity change, `m` = 180, `scale` = `SUSPENSION_FORCE_SCALE_FRONT`/`_BACK`):
//!
//! ```text
//! spring_w = SUSPENSION_STIFFNESS / (120 * m) * compression_w * scale_w
//! damp_w   = -D_branch / (120 * m) * rate_w * scale_w      // branch on sign(rate)
//! push_w   = extra_pushback_w / m                           // only if compression > 2.5
//! ```
//!
//! summed over wheels, with each wheel `spring_w + damp_w` clamped at zero
//! (which drops that wheel pushback too, as the sim does). Fit it *trimmed*:
//! the residual rms sits two orders of magnitude above the median residual, so
//! plain least squares reports the handful of landing impacts rather than the
//! bulk. Results and conclusions are in `how-to-test.md`.

use glam::Vec3A;
use rocketsim::CarControls;

use super::flat_floor::{BOOST_GROUND_DV, grounded, isolated, wheel_state};
use super::recording::Recording;
use super::runner::{has_discontinuity, make_arena, set_state_to_record_tick};

/// Gravity plus the floor sticky force, as a velocity change per tick (uu/s).
const GRAV_STICKY_DV: f32 = 1.5 * 650.0 / 120.0;

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

        let candidate = |j: usize| -> Option<[(f32, f32); 4]> {
            let (from, to) = (&from_tick.car_records[j], &to_tick.car_records[j]);
            if !grounded(from, to) {
                return None;
            }
            let here = Vec3A::from(from.phys.pos);
            if !isolated(from_tick, j, here) || !isolated(to_tick, j, here) {
                return None;
            }
            wheel_state(from, to)
        };

        let states: Vec<Option<[(f32, f32); 4]>> = (0..num_cars).map(candidate).collect();

        // Every tick is stepped, even one with no usable car. `set_state_to_record_tick`
        // restores car and ball state but not Bullet persistent contact manifolds,
        // whose cached impulses are warm-started at `WARMSTARTING_FACTOR`; skipping
        // ticks leaves that cache describing a pose from hundreds of ticks ago and it
        // then fires into an unrelated state. An earlier version of this pass skipped
        // non-candidate ticks and reported a steady +49 uu/s of phantom suspension
        // force on cars alone in an empty half of the field. This is the same reason
        // the sharded runner needs `SHARD_WARMUP_TICKS`.
        for (j, car_record) in to_tick.car_records.iter().enumerate() {
            controls_buf[j] = car_record.prev_controls.into();
        }
        set_state_to_record_tick(&mut arena, &car_idcs, from_tick, &controls_buf);
        arena.step_tick();

        for (j, &car_idx) in car_idcs.iter().enumerate() {
            let Some(state) = states[j] else { continue };
            let (from, to) = (&from_tick.car_records[j], &to_tick.car_records[j]);
            let from_z = from.phys.lin_vel[2];
            let mut offset = GRAV_STICKY_DV;
            if from.is_boosting {
                offset -= BOOST_GROUND_DV * from.phys.rot.rows[0][2];
            }
            let rl_dv = to.phys.lin_vel[2] - from_z + offset;
            let sim_dv = arena.get_car_state(car_idx).phys.vel.z - from_z + offset;
            // One `println!` per row: the harness runs recordings on several
            // threads, and a partially written line lets another thread output
            // splice into the middle of this one.
            use std::fmt::Write as _;
            let mut row = format!("SUSPROW {} {i} {j}", recording.name);
            for (c, rate) in state {
                let _ = write!(row, " {c:.5} {rate:.4}");
            }
            let _ = write!(row, " {rl_dv:.5} {sim_dv:.5}");
            println!("{row}");
        }
    }
}
