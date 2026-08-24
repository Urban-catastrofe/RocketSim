//! Landing-touchdown measurement (`RLTOUCH=1`), and the world-contact field
//! survey that produced it (`RLTOUCH=2`).
//!
//! This pass started out as a chassis-scrape study, aimed at the `body_scrape`
//! census bucket. That bucket turned out not to exist. `RLTOUCH=2` surveys the
//! field it was built on and finds `phys.has_world_contact` true on **zero** of
//! 193,339 car-ticks with no wheel in contact, so the bucket condition
//! `nw_from == 0 && (from.has_world_contact || to.has_world_contact)` could only
//! ever fire through its `to` term. And the flag lags contact: it is true on
//! 93.6% of steady wheel-contact ticks but only **2.9%** of touchdown ticks
//! (40 of 1383). So `body_scrape` was a 3% sample of landings, selected by that
//! lag, wearing the name of a mechanism this dataset never records. It is gone;
//! its steps belong to the `touchdown` bucket that replaced it.
//!
//! The survey also retires two more observer fields: across 254,252 logged
//! contacts `world_contact_point` is the origin every single time and
//! `world_contact_normal` is exactly `+z` every single time, including the
//! suite's thousands of wall and ceiling steps. Infer nothing from any of the
//! three.
//!
//! This is a statement about those three fields, not about the observer. The
//! **`WheelRecord`** contact normals are a different field and they are live -
//! 15.9% of contact ticks are off the flat floor and `min_contact_z < 0.9` holds
//! on 37,421 of them - so the `wall_ceiling` bucket, which is defined on those,
//! does **not** share the defect.
//!
//! So this pass measures what those steps actually are. On the touchdown tick
//! the vertical axis is closed in the same sense as [`super::suspdump`], but
//! from the other side: at the *start* of the step no wheel is in contact, so
//! there is no suspension force, no sticky force and no wheel friction, and the
//! car body has zero linear damping. Any vertical impulse beyond gravity is the
//! contact response, and it is recoverable from the recording alone:
//!
//! ```text
//! dv_contact = (vel[i+1].z - vel[i].z) + g * dt - boost_z
//! ```
//!
//! The point of the pass is the *phase* question. Rocket League logs at the end
//! of a tick, so the forces applied over the step `i -> i+1` are the ones a
//! solver would derive from the tick-`i` pose - which is the pose
//! `set_state_to_record_tick` hands the sim. Both ends of the step therefore
//! carry a pose-derived wheel ray length, and `n_pred` counts how many wheels
//! that pose puts within [`MAX_TRAVEL`]. If RL arrests the fall on steps where
//! only the tick-`i+1` pose has a wheel in range, RL is reacting to geometry
//! the sim has not reached yet, and the fix is structural rather than a
//! coefficient.
//!
//! It does. On 476 flat-floor near-upright touchdown steps the sim's post-step
//! wheel count equals what the tick-`i` pose predicts on 473 of 473 rows, so the
//! instrument reproduces the sim exactly - and on the 291 rows where that pose
//! puts no wheel in range, RL has already applied an impulse on 90% of them
//! while the sim found nothing on all 291. But the phase is *not* simply one
//! tick out. `TOUCHSUSP` carries the suspension basis at both ends so
//! `update_suspension` can be evaluated at either, and the end-pose model
//! overshoots RL by 2.5x (sum |model - RL| of 7454 against 3027 for the current
//! phase). RL's arrival impulse is a fraction of a full tick of suspension
//! force, which is sub-tick contact onset, not a whole-tick force moved in time.
//! Details and the numbers are in `how-to-test.md`.
//!
//! ```text
//! TOUCHROW <recording> <tick> <car> <flags> <up_z> <vz_from>
//!          <ray_from> <ray_to> <n_pred_from> <n_pred_to> <nw_to>
//!          <susp_len_from> <rl_dv_z> <sim_dv_z> <sim_nw>
//! ```
//!
//! `ray_*` is the shortest of the four pose-derived ray lengths, in uu, so
//! `ray <= MAX_TRAVEL` means at least one wheel is in range and a value above
//! it is the gap still to close. `susp_len_from` is RL own logged shortest
//! `susp_length` at the tick-`i` pose, which makes the pose-derived geometry
//! checkable against ground truth rather than merely assumed. `sim_nw` is how
//! many wheels the sim had in contact after the step.
//!
//! `flags` is a fixed-width string of `-` and letters: `b` boosting, `t`
//! throttle non-zero, `B` ball within [`CLEARANCE`], `C` another car within it,
//! `r` the car is not upright, `w` some contacting wheel at the far end is not
//! on the flat floor. Steps with a jump, a flip or a jump press are dropped
//! outright, since those add an impulse of their own on exactly this tick.

use glam::Vec3A;
use rocketsim::CarControls;

use super::flat_floor::{
    BOOST_GROUND_DV, CLEARANCE, FLAT_NORMAL_Z, MAX_TRAVEL, UPRIGHT_UP_Z, WHEEL_GEO, hard_point,
    isolated, rot_of, trace,
};
use super::recording::Recording;
use super::recording::cpp_records::{CarRecord, PhysRecord};
use super::recording::tick_record::TickRecord;
use super::runner::{has_discontinuity, is_car_sentinel, make_arena, set_state_to_record_tick};

/// Gravity as a velocity change per tick, uu/s. No sticky force term here: the
/// sticky force needs a wheel already in contact, and by construction none is.
const GRAV_DV: f32 = 650.0 / 120.0;

fn wheels_in_contact(car: &CarRecord) -> usize {
    car.wheels.iter().filter(|w| w.has_contact).count()
}

/// Shortest pose-derived wheel ray length and how many of the four wheels that
/// pose puts within [`MAX_TRAVEL`] of the floor plane `z = 0`.
///
/// Same construction as [`super::flat_floor::wheel_state`]: the ray leaves the
/// hardpoint along `-up` and meets the plane at [`trace`]. Derived from the pose
/// rather than read from the log because the logged block tick alignment is not
/// resolvable from the recordings, and this pass is *about* tick alignment.
fn ray_state(phys: &PhysRecord) -> (f32, usize) {
    let rot = rot_of(phys);
    let up = rot.z_axis;
    let mut shortest = f32::MAX;
    let mut in_range = 0;
    for (w, &(_, _, _, radius, rest1)) in WHEEL_GEO.iter().enumerate() {
        let length = trace(hard_point(phys, &rot, w), up) - radius - rest1;
        shortest = shortest.min(length);
        if length <= MAX_TRAVEL {
            in_range += 1;
        }
    }
    (shortest, in_range)
}

/// Per-wheel `(compression_uu, rate_uu_per_s)` at one pose, as
/// `WheelInfo::apply_ray_cast` derives them on a flat floor.
///
/// Compression is `rest1 + radius - trace`, clamped to the same
/// `+/- MAX_SUSPENSION_TRAVEL` band the sim clamps `suspension_length` to, and
/// is negative while the wheel is still short of the ground. Rate is
/// `suspension_relative_vel`: the contact-point velocity along the normal,
/// divided by `contact_normal . up`, which on the flat floor is `up.z`. A wheel
/// whose compression is below `-MAX_TRAVEL` has no contact at all and is
/// reported as such by the caller through `n_pred`.
///
/// This exists so the sim's own suspension model can be evaluated at *either*
/// end of the step, outside the sim, and compared against RL's measured
/// impulse. That is the phase question stated numerically.
fn susp_basis(phys: &PhysRecord) -> [(f32, f32); 4] {
    let rot = rot_of(phys);
    let up = rot.z_axis;
    let pos = Vec3A::from(phys.pos);
    let vel = Vec3A::from(phys.lin_vel);
    let ang_vel = Vec3A::from(phys.ang_vel);

    let mut out = [(0.0f32, 0.0f32); 4];
    for (w, &(_, _, _, radius, rest1)) in WHEEL_GEO.iter().enumerate() {
        let hp = hard_point(phys, &rot, w);
        let tr = trace(hp, up);
        let comp = (rest1 + radius - tr).clamp(-MAX_TRAVEL, MAX_TRAVEL);
        let rel = hp - pos - up * tr;
        let rate = (vel + ang_vel.cross(rel)).z / up.z;
        out[w] = (comp, rate);
    }
    out
}

/// True when every wheel that reports contact reports the flat floor.
fn contacts_flat(car: &CarRecord) -> bool {
    car.wheels
        .iter()
        .filter(|w| w.has_contact)
        .all(|w| w.contact_normal.z > FLAT_NORMAL_Z)
}

fn flags_of(from: &CarRecord, to: &CarRecord, from_tick: &TickRecord, j: usize) -> String {
    let here = Vec3A::from(from.phys.pos);
    let ball = Vec3A::from(from_tick.ball_record.pos);
    let ball_near = from_tick.ball_record.pos[2] > -1000.0 && (ball - here).length() < CLEARANCE;
    [
        (from.is_boosting, 'b'),
        (from.prev_controls.throttle != 0.0, 't'),
        (ball_near, 'B'),
        (!isolated(from_tick, j, here) && !ball_near, 'C'),
        (rot_of(&from.phys).z_axis.z <= UPRIGHT_UP_Z, 'r'),
        (!contacts_flat(to), 'w'),
    ]
    .iter()
    .map(|&(set, c)| if set { c } else { '-' })
    .collect()
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

        let is_candidate = |j: usize| -> bool {
            let (from, to) = (&from_tick.car_records[j], &to_tick.car_records[j]);
            if from.is_demoed || to.is_demoed || is_car_sentinel(&to.phys) {
                return false;
            }
            // A jump, a flip or a jump press adds a vertical impulse of its own
            // on exactly this tick, which would be indistinguishable from the
            // contact response being measured.
            if from.is_jumping
                || to.is_jumping
                || from.is_flipping
                || to.is_flipping
                || from.prev_controls.jump
                || to.prev_controls.jump
            {
                return false;
            }
            wheels_in_contact(from) == 0 && wheels_in_contact(to) > 0
        };

        let candidates: Vec<bool> = (0..num_cars).map(is_candidate).collect();

        // Every tick is stepped, candidate or not. `set_state_to_record_tick`
        // restores car and ball state but not Bullet persistent contact
        // manifolds, whose cached impulses are warm-started; skipping ticks
        // leaves that cache describing a pose from hundreds of ticks ago. In
        // `suspdump` that produced a steady +49 uu/s of phantom suspension
        // force. Same reason the sharded runner needs `SHARD_WARMUP_TICKS`.
        for (j, car_record) in to_tick.car_records.iter().enumerate() {
            controls_buf[j] = car_record.prev_controls.into();
        }
        set_state_to_record_tick(
            &mut arena,
            &car_idcs,
            from_tick,
            i.checked_sub(stride).map(|i| &recording.ticks[i]),
            &controls_buf,
        );
        arena.step_tick();

        for (j, &car_idx) in car_idcs.iter().enumerate() {
            if !candidates[j] {
                continue;
            }
            let (from, to) = (&from_tick.car_records[j], &to_tick.car_records[j]);
            let from_z = from.phys.lin_vel[2];

            let mut offset = GRAV_DV;
            if from.is_boosting {
                offset -= BOOST_GROUND_DV * from.phys.rot.rows[0][2];
            }
            let rl_dv = to.phys.lin_vel[2] - from_z + offset;
            let sim_state = arena.get_car_state(car_idx);
            let sim_dv = sim_state.phys.vel.z - from_z + offset;

            let (ray_from, pred_from) = ray_state(&from.phys);
            let (ray_to, pred_to) = ray_state(&to.phys);
            let susp_from = from
                .wheels
                .iter()
                .map(|w| w.susp_length)
                .fold(f32::MAX, f32::min);

            println!(
                "TOUCHROW {} {i} {j} {} {:.5} {:.2} {:.3} {:.3} {pred_from} {pred_to} {} \
                 {:.3} {:.4} {:.4} {}",
                recording.name,
                flags_of(from, to, from_tick, j),
                rot_of(&from.phys).z_axis.z,
                from_z,
                ray_from,
                ray_to,
                wheels_in_contact(to),
                susp_from,
                rl_dv,
                sim_dv,
                sim_state.wheels_with_contact.iter().filter(|c| **c).count(),
            );

            // Companion line: the suspension basis at both ends of the step,
            // joined on (recording, tick, car).
            use std::fmt::Write as _;
            let mut row = format!("TOUCHSUSP {} {i} {j}", recording.name);
            for phys in [&from.phys, &to.phys] {
                let _ = write!(row, " {:.6}", rot_of(phys).z_axis.z);
                for (comp, rate) in susp_basis(phys) {
                    let _ = write!(row, " {comp:.4} {rate:.3}");
                }
            }
            println!("{row}");
        }
    }
}

/// World-contact field survey (`RLTOUCH=2`).
///
/// Counts, per recording, how `phys.has_world_contact` relates to wheel contact
/// and what the two companion fields ever hold. This needs no simulation - it
/// reads the recording and counts:
///
/// ```text
/// SURVEY <recording> ticks=<n> hwc=<n> nw0=<n> nw0_hwc=<n> nw4_hwc=<n>
///        nrm_off_flat=<n> pt_nonzero=<n> touch=<n> touch_hwc=<n>
///        nw_pos=<n> nw_pos_hwc=<n> wheel_nrm_off_flat=<n>
///        wheel_below_floor_thresh=<n> wheel_nrm_negative=<n>
/// ```
///
/// `nw0` is car-ticks with no wheel in contact, `nrm_off_flat` logged
/// `phys.world_contact_normal`s that are not exactly `+z`, `pt_nonzero` logged
/// `phys.world_contact_point`s away from the origin. `touch` counts touchdown
/// steps and `touch_hwc` how many of them the old `body_scrape` bucket could
/// even see; `nw_pos_hwc / nw_pos` is how faithfully the flag tracks wheel
/// contact in the steady state.
///
/// The `wheel_*` counters are the control, and they matter: the *`WheelRecord`*
/// contact normals are a different field from the `PhysRecord` one, and they are
/// **live**. `wheel_below_floor_thresh` reproduces `census::min_contact_z`
/// exactly, so it confirms the `wall_ceiling` bucket rests on real data rather
/// than sharing `body_scrape`'s defect. `RLTOUCH_DUMP=1` prints the raw per-wheel
/// normals for a spot check. The suite results are in the module docs above.
///
/// Keep this one `println!` on one physical line. An earlier version wrapped it
/// and the aggregation, which greps `^SURVEY`, silently read the wrapped
/// counters as absent - which looks identical to their being zero.
pub fn survey(recording: &Recording) {
    let num_cars = recording.info.num_cars as usize;
    if num_cars == 0 {
        return;
    }
    let stride = recording.stride;
    let (mut ticks, mut hwc, mut nw0, mut nw0_hwc, mut nw4_hwc) = (0u32, 0u32, 0u32, 0u32, 0u32);
    let (mut nrm_off_flat, mut pt_nonzero, mut wheel_nrm_off_flat) = (0u32, 0u32, 0u32);
    let (mut wheel_below_floor_thresh, mut wheel_nrm_negative) = (0u32, 0u32);
    let (mut touch, mut touch_hwc, mut nw_pos, mut nw_pos_hwc) = (0u32, 0u32, 0u32, 0u32);

    let last = recording.ticks.len().saturating_sub(stride + 1);
    for i in (0..=last).step_by(stride) {
        if has_discontinuity(recording, i, stride) {
            continue;
        }
        let to_tick = &recording.ticks[i + stride];
        for (j, from) in recording.ticks[i].car_records[..num_cars]
            .iter()
            .enumerate()
        {
            if from.is_demoed || is_car_sentinel(&from.phys) {
                continue;
            }
            ticks += 1;
            // How often is a touchdown step - the `body_scrape` census bucket's
            // only possible entry - actually visible to that bucket? It needs
            // `to.has_world_contact`, and that flag is what is under test.
            let to = &to_tick.car_records[j];
            if wheels_in_contact(from) == 0 && wheels_in_contact(to) > 0 {
                touch += 1;
                if to.phys.has_world_contact {
                    touch_hwc += 1;
                }
            }
            if wheels_in_contact(from) > 0 {
                nw_pos += 1;
                if from.phys.has_world_contact {
                    nw_pos_hwc += 1;
                }
                // The *wheel* contact normals are a separate field, and
                // `wall_ceiling` is defined on them. Count how often they leave
                // the flat floor: if this were also zero, that bucket would be a
                // phantom too. It is not - see the module docs.
                if from
                    .wheels
                    .iter()
                    .any(|w| w.has_contact && w.contact_normal.z.abs() < 0.999)
                {
                    wheel_nrm_off_flat += 1;
                }
                // Exactly what `census::min_contact_z` computes, so the two
                // cannot disagree about what the field holds.
                let mcz = from
                    .wheels
                    .iter()
                    .filter(|w| w.has_contact)
                    .map(|w| w.contact_normal.z.abs())
                    .fold(1.0f32, f32::min);
                if mcz < 0.9 {
                    wheel_below_floor_thresh += 1;
                }
                if std::env::var("RLTOUCH_DUMP").is_ok() {
                    let n: Vec<String> = from
                        .wheels
                        .iter()
                        .map(|w| {
                            format!(
                                "{}({:.3},{:.3},{:.3})",
                                u8::from(w.has_contact),
                                w.contact_normal.x,
                                w.contact_normal.y,
                                w.contact_normal.z
                            )
                        })
                        .collect();
                    println!(
                        "NRM {} {i} {j} mcz={mcz:.4} {}",
                        recording.name,
                        n.join(" ")
                    );
                }
                if from
                    .wheels
                    .iter()
                    .any(|w| w.has_contact && w.contact_normal.z < 0.0)
                {
                    wheel_nrm_negative += 1;
                }
            }
            let nw = wheels_in_contact(from);
            if nw == 0 {
                nw0 += 1;
            }
            if !from.phys.has_world_contact {
                continue;
            }
            hwc += 1;
            match nw {
                0 => nw0_hwc += 1,
                4 => nw4_hwc += 1,
                _ => {}
            }
            if Vec3A::from(from.phys.world_contact_normal) != Vec3A::Z {
                nrm_off_flat += 1;
            }
            if Vec3A::from(from.phys.world_contact_point) != Vec3A::ZERO {
                pt_nonzero += 1;
            }
        }
    }
    println!(
        "SURVEY {} ticks={ticks} hwc={hwc} nw0={nw0} nw0_hwc={nw0_hwc} nw4_hwc={nw4_hwc} nrm_off_flat={nrm_off_flat} pt_nonzero={pt_nonzero} touch={touch} touch_hwc={touch_hwc} nw_pos={nw_pos} nw_pos_hwc={nw_pos_hwc} wheel_nrm_off_flat={wheel_nrm_off_flat} wheel_below_floor_thresh={wheel_below_floor_thresh} wheel_nrm_negative={wheel_nrm_negative}",
        recording.name,
    );
}
