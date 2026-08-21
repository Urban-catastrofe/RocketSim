//! Sustained car-car contact measurement (`RLCARC=1`).
//!
//! `car_proximity` is the largest bucket in the census at 27.1% of the suite's
//! measurable error mass, and `RLCENSUS=8` shows where inside it that mass sits:
//! banding the bucket by the *actual* hitbox gap rather than the 240 UU centre
//! net puts 82.9% of it on steps where the two Octane hitboxes interpenetrate
//! (n = 4036, mean 96.9 UU/s). Every non-touching band runs a mean of 3.4 to
//! 10.1, which is ordinary driving error. So the bucket really is car-car
//! contact and not "cars near each other are also cars driving hard".
//!
//! It is also *not* the bump. `detect_bump_onsets` diverts every step whose
//! recorded position/velocity identity breaks -- i.e. every sub-frame impulse --
//! into the `impulse` bucket, which the census total excludes. What is left in
//! `gap:touch` is contact that Rocket League itself integrated cleanly across
//! the whole frame: sustained overlap being pushed apart, not a discrete hit.
//!
//! # The closed channel
//!
//! Two cars in contact give a closed axis for free, and a better one than any
//! single-car channel: **gravity cancels in their relative velocity.** Both
//! bodies have the same mass and take the same `GRAVITY_Z`, so
//! `d/dt (vel_b - vel_a)` has no gravity term at all. Restrict to steps where
//! both cars are airborne (no wheel in contact, so no suspension and no wheel
//! friction), neither is boosting, neither is mid-jump or mid-flip, and the ball
//! is not touching either hitbox, and the only force left acting on the pair is
//! the contact between them. Then
//!
//! ```text
//! delta_vrel_n = ((vel_b' - vel_a') - (vel_b - vel_a)) . n
//! ```
//!
//! is Rocket League's own contact impulse along the contact normal, read
//! straight off the recording with no simulation involved. Air control torques
//! do not enter -- they change `ang_vel`, not `lin_vel`.
//!
//! `n` is the minimum-penetration axis from the 15-axis SAT test in
//! `census::hitbox_separation`, oriented from car `a` to car `b`, so a
//! separating impulse is positive.
//!
//! # Stepping
//!
//! Every tick is stepped whether it is a candidate or not.
//! `set_state_to_record_tick` restores car and ball state but not Bullet's
//! persistent contact manifolds, which are warm-started from the previous step;
//! skipping ticks leaves the solver warm-started on unrelated geometry. Ignoring
//! this in the suspension study once produced a phantom +49 UU/s of force.

use glam::{Mat3A, Vec3A};
use rocketsim::CarControls;

use super::census::{ball_touches_car, hitbox_separation, is_sentinel};
use super::recording::Recording;
use super::recording::cpp_records::{CarRecord, PhysRecord};
use super::runner::{has_discontinuity, is_car_sentinel, make_arena, set_state_to_record_tick};

fn rot_of(phys: &PhysRecord) -> Mat3A {
    Mat3A::from_cols(
        phys.rot.rows[0].into(),
        phys.rot.rows[1].into(),
        phys.rot.rows[2].into(),
    )
}

fn wheels_in_contact(car: &CarRecord) -> usize {
    car.wheels.iter().filter(|w| w.has_contact).count()
}

/// Is this car's velocity driven by anything other than gravity and the car-car
/// contact? Any `true` here breaks the closed channel.
fn contaminated(car: &CarRecord) -> bool {
    wheels_in_contact(car) > 0
        || car.is_boosting
        || car.is_jumping
        || car.is_flipping
        || car.is_demoed
        || car.prev_controls.boost
        || car.prev_controls.jump
}

/// One-letter flags, so a row says at a glance why it looks the way it does.
fn flags_of(a: &CarRecord, b: &CarRecord) -> String {
    let mut s = String::new();
    if a.is_supersonic || b.is_supersonic {
        s.push('S');
    }
    if a.handbrake_val > 0.0 || b.handbrake_val > 0.0 {
        s.push('h');
    }
    if a.has_jumped || b.has_jumped {
        s.push('j');
    }
    if s.is_empty() {
        s.push('-');
    }
    s
}

pub fn analyze(recording: &Recording) {
    let num_cars = recording.info.num_cars as usize;
    if num_cars < 2 {
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

        for a in 0..num_cars {
            for b in (a + 1)..num_cars {
                let (fa, fb) = (&from_tick.car_records[a], &from_tick.car_records[b]);
                let (ta, tb) = (&to_tick.car_records[a], &to_tick.car_records[b]);
                if [&fa.phys, &fb.phys, &ta.phys, &tb.phys]
                    .iter()
                    .any(|p| is_sentinel(p) || is_car_sentinel(p))
                {
                    continue;
                }
                // Either endpoint overlapping counts: the contact is resolved
                // across the step, so the pose that shows it may be either one.
                let (sep_from, axis_from) = hitbox_separation(&fa.phys, &fb.phys);
                let (sep_to, axis_to) = hitbox_separation(&ta.phys, &tb.phys);
                if sep_from > 0.0 && sep_to > 0.0 {
                    continue;
                }
                // A sub-frame impulse on either car makes the step's split
                // arbitrary; those steps are the `impulse` bucket's business.
                if recording.step_straddles_impulse(i, stride, a)
                    || recording.step_straddles_impulse(i, stride, b)
                {
                    continue;
                }
                let ball_from = &from_tick.ball_record;
                let ball_to = &to_tick.ball_record;
                let ball_near = [
                    (ball_from, &fa.phys),
                    (ball_from, &fb.phys),
                    (ball_to, &ta.phys),
                    (ball_to, &tb.phys),
                ]
                .iter()
                .any(|(ball, car)| {
                    !is_sentinel(ball) && ball_touches_car(Vec3A::from(ball.pos), car)
                });

                let clean = !ball_near
                    && !contaminated(fa)
                    && !contaminated(fb)
                    && !contaminated(ta)
                    && !contaminated(tb);

                // Use the `from` pose's axis when it already overlaps, since
                // that is the geometry the step's impulse was computed from.
                let (pen, axis) = if sep_from <= 0.0 {
                    (-sep_from, axis_from)
                } else {
                    (-sep_to, axis_to)
                };

                let rl_rel_from = Vec3A::from(fb.phys.lin_vel) - Vec3A::from(fa.phys.lin_vel);
                let rl_rel_to = Vec3A::from(tb.phys.lin_vel) - Vec3A::from(ta.phys.lin_vel);
                let rl_dv = (rl_rel_to - rl_rel_from).dot(axis);

                let sim_a = arena.get_car_state(car_idcs[a]).phys.vel;
                let sim_b = arena.get_car_state(car_idcs[b]).phys.vel;
                let sim_dv = ((sim_b - sim_a) - rl_rel_from).dot(axis);

                // Per-car residual along the same axis, to say whether the sim
                // splits the impulse between the pair the way RL does.
                let resid_a = (sim_a - Vec3A::from(ta.phys.lin_vel)).dot(axis);
                let resid_b = (sim_b - Vec3A::from(tb.phys.lin_vel)).dot(axis);

                // Closing speed along the normal at the start of the step: how
                // hard the pair was driven together before anything resolved it.
                let closing = -rl_rel_from.dot(axis);
                let up_a = rot_of(&fa.phys).z_axis.z;
                let up_b = rot_of(&fb.phys).z_axis.z;

                println!(
                    "[{}] CARC {} {} {} {} d={:.2} spa={:.1} spb={:.1} lagab={:.2} lagba={:.2} pen={:.3} closing={:.2} rl_dv={:.3} sim_dv={:.3} resid_a={:.3} resid_b={:.3} nwa={} nwb={} upa={:.3} upb={:.3} clean={} nrm=({:.3},{:.3},{:.3})",
                    recording.name,
                    i,
                    a,
                    b,
                    flags_of(fa, fb),
                    (Vec3A::from(fb.phys.pos) - Vec3A::from(fa.phys.pos)).length(),
                    Vec3A::from(fa.phys.lin_vel).length(),
                    Vec3A::from(fb.phys.lin_vel).length(),
                    // Aliasing test: is car `b` at tick `i` simply car `a` one
                    // tick later (or the reverse)? A duplicate record of the
                    // same physical car shows near-zero relative velocity and a
                    // centre offset of exactly one tick of travel.
                    (Vec3A::from(fb.phys.pos) - Vec3A::from(ta.phys.pos)).length(),
                    (Vec3A::from(fa.phys.pos) - Vec3A::from(tb.phys.pos)).length(),
                    pen,
                    closing,
                    rl_dv,
                    sim_dv,
                    resid_a,
                    resid_b,
                    wheels_in_contact(fa),
                    wheels_in_contact(fb),
                    up_a,
                    up_b,
                    u8::from(clean),
                    axis.x,
                    axis.y,
                    axis.z,
                );
            }
        }
    }
}

/// Survey intra-tick `physics_frame` agreement (`RLCARC=2`).
///
/// `RLCARC=1` turned up 1736 of 2255 overlapping-hitbox pairs at centre
/// separations below 38.66 UU -- the smallest Octane hitbox dimension, and so
/// closer than two hitboxes can be in *any* orientation. On 91% of them the
/// separation equals exactly one tick of travel (`|d - speed * dt|` p50 = 0.005
/// UU, float noise) along the car's own velocity, and the pairing is structured:
/// in `2v2_3` the pairs `[0,2]` and `[1,3]` each account for 174 steps.
///
/// The raw records say what that is. At tick 356 of `2v2_1` the four car slots
/// carry `physics_frame` 357, 358, 358, 357 -- the cars are on *different
/// physics frames within one recorded tick*. Slot 1 holds slot 0's car advanced
/// by one frame (identical `fwd` and `up` to three decimals, velocity within 6
/// UU/s, position offset exactly one frame of travel along its own velocity),
/// and the car that really was in slot 1 the tick before, 3200 UU away, has
/// vanished from the array. The same happens to slots 2 and 3. Four slots, two
/// real cars, each duplicated one frame apart.
///
/// `car_order::reorder_cars` cannot repair this and is not causing it: its
/// matching is bijective, so when the array holds one car twice it is forced to
/// file the copies under two different canonical slots. It has no car to put in
/// the vanished one.
///
/// This is the same logger defect `detect_bump_onsets` already documents from
/// the other side -- "`2v2_1` cars 1 and 3 hold `|dpos|` = 38.4 UU for 18
/// consecutive ticks at exactly 2300 UU/s, precisely `2 * vel * dt`" -- caught
/// there per car via the `physics_frame` advance, and caught here as
/// disagreement *between* cars in one tick.
///
/// A uniform `physics_frame` across a tick's live cars is the direct test, so
/// this reports that rather than any geometric proxy.
pub fn alias_survey(recording: &Recording) {
    let num_cars = recording.info.num_cars as usize;
    if num_cars < 2 {
        return;
    }
    let n_ticks = recording.ticks.len();
    let mut desync = 0usize;
    let mut measured = 0usize;
    let mut worst_spread = 0u32;
    // How many distinct frames a desynced tick is spread over, so a one-frame
    // stagger is not confused with a wholesale collapse.
    let mut spread_hist: std::collections::BTreeMap<u32, usize> = Default::default();

    for tick in &recording.ticks {
        if tick.car_records.len() != num_cars {
            continue;
        }
        let frames: Vec<u32> = tick
            .car_records
            .iter()
            .filter(|c| !c.is_demoed && !is_sentinel(&c.phys) && !is_car_sentinel(&c.phys))
            .map(|c| c.phys.physics_frame)
            .collect();
        if frames.len() < 2 {
            continue;
        }
        measured += 1;
        let (lo, hi) = (*frames.iter().min().unwrap(), *frames.iter().max().unwrap());
        let spread = hi - lo;
        if spread > 0 {
            desync += 1;
            worst_spread = worst_spread.max(spread);
            *spread_hist.entry(spread).or_default() += 1;
        }
    }

    // The stagger only fabricates error for the car being measured if it
    // *changes* across the step: then that car's own `from -> to` is not one
    // physics frame long, and the harness grades a two-frame (or zero-frame)
    // ground-truth delta against a one-frame simulation.
    let stride = recording.stride;
    let mut advance: std::collections::BTreeMap<i64, usize> = Default::default();
    for t in 0..n_ticks.saturating_sub(stride) {
        let (from, to) = (&recording.ticks[t], &recording.ticks[t + stride]);
        if from.car_records.len() != num_cars || to.car_records.len() != num_cars {
            continue;
        }
        for c in 0..num_cars {
            let (a, b) = (&from.car_records[c], &to.car_records[c]);
            if a.is_demoed || b.is_demoed {
                continue;
            }
            if is_sentinel(&a.phys) || is_car_sentinel(&a.phys) {
                continue;
            }
            if is_sentinel(&b.phys) || is_car_sentinel(&b.phys) {
                continue;
            }
            let d = b.phys.physics_frame as i64 - a.phys.physics_frame as i64;
            *advance.entry(d).or_default() += 1;
        }
    }
    let adv_total: usize = advance.values().sum();
    let adv_ok = advance.get(&(stride as i64)).copied().unwrap_or(0);
    let adv: Vec<String> = advance.iter().map(|(k, v)| format!("{k}:{v}")).collect();

    let hist: Vec<String> = spread_hist
        .iter()
        .map(|(k, v)| format!("{k}:{v}"))
        .collect();
    println!(
        "[{}] CARCSYNC cars={} ticks={} measured={} desync={} frac={:.4} worst_spread={} spread=[{}]",
        recording.name,
        num_cars,
        n_ticks,
        measured,
        desync,
        desync as f32 / measured.max(1) as f32,
        worst_spread,
        hist.join(","),
    );
    println!(
        "[{}] CARCADV steps={} unit={} frac_bad={:.4} adv=[{}]",
        recording.name,
        adv_total,
        adv_ok,
        1.0 - adv_ok as f32 / adv_total.max(1) as f32,
        adv.join(","),
    );
}

/// Raw per-car dump for a tick range (`RLCARC=3`, `RLCARC_REC=<name>`,
/// `RLCARC_T=<first>:<last>`). Used to settle what an anomalous pair actually
/// is, rather than inferring it from derived quantities.
pub fn raw_dump(recording: &Recording) {
    let want = std::env::var("RLCARC_REC").unwrap_or_default();
    if !want.is_empty() && recording.name != want {
        return;
    }
    let range = std::env::var("RLCARC_T").unwrap_or_default();
    let (lo, hi) = match range.split_once(':') {
        Some((a, b)) => (
            a.parse::<usize>().unwrap_or(0),
            b.parse::<usize>().unwrap_or(0),
        ),
        None => (0, 0),
    };
    for t in lo..=hi.min(recording.ticks.len().saturating_sub(1)) {
        for (j, c) in recording.ticks[t].car_records.iter().enumerate() {
            let p = &c.phys;
            println!(
                "[{}] RAW t{} car{} pos=({:.2},{:.2},{:.2}) vel=({:.1},{:.1},{:.1}) fwd=({:.3},{:.3},{:.3}) up=({:.3},{:.3},{:.3}) nw={} onground={} demoed={} frame={} team_hint={}",
                recording.name,
                t,
                j,
                p.pos.x,
                p.pos.y,
                p.pos.z,
                p.lin_vel.x,
                p.lin_vel.y,
                p.lin_vel.z,
                p.rot.rows[0].x,
                p.rot.rows[0].y,
                p.rot.rows[0].z,
                p.rot.rows[2].x,
                p.rot.rows[2].y,
                p.rot.rows[2].z,
                wheels_in_contact(c),
                u8::from(c.is_on_ground),
                u8::from(c.is_demoed),
                p.physics_frame,
                j % 2,
            );
        }
    }
}
