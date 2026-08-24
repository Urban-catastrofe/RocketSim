//! Ball error census (`RLBALL`).
//!
//! The car has had an error-mass census since `census.rs`; the ball never has,
//! even though it is gated on the same per-tick bar and carries 461 of the
//! suite's 1009 gate violations. This pass files every ball step into exactly
//! one bucket named after the *cause* and reports the error mass per bucket,
//! the same method that found every car win.
//!
//! Two things make the ball a cleaner subject than the car. There are 143
//! `ball_*` recordings with no car in them at all, so the system is closed:
//! gravity, drag, and the arena mesh. And free flight is already exact
//! (`ball_very_slow` scores 0.009 uu/s, `ball_spin` 0.002), which means drag
//! and gravity are right and anything left is a *contact* error.
//!
//! - `RLBALL=1` -- census of ball velocity error by contact cause.
//! - `RLBALL=2` -- liveness survey of the ball's own world-contact fields.
//!   `has_world_contact` / `world_contact_point` / `world_contact_normal` are
//!   dead on *car* records (see `touchdown.rs` and the `RLTOUCH=2` survey), but
//!   nobody had ever checked them on the *ball* record, and a live contact
//!   normal would be Rocket League's own answer for the one quantity a bounce
//!   turns on.
//! - `RLBALL=3` -- per-bounce dump: one row per measured ball step.

use std::collections::BTreeMap;

use glam::{Mat3A, Vec3A};
use rocketsim::consts::TICK_TIME;
use rocketsim::{ArenaEvent, CarControls};

use super::census::{ball_touches_car, is_sentinel};
use super::recording::Recording;
use super::recording::cpp_records::PhysRecord;
use super::runner::{has_discontinuity, is_car_sentinel, make_arena, set_state_to_record_tick};

/// A contact normal this close to an axis counts as that flat face. Bounces off
/// the flat floor/walls/ceiling are already accurate, so the interesting classes
/// are the ones that fall through every flat test.
const FLAT_TOL: f32 = 0.02;
/// Above this velocity residual a nominally contact-free step is really a
/// contact the sim missed: free flight is otherwise exact to about 0.01 uu/s.
const MISS_THRESHOLD: f32 = 10.0;
const BALL_RADIUS: f32 = 91.25;
/// Nominal soccar arena extents, for naming which surface a missed contact was
/// near. Only used for reporting -- the real geometry is the collision mesh.
const ARENA_X: f32 = 4096.0;
const ARENA_Y: f32 = 5120.0;
const ARENA_Z: f32 = 2048.0;
/// The corner chamfer plane, `|x| + |y| = CORNER_C`.
const CORNER_C: f32 = 8064.0;
const SQRT2: f32 = std::f32::consts::SQRT_2;
/// How far the ball's recorded velocity may sit from the velocity implied by its
/// own recorded position change before the step is called self-inconsistent.
/// Generous: the channels agree to well under 1 uu/s wherever they agree at all.
const SELF_CONSISTENCY_TOL: f32 = 5.0;
/// Gravity's contribution to one tick of velocity, 650 / 120.
const GRAV_DV: f32 = 650.0 / 120.0;
/// Octane hitbox half-extents and centre offset, as in `census.rs`.
const HITBOX_HALF: Vec3A = Vec3A::new(120.507 / 2.0, 86.6994 / 2.0, 38.6591 / 2.0);
const HITBOX_OFFSET: Vec3A = Vec3A::new(13.8757, 0.0, 20.755);

#[derive(Default)]
struct Bucket {
    n: u64,
    sum: f64,
    sum_sq: f64,
    max: f32,
    max_tick: usize,
    ang_sum: f64,
    /// Signed velocity residual along the sim's contact normal, summed. A bounce
    /// that comes out too elastic biases positive along the normal.
    normal_bias: f64,
    normal_n: u64,
    /// Tail counts. Every ball bucket is bimodal -- `free` has p50 0.006 and a
    /// max of 5268 -- so a mean says almost nothing and the count of steps that
    /// are actually wrong says almost everything.
    n_over_1: u64,
    n_over_10: u64,
    n_over_100: u64,
    samples: Vec<f32>,
}

impl Bucket {
    fn add(&mut self, err: f32, ang: f32, tick: usize, along_normal: Option<f32>) {
        self.n += 1;
        self.sum += f64::from(err);
        self.sum_sq += f64::from(err) * f64::from(err);
        self.ang_sum += f64::from(ang);
        if err > self.max {
            self.max = err;
            self.max_tick = tick;
        }
        if let Some(a) = along_normal {
            self.normal_bias += f64::from(a);
            self.normal_n += 1;
        }
        if err > 1.0 {
            self.n_over_1 += 1;
        }
        if err > 10.0 {
            self.n_over_10 += 1;
        }
        if err > 100.0 {
            self.n_over_100 += 1;
        }
        self.samples.push(err);
    }

    fn pct(&mut self, q: f32) -> f32 {
        if self.samples.is_empty() {
            return 0.0;
        }
        self.samples
            .sort_unstable_by(|a, b| a.partial_cmp(b).unwrap());
        let idx = ((self.samples.len() as f32 - 1.0) * q).round() as usize;
        self.samples[idx]
    }
}

/// Name the surface a contact normal belongs to, using only the normal. Avoids
/// hard-coding arena geometry, and names exactly the distinction that matters:
/// an axis-aligned flat face versus anything else.
fn normal_class(n: Vec3A) -> &'static str {
    let (ax, ay, az) = (n.x.abs(), n.y.abs(), n.z.abs());
    if az > 1.0 - FLAT_TOL {
        return if n.z > 0.0 { "floor" } else { "ceiling" };
    }
    if ax > 1.0 - FLAT_TOL {
        return "sidewall";
    }
    if ay > 1.0 - FLAT_TOL {
        return "backwall";
    }
    // The soccar corners are a 45 degree chamfer between the side and back
    // walls: a normal with equal x and y and no z.
    if az < FLAT_TOL && (ax - ay).abs() < 0.05 {
        return "corner45";
    }
    if az < FLAT_TOL {
        return "vert_oblique";
    }
    "oblique"
}

/// `RLBALL=6`: how the sim's ball-world contact response compares with Rocket
/// League's, banded by how far the ball actually is from the surface.
///
/// The crossbar dump raised a specific suspicion. `convert_contact_special`
/// takes `distance = total_dist / num_special_collisions` where each term is
/// `rel_pos.length()` -- the distance from the ball *centre* to the contact
/// point, i.e. about one ball radius. So `penetration` is always positive, the
/// positional term is always zero, and the velocity term always runs at full
/// strength. Bullet calls that a speculative contact and it is deliberate, but
/// it means a ball still separated from a surface has its entire approach
/// velocity cancelled. At the crossbar the gap was +0.25 UU and the sim removed
/// 1847 uu/s where RL removed 241.
///
/// This banding is the test: if the over-reaction is speculative contact, the
/// sim/RL impulse ratio blows up as the gap goes positive and sits near 1 where
/// the ball genuinely overlaps.
pub fn response_study(recording: &Recording) {
    /// A car this close means the impulse is not a clean world contact.
    const CAR_CLEAR: f32 = 500.0;

    let num_cars = recording.info.num_cars as usize;
    let stride = recording.stride;
    let (mut arena, car_idcs) = make_arena(num_cars);
    let mut controls_buf: Vec<CarControls> = vec![CarControls::DEFAULT; num_cars];

    let last = recording.ticks.len().saturating_sub(stride + 1);
    for i in (0..=last).step_by(stride) {
        let discontinuous = has_discontinuity(recording, i, stride);
        let from_tick = &recording.ticks[i];
        let to_tick = &recording.ticks[i + stride];
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

        let v_before = arena.get_ball_state().phys.vel;
        let mut pts: Vec<(Vec3A, Vec3A)> = Vec::new();
        for ev in arena.step_tick() {
            if let ArenaEvent::BallHitWorld(e) = ev {
                pts.push((e.contact_point, e.contact_normal));
            }
        }
        if discontinuous || pts.is_empty() {
            continue;
        }
        let from = &from_tick.ball_record;
        let to = &to_tick.ball_record;
        if is_sentinel(from) || is_sentinel(to) {
            continue;
        }
        let p = Vec3A::from(from.pos);
        let car_near = from_tick
            .car_records
            .iter()
            .chain(to_tick.car_records.iter())
            .filter(|c| !c.is_demoed && !is_car_sentinel(&c.phys))
            .any(|c| hitbox_dist(p, &c.phys) - BALL_RADIUS < CAR_CLEAR);
        if car_near {
            continue;
        }

        // Closest approach of the ball surface to any contact point the sim
        // produced. Positive means the ball had not reached the surface.
        let gap = pts
            .iter()
            .map(|&(pt, _)| (pt - p).length() - BALL_RADIUS)
            .fold(f32::INFINITY, f32::min);
        let mean_nrm = pts
            .iter()
            .fold(Vec3A::ZERO, |a, &(_, n)| a + n)
            .normalize_or_zero();
        // Distinct normals among the points, so duplicate-driven weighting is
        // visible: the crossbar reported the same point eight times.
        let mut distinct = 0usize;
        let mut seen: Vec<Vec3A> = Vec::new();
        for &(_, n) in &pts {
            if !seen.iter().any(|&s| (s - n).length() < 1e-4) {
                seen.push(n);
                distinct += 1;
            }
        }

        // Candidate normals for the single contact `convert_contact_special`
        // builds. The current rule is the plain mean, which at the goal post
        // base averages 58 distinct normals into a direction 35 degrees off the
        // impulse RL actually applied. These are the alternatives worth testing
        // against RL's own impulse direction.
        let per_point_gap =
            |pt: Vec3A| -> f32 { (pt - p).length() - BALL_RADIUS };
        // Deepest contact: the most negative signed gap.
        let deepest = pts
            .iter()
            .min_by(|a, b| {
                per_point_gap(a.0)
                    .partial_cmp(&per_point_gap(b.0))
                    .unwrap()
            })
            .map_or(Vec3A::ZERO, |&(_, n)| n);
        // The surface the ball is driving into hardest.
        let most_opposed = pts
            .iter()
            .min_by(|a, b| {
                v_before
                    .dot(a.1)
                    .partial_cmp(&v_before.dot(b.1))
                    .unwrap()
            })
            .map_or(Vec3A::ZERO, |&(_, n)| n);
        // The mean over distinct normals, so duplicate points stop acting as
        // weights: at the crossbar one point was reported eight times.
        let mut uniq: Vec<Vec3A> = Vec::new();
        for &(_, n) in &pts {
            if !uniq.iter().any(|&u| (u - n).length() < 1e-4) {
                uniq.push(n);
            }
        }
        let mean_distinct = uniq
            .iter()
            .fold(Vec3A::ZERO, |a, &b| a + b)
            .normalize_or_zero();

        let sim_dv = arena.get_ball_state().phys.vel - v_before;
        let rl_dv = Vec3A::from(to.lin_vel) - Vec3A::from(from.lin_vel);
        // Gravity is in both, so remove it before comparing impulses.
        let g = Vec3A::new(0.0, 0.0, -GRAV_DV);
        let sim_imp = sim_dv - g;
        let rl_imp = rl_dv - g;
        let approach = v_before.dot(mean_nrm);
        println!(
            "BALLRESP {} t{i} np={} distinct={distinct} gap={gap:.2} class={} approach={approach:.1} simimp={:.1} rlimp={:.1} ratio={:.3} angle={:.1} nrm=({:.3},{:.3},{:.3}) pos=({:.0},{:.0},{:.0})",
            recording.name,
            pts.len(),
            normal_class(mean_nrm),
            sim_imp.length(),
            rl_imp.length(),
            if rl_imp.length() > 1.0 {
                sim_imp.length() / rl_imp.length()
            } else {
                f32::NAN
            },
            if sim_imp.length() > 1.0 && rl_imp.length() > 1.0 {
                sim_imp
                    .normalize()
                    .dot(rl_imp.normalize())
                    .clamp(-1.0, 1.0)
                    .acos()
                    .to_degrees()
            } else {
                f32::NAN
            },
            mean_nrm.x,
            mean_nrm.y,
            mean_nrm.z,
            p.x,
            p.y,
            p.z,
        );
        // Angle from RL's own impulse direction to each candidate normal, so the
        // selection rule can be chosen by measurement rather than by taste.
        if rl_imp.length() > 50.0 && uniq.len() > 1 {
            let ang = |n: Vec3A| -> f32 {
                if n.length() < 1e-6 {
                    return f32::NAN;
                }
                rl_imp
                    .normalize()
                    .dot(n.normalize())
                    .clamp(-1.0, 1.0)
                    .acos()
                    .to_degrees()
            };
            println!(
                "BALLNRM {} t{i} nuniq={} mean={:.1} meandistinct={:.1} deepest={:.1} opposed={:.1} rlimp={:.0}",
                recording.name,
                uniq.len(),
                ang(mean_nrm),
                ang(mean_distinct),
                ang(deepest),
                ang(most_opposed),
                rl_imp.length(),
            );
        }
    }
}

/// `RLBALL=5`: print every individual ball-world contact point the sim produces,
/// for `RLBALL_REC` over the tick range `RLBALL_T=lo:hi`.
///
/// Ball-vs-world does not go through Bullet's contact solver at all. The
/// contact-added callback marks these points `is_special`, and
/// `SeqImpulseConstraintSolver::convert_contact_special` collapses *every* point
/// into a single contact whose normal is the plain mean `total_normal /
/// num_special_collisions`. So when the ball spans several surfaces the impulse
/// is driven by an average direction that need not be any real surface's normal,
/// and this dump is what shows which surfaces went into that average.
pub fn point_dump(recording: &Recording) {
    let want = std::env::var("RLBALL_REC").unwrap_or_default();
    if !want.is_empty() && recording.name != want {
        return;
    }
    let (lo, hi) = std::env::var("RLBALL_T")
        .ok()
        .and_then(|v| {
            let (a, b) = v.split_once(':')?;
            Some((a.parse::<usize>().ok()?, b.parse::<usize>().ok()?))
        })
        .unwrap_or((0, usize::MAX));

    let num_cars = recording.info.num_cars as usize;
    let stride = recording.stride;
    let (mut arena, car_idcs) = make_arena(num_cars);
    let mut controls_buf: Vec<CarControls> = vec![CarControls::DEFAULT; num_cars];

    let last = recording.ticks.len().saturating_sub(stride + 1);
    for i in (0..=last).step_by(stride) {
        let from_tick = &recording.ticks[i];
        for (j, car_record) in recording.ticks[i + stride].car_records.iter().enumerate() {
            controls_buf[j] = car_record.prev_controls.into();
        }
        set_state_to_record_tick(
            &mut arena,
            &car_idcs,
            from_tick,
            i.checked_sub(stride).map(|i| &recording.ticks[i]),
            &controls_buf,
        );

        let mut pts: Vec<(Vec3A, Vec3A)> = Vec::new();
        for ev in arena.step_tick() {
            if let ArenaEvent::BallHitWorld(e) = ev {
                pts.push((e.contact_point, e.contact_normal));
            }
        }
        if i < lo || i > hi || pts.is_empty() {
            continue;
        }

        let from = &from_tick.ball_record;
        let to = &recording.ticks[i + stride].ball_record;
        let mean = pts
            .iter()
            .fold(Vec3A::ZERO, |a, &(_, n)| a + n)
            / pts.len() as f32;
        let rl_dv = Vec3A::from(to.lin_vel) - Vec3A::from(from.lin_vel);
        println!(
            "BALLPT {} t{i} np={} meannrm=({:.3},{:.3},{:.3}) |meannrm|={:.3} rldv=({:.1},{:.1},{:.1}) rldir=({:.3},{:.3},{:.3}) ballpos=({:.1},{:.1},{:.1})",
            recording.name,
            pts.len(),
            mean.x,
            mean.y,
            mean.z,
            mean.length(),
            rl_dv.x,
            rl_dv.y,
            rl_dv.z,
            rl_dv.normalize_or_zero().x,
            rl_dv.normalize_or_zero().y,
            rl_dv.normalize_or_zero().z,
            from.pos.x,
            from.pos.y,
            from.pos.z,
        );
        for (k, (pt, nrm)) in pts.iter().enumerate() {
            // Where the point sits relative to the ball centre, in radii: a real
            // surface contact sits one radius out along the inward normal.
            let rel = *pt - Vec3A::from(from.pos);
            println!(
                "BALLPT   #{k} nrm=({:.3},{:.3},{:.3}) pt=({:.1},{:.1},{:.1}) rel=({:.1},{:.1},{:.1}) |rel|={:.1} class={}",
                nrm.x,
                nrm.y,
                nrm.z,
                pt.x,
                pt.y,
                pt.z,
                rel.x,
                rel.y,
                rel.z,
                rel.length(),
                normal_class(*nrm),
            );
        }
    }
}

/// `RLBALL=4`: is a missed bounce a wrong impulse, or just a late one?
///
/// The per-tick bar charges a one-tick phase difference the *entire* impulse, so
/// a bounce the sim resolves one tick later than Rocket League scores as two
/// catastrophic steps even when the physics is exact. `ball_corner_fast_from_center`
/// is the clean example: the ball crosses the corner at 4340 uu/s (36 UU/tick),
/// RL splits the bounce across two ticks, and the sim -- restored to RL's state
/// every tick -- never sees a contact at all, because RL's own resolution keeps
/// the ball a few UU short of the sim's contact threshold at every tick boundary.
///
/// So measure the *outcome* instead of the tick. Free-run the sim across the
/// whole bounce and compare the outgoing velocity, allowing a small tick shift.
/// If a shift of one or two ticks collapses the error, the impulse is right and
/// only the phase is wrong -- which is a measurement artifact, not a physics
/// defect. If no shift helps, the impulse itself is wrong.
pub fn bounce_study(recording: &Recording) {
    /// Ticks of run-up before the bounce, so the sim enters it from clean free
    /// flight rather than from a state RL has already partly resolved.
    const PRE: usize = 3;
    /// Ticks to run past the bounce for both sides to have finished it.
    const HORIZON: usize = 10;
    /// Tick shifts to try when aligning the outcome.
    const MAX_SHIFT: i64 = 3;
    /// A car this close could be the thing that hit the ball, so the step is not
    /// a clean world bounce.
    const CAR_CLEAR: f32 = 500.0;

    let num_cars = recording.info.num_cars as usize;
    let stride = recording.stride;
    if stride != 1 {
        return;
    }
    let (mut arena, car_idcs) = make_arena(num_cars);
    let mut controls_buf: Vec<CarControls> = vec![CarControls::DEFAULT; num_cars];

    let n = recording.ticks.len();
    for i in PRE..n.saturating_sub(HORIZON + MAX_SHIFT as usize + 1) {
        let from = &recording.ticks[i].ball_record;
        let to = &recording.ticks[i + 1].ball_record;
        if is_sentinel(from) || is_sentinel(to) {
            continue;
        }
        // A bounce in the ground truth: a velocity change far beyond gravity.
        let dv = Vec3A::from(to.lin_vel) - Vec3A::from(from.lin_vel);
        if dv.length() < 100.0 {
            continue;
        }
        // Label rather than skip: the same outcome test answers the car-ball
        // question, and the two subsystems must be reported apart.
        let p = Vec3A::from(from.pos);
        let gap_to_car = recording.ticks[i]
            .car_records
            .iter()
            .chain(recording.ticks[i + 1].car_records.iter())
            .filter(|c| !c.is_demoed && !is_car_sentinel(&c.phys))
            .map(|c| hitbox_dist(p, &c.phys) - BALL_RADIUS)
            .fold(f32::INFINITY, f32::min);
        let kind = if gap_to_car < 40.0 {
            "car"
        } else if gap_to_car < CAR_CLEAR {
            // Too close to attribute cleanly to either subsystem.
            continue;
        } else {
            "world"
        };

        let start = i - PRE;
        // Restore once, then run free: this measures the trajectory across the
        // bounce rather than one tick of it.
        let start_tick = &recording.ticks[start];
        for (j, car_record) in recording.ticks[start + 1].car_records.iter().enumerate() {
            controls_buf[j] = car_record.prev_controls.into();
        }
        set_state_to_record_tick(
            &mut arena,
            &car_idcs,
            start_tick,
            start.checked_sub(1).map(|i| &recording.ticks[i]),
            &controls_buf,
        );

        let mut sim_vels: Vec<Vec3A> = Vec::with_capacity(HORIZON + 1);
        let mut n_contacts = 0usize;
        for k in 0..HORIZON {
            let ci = (start + k + 1).min(n - 1);
            for (j, car_record) in recording.ticks[ci].car_records.iter().enumerate() {
                controls_buf[j] = car_record.prev_controls.into();
            }
            for (j, &car_idx) in car_idcs.iter().enumerate() {
                let mut cs = *arena.get_car_state(car_idx);
                cs.controls = controls_buf[j];
                arena.set_car_state(car_idx, cs);
            }
            for ev in arena.step_tick() {
                if matches!(ev, ArenaEvent::BallHitWorld(_)) {
                    n_contacts += 1;
                }
            }
            sim_vels.push(arena.get_ball_state().phys.vel);
        }

        let sim_end = sim_vels[HORIZON - 1];
        let rl_at = |t: i64| -> Option<Vec3A> {
            let t = t as usize;
            if t >= n {
                return None;
            }
            let r = &recording.ticks[t].ball_record;
            if is_sentinel(r) {
                None
            } else {
                Some(Vec3A::from(r.lin_vel))
            }
        };
        let base = (start + HORIZON) as i64;
        let err0 = rl_at(base).map_or(f32::NAN, |v| (sim_end - v).length());
        let mut best = f32::INFINITY;
        let mut best_shift = 0i64;
        for sh in -MAX_SHIFT..=MAX_SHIFT {
            if let Some(v) = rl_at(base + sh) {
                let e = (sim_end - v).length();
                if e < best {
                    best = e;
                    best_shift = sh;
                }
            }
        }
        // Magnitude of the bounce being judged: how much velocity RL removed or
        // added across the whole event. An outcome error has to be read against
        // this, not against zero.
        let pre = rl_at(i as i64).unwrap_or(Vec3A::ZERO);
        let post = rl_at((i + PRE) as i64).unwrap_or(Vec3A::ZERO);
        let impulse = (post - pre).length();
        println!(
            "BALLBOUNCE {} t{i} kind={kind} speed={:.0} impulse={impulse:.0} err0={err0:.1} best={best:.1} shift={best_shift} simcontacts={n_contacts} pos=({:.0},{:.0},{:.0})",
            recording.name,
            pre.length(),
            p.x,
            p.y,
            p.z,
        );
    }
}

/// Distance from a world point to the surface of a car's oriented hitbox (0
/// inside). The same box `census::ball_touches_car` tests, but returning the
/// distance instead of a slack-thresholded bool.
fn hitbox_dist(point: Vec3A, car: &PhysRecord) -> f32 {
    let rot = Mat3A::from_cols(
        car.rot.rows[0].into(),
        car.rot.rows[1].into(),
        car.rot.rows[2].into(),
    );
    let local = rot.transpose() * (point - Vec3A::from(car.pos)) - HITBOX_OFFSET;
    Vec3A::new(
        (local.x.abs() - HITBOX_HALF.x).max(0.0),
        (local.y.abs() - HITBOX_HALF.y).max(0.0),
        (local.z.abs() - HITBOX_HALF.z).max(0.0),
    )
    .length()
}

/// `RLBALL=2`: is anything in the ball's world-contact triple actually written?
pub fn survey(recording: &Recording) {
    let mut n_ticks = 0u64;
    let mut n_flag = 0u64;
    let mut n_pt = 0u64;
    let mut n_nrm_nonzero = 0u64;
    let mut n_nrm_not_up = 0u64;
    // The flag's usefulness is whether it tracks a real bounce. Compare it
    // against a purely kinematic test: a velocity change far larger than
    // gravity's 5.4167 uu/s per tick is a contact whatever the flag says.
    let mut n_bounce = 0u64;
    let mut n_bounce_flagged = 0u64;
    // Internal consistency of the ball's own ground truth, with no simulation
    // involved. Bullet integrates `pos += vel * dt` using the *post*-step
    // velocity, so across a recorded step `(pos_to - pos_from) / dt` must equal
    // `vel_to` exactly. Where it does not, the recording's position and velocity
    // channels disagree with each other and the step cannot grade anything.
    let mut n_self_inconsistent = 0u64;
    let mut worst_self = 0.0f32;
    let mut self_sum = 0.0f64;

    let stride = recording.stride;
    let last = recording.ticks.len().saturating_sub(stride + 1);
    for i in (0..=last).step_by(stride) {
        let from = &recording.ticks[i].ball_record;
        let to = &recording.ticks[i + stride].ball_record;
        if is_sentinel(from) || is_sentinel(to) {
            continue;
        }
        n_ticks += 1;
        if from.has_world_contact {
            n_flag += 1;
        }
        let pt = Vec3A::from(from.world_contact_point);
        let nrm = Vec3A::from(from.world_contact_normal);
        if pt.length() > 1e-6 {
            n_pt += 1;
        }
        if nrm.length() > 1e-6 {
            n_nrm_nonzero += 1;
        }
        if (nrm - Vec3A::Z).length() > 1e-6 {
            n_nrm_not_up += 1;
        }

        let implied = (Vec3A::from(to.pos) - Vec3A::from(from.pos)) / (TICK_TIME * stride as f32);
        let self_err = (implied - Vec3A::from(to.lin_vel)).length();
        if self_err > SELF_CONSISTENCY_TOL {
            n_self_inconsistent += 1;
        }
        worst_self = worst_self.max(self_err);
        self_sum += f64::from(self_err);

        let dv = Vec3A::from(to.lin_vel) - Vec3A::from(from.lin_vel);
        if dv.length() > 30.0 {
            n_bounce += 1;
            if from.has_world_contact || to.has_world_contact {
                n_bounce_flagged += 1;
            }
        }
    }

    println!(
        "BALLSURVEY {} ticks={n_ticks} flag={n_flag} pt_nonzero={n_pt} nrm_nonzero={n_nrm_nonzero} nrm_not_up={n_nrm_not_up} bounce={n_bounce} bounce_flagged={n_bounce_flagged} selfinc={n_self_inconsistent} selfmax={worst_self:.1} selfsum={self_sum:.0}",
        recording.name
    );
}

/// `RLBALL=1` (census) and `RLBALL=3` (per-step dump).
pub fn analyze(recording: &Recording) {
    let dump = matches!(std::env::var("RLBALL").as_deref(), Ok("3"));
    let num_cars = recording.info.num_cars as usize;
    let stride = recording.stride;
    let (mut arena, car_idcs) = make_arena(num_cars);
    let mut controls_buf: Vec<CarControls> = vec![CarControls::DEFAULT; num_cars];
    let mut buckets: BTreeMap<String, Bucket> = BTreeMap::new();

    let last = recording.ticks.len().saturating_sub(stride + 1);

    // Every step must be taken, candidate or not: `set_state_to_record_tick`
    // does not restore Bullet's warm-started persistent contact manifolds, so
    // skipping a step changes the answer on the next one.
    for i in (0..=last).step_by(stride) {
        let discontinuous = has_discontinuity(recording, i, stride);

        let from_tick = &recording.ticks[i];
        let to_tick = &recording.ticks[i + stride];
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

        // Collect the ball's world contacts this tick from the sim's own event
        // stream: how many there were, and their normals.
        let mut normals: Vec<Vec3A> = Vec::new();
        for ev in arena.step_tick() {
            if let ArenaEvent::BallHitWorld(e) = ev {
                normals.push(e.contact_normal);
            }
        }

        if discontinuous {
            continue;
        }
        let from = &from_tick.ball_record;
        let to = &to_tick.ball_record;
        if is_sentinel(from) || is_sentinel(to) {
            continue;
        }

        let sim = arena.get_ball_state().phys;
        let resid = sim.vel - Vec3A::from(to.lin_vel);
        let err = resid.length();
        let ang_err = (sim.ang_vel - Vec3A::from(to.ang_vel)).length();

        // Is a live car touching the ball? Then the impulse is the car-ball
        // subsystem, not the arena mesh.
        let car_touch = from_tick
            .car_records
            .iter()
            .chain(to_tick.car_records.iter())
            .any(|c| {
                !c.is_demoed
                    && !is_car_sentinel(&c.phys)
                    && (ball_touches_car(Vec3A::from(from.pos), &c.phys)
                        || ball_touches_car(Vec3A::from(to.pos), &c.phys))
            });

        // Mean normal, for the signed along-normal residual.
        let mean_nrm = normals
            .iter()
            .fold(Vec3A::ZERO, |a, &b| a + b)
            .normalize_or_zero();
        let along = if mean_nrm == Vec3A::ZERO {
            None
        } else {
            Some(resid.dot(mean_nrm))
        };

        // The ball's own `physics_frame` is validated nowhere in the harness:
        // `runner::frame_advance_broken` walks `car_records` only. The logger
        // samples the ball asynchronously too, so if its frame does not advance
        // exactly `stride` then the recording's own `from -> to` is not the step
        // the sim was asked to take, and the divergence is arithmetic rather
        // than physics -- the same defect that turned out to own three quarters
        // of the car's `car_proximity` bucket. This test takes priority over
        // every physical bucket so the mass cannot hide inside one.
        let badv = to.physics_frame as i64 - from.physics_frame as i64;
        let key = if badv != stride as i64 {
            format!("ballframe/adv{badv}")
        } else if car_touch {
            "car_ball".to_string()
        } else if normals.is_empty() {
            "free".to_string()
        } else {
            // Distinct surfaces this tick, and how many contact points. An edge
            // between two triangles reports two points; if the solver applies a
            // full restitution impulse per point the bounce comes out twice as
            // elastic as it should be, which is the leading hypothesis for why
            // flat faces are accurate and edges are not.
            let mut classes: Vec<&str> = normals.iter().map(|&n| normal_class(n)).collect();
            classes.sort_unstable();
            classes.dedup();
            format!("world/{}/np{}", classes.join("+"), normals.len().min(4))
        };

        if dump {
            println!(
                "BALLDUMP {} t{i} err={err:.3} ang={ang_err:.3} np={} key={key} pos=({:.1},{:.1},{:.1}) v0=({:.1},{:.1},{:.1}) v1=({:.1},{:.1},{:.1}) sim=({:.1},{:.1},{:.1}) nrm=({:.3},{:.3},{:.3}) rlflag={} rlnrm=({:.3},{:.3},{:.3}) rlpt=({:.1},{:.1},{:.1}) rlptd={:.2}",
                recording.name,
                normals.len(),
                from.pos.x,
                from.pos.y,
                from.pos.z,
                from.lin_vel.x,
                from.lin_vel.y,
                from.lin_vel.z,
                to.lin_vel.x,
                to.lin_vel.y,
                to.lin_vel.z,
                sim.vel.x,
                sim.vel.y,
                sim.vel.z,
                mean_nrm.x,
                mean_nrm.y,
                mean_nrm.z,
                u8::from(from.has_world_contact),
                from.world_contact_normal.x,
                from.world_contact_normal.y,
                from.world_contact_normal.z,
                from.world_contact_point.x,
                from.world_contact_point.y,
                from.world_contact_point.z,
                // Distance from the ball centre to the logged contact point. A
                // real contact point on the ball surface sits one radius away.
                (Vec3A::from(from.world_contact_point) - Vec3A::from(from.pos)).length(),
            );
        }

        // A step the sim thinks is free flight but which diverges badly is a
        // contact Rocket League resolved and the sim never saw. Free flight is
        // otherwise exact to 0.01 uu/s, so on such a step the *entire* residual
        // is RL's own impulse -- direction included. That makes the missed
        // contact a closed channel: `dir` below is the contact normal RL used,
        // recovered without simulating anything.
        if key == "free" && err > MISS_THRESHOLD {
            let dir = (Vec3A::from(to.lin_vel) - sim.vel).normalize_or_zero();
            let p = Vec3A::from(from.pos);
            // Gap from the ball *surface* to the nearest live car's hitbox. The
            // bucket test uses a fixed 30 UU slack; measuring the real gap is
            // what separates "the sim missed an arena contact" from "a car hit
            // the ball just outside the slack".
            let dcar = from_tick
                .car_records
                .iter()
                .chain(to_tick.car_records.iter())
                .filter(|c| !c.is_demoed && !is_car_sentinel(&c.phys))
                .map(|c| hitbox_dist(p, &c.phys) - BALL_RADIUS)
                .fold(f32::INFINITY, f32::min);
            println!(
                "BALLMISS {} t{i} err={err:.1} pos=({:.1},{:.1},{:.1}) v0=({:.1},{:.1},{:.1}) dir=({:.3},{:.3},{:.3}) dfloor={:.1} dceil={:.1} dside={:.1} dback={:.1} dcorner={:.1} dcar={:.1} badv={badv} angerr={ang_err:.3}",
                recording.name,
                p.x,
                p.y,
                p.z,
                from.lin_vel.x,
                from.lin_vel.y,
                from.lin_vel.z,
                dir.x,
                dir.y,
                dir.z,
                // Gap between the ball surface and each nominal flat plane of
                // the soccar arena. Negative means the ball is already past it.
                p.z - BALL_RADIUS,
                (ARENA_Z - p.z) - BALL_RADIUS,
                (ARENA_X - p.x.abs()) - BALL_RADIUS,
                (ARENA_Y - p.y.abs()) - BALL_RADIUS,
                // Distance to the 45 degree corner chamfer plane.
                (CORNER_C - (p.x.abs() + p.y.abs())) / SQRT2 - BALL_RADIUS,
                dcar,
            );
        }

        buckets.entry(key).or_default().add(err, ang_err, i, along);
    }

    if dump {
        return;
    }
    for (key, b) in &mut buckets {
        let (p50, p90, p99) = (b.pct(0.50), b.pct(0.90), b.pct(0.99));
        let nb = if b.normal_n > 0 {
            b.normal_bias / b.normal_n as f64
        } else {
            0.0
        };
        println!(
            "BALLCENSUS[{}] {key} n={} sum={:.3} sumsq={:.3} max={:.3}@t{} p50={p50:.3} p90={p90:.3} p99={p99:.3} ang={:.4} nrmbias={nb:.3} n1={} n10={} n100={}",
            recording.name,
            b.n,
            b.sum,
            b.sum_sq,
            b.max,
            b.max_tick,
            b.ang_sum / b.n as f64,
            b.n_over_1,
            b.n_over_10,
            b.n_over_100,
        );
    }
}
