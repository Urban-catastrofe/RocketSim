//! Arena driving: restore state from recordings, step the sim, and feed the
//! lean per-tick deltas into a [`Report`].
//!
//! Two modes:
//! - [`run_per_tick`] restores the full recorded state every tick, isolating
//!   single-tick physics accuracy. This is the gated, deep-divable mode.
//! - [`run_continuous`] sets state once at tick 0 and runs freely, showing how
//!   errors compound. Informational only.

use rocketsim::{
    Arena, BallHitState, BallState, CarBodyConfig, CarControls, CarState, GameMode, HitCadence,
    PhysState, Team,
};

use super::config::HarnessConfig;
use super::measure::compute_delta;
use super::recording::Recording;
use super::recording::cpp_records::{CarRecord, PhysRecord};
use super::recording::tick_record::TickRecord;
use super::report::Report;
use super::stats::Field;

/// The logger parks the ball at a sentinel position far below the arena when it
/// is not part of a (car-focused) scenario. A real ball is always inside the
/// arena (z >= ~ball radius), so a deeply-negative z marks an absent ball —
/// comparing against it would fabricate a huge divergence.
pub fn is_ball_sentinel(phys: &PhysRecord) -> bool {
    phys.pos.z < -1000.0
}

/// The logger parks an absent car at the origin (0,0,0) with zero velocity.
///
/// This was long assumed to be how a demolished car is recorded, but it is
/// not: `RLDIAG=1` over all 440 recordings finds 42 parkings and every one
/// is on the recording's own last tick, all cars at once -- logger teardown,
/// not a demolition. The ground truth contains no demolition in any form
/// (`is_demoed` never set, no origin-parking mid-recording, no frozen
/// respawn span). The guard is still needed for those teardown ticks. A real car centre cannot sit below ~z=17 on
/// flat ground (its hitbox keeps it above the floor), so a near-origin position
/// with zero velocity marks a parked/demoed car — the sim restoring that pose
/// and running full physics fabricates a huge divergence (e.g. the ground
/// shoving the embedded hitbox upward at ~158 UU/s).
pub fn is_car_sentinel(phys: &PhysRecord) -> bool {
    // A real car centre never sits at the arena origin at rest (the floor keeps
    // it at z~17), so an origin-parked, inert pose marks an absent car — the sim restoring that pose and running full physics fabricates a
    // huge divergence (the ground shoves the embedded hitbox upward).
    let v = phys.lin_vel;
    let w = phys.ang_vel;
    let v_len_sq = v.x * v.x + v.y * v.y + v.z * v.z;
    let w_len_sq = w.x * w.x + w.y * w.y + w.z * w.z;
    phys.pos.x.abs() < 1.0
        && phys.pos.y.abs() < 1.0
        && phys.pos.z < 5.0
        && v_len_sq < 25.0
        && w_len_sq < 1.0
}

/// Max displacement any entity can physically travel in one recorded step.
/// Cars top out near 2300 UU/s (~19 UU/tick at 120 Hz) and the ball near
/// ~50 UU/tick, so anything beyond this is a game-state discontinuity — a goal
/// reset, demo/respawn, or a logger car-index swap — not physics the sim can
/// reproduce. Such ticks are voided from measurement.
const MAX_PHYS_DISPLACEMENT: f32 = 100.0;

/// Ball radius in UU, and the nominal soccar arena extents.
///
/// The real geometry is the collision mesh; these planes are a deliberately
/// *conservative* stand-in, used only to answer "was the ball anywhere near a
/// surface". Every real feature the mesh has -- goal mouth, crossbar, corner
/// chamfer, ramps -- sits at or inside these planes, so a ball this test calls
/// far from a surface is genuinely far from one.
pub(crate) const BALL_RADIUS: f32 = 91.25;
pub(crate) const ARENA_X: f32 = 4096.0;
pub(crate) const ARENA_Y: f32 = 5120.0;
pub(crate) const ARENA_Z: f32 = 2048.0;
/// Conservative diagonal bound on the soccar corner, `|x| + |y| = CORNER_C`.
///
/// Not a chamfer: `RLBALL=9` walks the ball out along `x = y` and finds the mesh
/// answering at an implied plane of 7849 (z=1800) to 8009 (z=300..1200), and off
/// the diagonal the corner is roomier still -- `ball_corner_trap_slow` rests the
/// ball at `|x| + |y| = 8760` against ordinary contacts. The corner is a fillet
/// and no single plane describes it. This is set to the *tightest* value the
/// probe found so the plane stays at or inside the mesh everywhere, which is the
/// direction that keeps [`ball_gap_to_surface`] from ever calling a ball far
/// from a surface that is actually beside one. An earlier 8064 was guesswork and
/// sat outside the mesh, breaking that invariant.
pub(crate) const CORNER_C: f32 = 7849.0;
pub(crate) const SQRT2: f32 = std::f32::consts::SQRT_2;
/// Octane hitbox half-extents and centre offset, as in `census.rs`. Every car in
/// the suite is an Octane.
pub(crate) const HITBOX_HALF: glam::Vec3A =
    glam::Vec3A::new(120.507 / 2.0, 86.6994 / 2.0, 38.6591 / 2.0);
pub(crate) const HITBOX_OFFSET: glam::Vec3A = glam::Vec3A::new(13.8757, 0.0, 20.755);

/// Distance from a world point to the surface of a car's oriented hitbox (0
/// inside).
pub(crate) fn hitbox_dist(point: glam::Vec3A, car: &PhysRecord) -> f32 {
    let rot = glam::Mat3A::from_cols(
        car.rot.rows[0].into(),
        car.rot.rows[1].into(),
        car.rot.rows[2].into(),
    );
    let local = rot.transpose() * (point - glam::Vec3A::from(car.pos)) - HITBOX_OFFSET;
    glam::Vec3A::new(
        (local.x.abs() - HITBOX_HALF.x).max(0.0),
        (local.y.abs() - HITBOX_HALF.y).max(0.0),
        (local.z.abs() - HITBOX_HALF.z).max(0.0),
    )
    .length()
}

/// Gap from the ball's surface to the nearest nominal arena surface. Negative
/// inside the goal mouth, which is why this test can never flag a goal-frame
/// bounce.
fn ball_gap_to_surface(p: glam::Vec3A) -> f32 {
    [
        p.z - BALL_RADIUS,
        (ARENA_Z - p.z) - BALL_RADIUS,
        (ARENA_X - p.x.abs()) - BALL_RADIUS,
        (ARENA_Y - p.y.abs()) - BALL_RADIUS,
        (CORNER_C - (p.x.abs() + p.y.abs())) / SQRT2 - BALL_RADIUS,
    ]
    .into_iter()
    .fold(f32::INFINITY, f32::min)
}

/// Slack on the embedding test below, in UU. Bullet's narrowphase only creates
/// points within `CONTACT_BREAKING_THRESHOLD` (0.02 BT = 1 UU) of a surface, so
/// this is the depth a step can produce over and above its own approach.
const BALL_EMBED_SLACK: f32 = 1.0;

/// Fraction of a recording's ball steps that must be embedded in the arena mesh
/// before its ball ground truth is thrown out wholesale.
///
/// Depth alone cannot make this call. A ball genuinely bouncing out of the
/// goal-post base reports up to 13.6 UU of depth the step cannot explain,
/// because the deepest of 47 coincident points on that manifold overstates the
/// real overlap -- and those steps are exactly where the ball's remaining error
/// lives, so hiding them would be hiding the problem. What separates them is not
/// how deep but how long: a real wedge lasts two or three ticks, whereas
/// `ball_corner_exit_high` and `ball_corner_exit_low` spawn the ball 74-80 UU
/// inside the corner wall at tick 0 and leave it there while Rocket League
/// applies no contact force at all.
///
/// Measured over the suite the split is not close: those two recordings violate
/// on 89% and 100% of their ball steps, and every other recording that violates
/// at all does so on at most 3%. Anything from roughly 5% to 85% picks out the
/// same two.
const BALL_EMBED_RECORDING_FRACTION: f64 = 0.25;

/// True when the *recorded* ball position sits inside the arena collision mesh
/// deeper than the step's own motion into that surface can account for.
///
/// The bound is the approach along the contact normal, not the total distance
/// travelled. Sliding fast *along* a wall does not entitle the ball to be inside
/// it: only motion into the surface can create penetration, so the depth a step
/// can produce is `max(0, -v . n) * dt` plus the narrowphase's own threshold.
/// Using total speed instead leaves exactly the wrong steps behind —
/// `ball_corner_exit_low` kept precisely one of 239, the single worst, because
/// the ball was moving quickly across the corner while buried 70 UU inside it.
///
/// A recorded state deeper than this was not produced by a step against this
/// mesh — Rocket League would have pushed it out and did not. That makes it the
/// same bound the ball's positional push-out is capped at in
/// `convert_contact_special`: a state the push-out could not have created is one
/// it must not be asked to undo, and grading against it measures a disagreement
/// about where the wall is rather than any physics.
///
/// The reference is the mesh itself, through the contact points the narrowphase
/// generated — not the nominal planes above, which describe the corner far too
/// crudely to judge this (see [`CORNER_C`]).
///
/// `RLBALL=8` sweeps the suite. Violations land in three recordings only:
/// `ball_corner_exit_high` and `ball_corner_exit_low` spawn the ball 74-80 UU
/// inside the corner wall at tick 0 and leave it there for the whole recording
/// while Rocket League applies no contact force at all, and
/// `ball_goal_post_base_right` has two transient steps against a 47-point
/// manifold. The other 436 recordings never violate the mesh, corner-heavy ones
/// included — which is what says the mesh is right and those recordings are not.
fn ball_embedded_in_mesh(deepest_penetration: f32, approach_speed: f32, dt: f32) -> bool {
    deepest_penetration > approach_speed.max(0.0) * dt + BALL_EMBED_SLACK
}

/// Throw out a recording's whole ball channel when the ball spends most of the
/// recording inside the arena mesh -- see [`BALL_EMBED_RECORDING_FRACTION`].
///
/// Applied after the shards merge rather than per step, because the test is a
/// property of the recording and no single step can answer it. Individually
/// suspicious steps are therefore still measured: on a recording that passes
/// this test they are real contacts, and dropping them would delete signal.
pub(crate) fn reject_embedded_ball_ground_truth(report: &mut Report) {
    if keep_corrupt_steps() || report.ball_steps_embedded == 0 {
        return;
    }
    let ball_index = report.num_cars;
    let measured = report.entities[ball_index].measured_steps();
    let total = measured + report.ball_steps_embedded;
    if total == 0 {
        return;
    }
    if (report.ball_steps_embedded as f64) / (total as f64) < BALL_EMBED_RECORDING_FRACTION {
        return;
    }
    report.entities[ball_index].clear_samples();
    report.ball_ground_truth_rejected = true;
}

/// How far the ball's recorded velocity may sit from the velocity implied by its
/// own recorded position change before the step is suspect.
///
/// Rocket League applies a contact impulse *after* integrating the transform, so
/// on a contact step `(pos_to - pos_from) / dt` legitimately disagrees with
/// `vel_to` by exactly one impulse. That is every bounce in the suite and must
/// not be voided, which is why disagreement alone is not the test -- see
/// [`ball_record_impossible`].
const BALL_CHANNEL_DISAGREEMENT: f32 = 500.0;

/// How close a car or a surface must be for a large disagreement to have a
/// possible cause. Two ticks of travel at the ball's 6 000 uu/s cap is 100 UU,
/// so nothing resolved anywhere inside the step can be farther away than this.
const BALL_CAUSE_RANGE: f32 = 100.0;

/// True when the ball's own ground truth for this step describes something
/// Rocket League cannot have done, so the step cannot grade the sim.
///
/// The defect is in the ball's *velocity* channel, and it is distinct from the
/// car-side corruption [`has_duplicate_car`] and [`frame_advance_broken`] guard
/// against. The logger samples the ball's velocity separately from its position;
/// where the two contradict each other by more than the whole arena could
/// explain, the record is wrong rather than surprising. At `2v2` t4178-t4182 the
/// recorded velocity flips sign three times in five ticks while the recorded
/// position moves smoothly, with no car within 900 UU and no surface within 21.
///
/// A disagreement on its own is *not* enough — it is the normal signature of a
/// bounce. The test is a disagreement **with no possible source**: no car and no
/// arena surface anywhere near the ball at either end of the step. Measured over
/// the suite that separates completely: it fires on 99 steps, every one of them
/// in a match replay and none in a scripted recording, and there is a wide
/// plateau around both thresholds rather than a close call to arbitrate.
///
/// Those 99 steps are 0.063% of the ball's measured steps and carry **18.3% of
/// the suite's whole ball velocity error mass** — the same shape as the
/// duplicated-car defect on the car side.
///
/// Only the ball's own rows are dropped: the cars on these steps are fine, and
/// [`has_discontinuity`] already covers the defects that spoil a whole tick
/// (including the 631 156 uu/s cases, which are position teleports).
pub fn ball_record_impossible(from: &TickRecord, to: &TickRecord, stride: usize) -> bool {
    if keep_corrupt_steps() {
        return false;
    }
    let p0: glam::Vec3A = from.ball_record.pos.into();
    let p1: glam::Vec3A = to.ball_record.pos.into();
    let v_to: glam::Vec3A = to.ball_record.lin_vel.into();
    let implied = (p1 - p0) / (rocketsim::consts::TICK_TIME * stride as f32);
    if (implied - v_to).length() <= BALL_CHANNEL_DISAGREEMENT {
        return false;
    }

    let near_surface = ball_gap_to_surface(p0).min(ball_gap_to_surface(p1)) < BALL_CAUSE_RANGE;
    if near_surface {
        return false;
    }
    let near_car = from
        .car_records
        .iter()
        .chain(to.car_records.iter())
        .filter(|c| !c.is_demoed && !is_car_sentinel(&c.phys))
        .any(|c| {
            hitbox_dist(p0, &c.phys).min(hitbox_dist(p1, &c.phys)) - BALL_RADIUS < BALL_CAUSE_RANGE
        });
    !near_car
}

/// Restore+step cycles to run on a fresh shard arena before measuring, so the
/// Bullet contact manifolds settle (a cold arena's first steps diverge).
///
/// These cycles are spent re-stepping the shard's *first* tick and their results
/// are thrown away, so every tick in the shard's range is still measured.
/// Previously the warmup consumed the first two measured ticks instead, which
/// silently hid real divergence — a genuine 7.5 UU/s jump-impulse error at t=0
/// went unreported — and made the secondary stats depend on the thread count,
/// because each shard dropped its own first two ticks.
pub const SHARD_WARMUP_TICKS: usize = 2;

/// Smallest Octane hitbox dimension (UU): 38.6591 tall, against 120.507 long and
/// 86.6994 wide. Two identical boxes cannot have their centres closer than the
/// smallest dimension in any orientation whatsoever, so a live pair below it is
/// not a physical state.
const MIN_HITBOX_DIM: f32 = 38.6591;

/// Is this car's record real ground truth this tick? A demoed car is parked by
/// the logger, either at the origin or below the floor.
fn car_live(car: &CarRecord) -> bool {
    !car.is_demoed && car.phys.pos.z > -1000.0 && !is_car_sentinel(&car.phys)
}

/// True if this tick's live cars do not all share one `physics_frame`.
///
/// The match replays sample cars asynchronously, so this is *common* and mostly
/// harmless: `2v2_4` runs 4.7% of ticks and `3v3` 93.7%, and a one-frame stagger
/// between two cars 3000 UU apart changes nothing. Every scripted recording is
/// at exactly 0.0000. On its own this is not a defect and must not void a tick.
fn frames_disagree(tick: &TickRecord) -> bool {
    let mut seen: Option<u32> = None;
    for car in tick.car_records.iter().filter(|c| car_live(c)) {
        match seen {
            None => seen = Some(car.phys.physics_frame),
            Some(f) if f != car.phys.physics_frame => return true,
            _ => {}
        }
    }
    false
}

/// True if this tick holds the same physical car in two slots.
///
/// When the logger's array rotates it can write one car into two slots a frame
/// apart and lose another car entirely. At tick 356 of `2v2_1` the four slots
/// carry `physics_frame` 357, 358, 358, 357; slot 1 holds slot 0's car advanced
/// one frame (same `fwd` and `up` to three decimals, velocity within 6 UU/s,
/// centre offset exactly one frame of travel along its own velocity) and the car
/// that was in slot 1 the tick before, 3200 UU away, is gone. `car_order` cannot
/// repair it: its matching is bijective, so it is forced to file both copies
/// under separate canonical slots with nothing to put in the vacated one.
///
/// Both halves of the test are needed. Impossible proximity alone
/// false-positives on `car_car_boost_contest` and `car_car_long_boost_headon`,
/// which each spend 6 steps genuinely interpenetrating during a head-on boost
/// collision -- a real state RL is resolving, at an ordinary 10 UU/s of error
/// rather than the 174 the collapsed ticks carry -- and both are scripted with a
/// uniform `physics_frame` on every tick. Requiring disagreeing frames as well
/// removes exactly those false positives and nothing else.
pub fn has_duplicate_car(tick: &TickRecord) -> bool {
    if !frames_disagree(tick) {
        return false;
    }
    let live: Vec<glam::Vec3A> = tick
        .car_records
        .iter()
        .filter(|c| car_live(c))
        .map(|c| c.phys.pos.into())
        .collect();
    for i in 0..live.len() {
        for j in (i + 1)..live.len() {
            if (live[i] - live[j]).length() < MIN_HITBOX_DIM {
                return true;
            }
        }
    }
    false
}

/// True if any live car's own ground truth did not advance exactly `stride`
/// physics frames across the step.
///
/// The per-car frame stagger is harmless while it holds steady, because the
/// car's own `from -> to` is still one frame. When it *flips* mid-step the
/// recording moved two or three frames while the sim is asked for one, and the
/// resulting divergence is arithmetic, not physics: those steps average 58 UU/s
/// against the suite's 3.
fn frame_advance_broken(from: &TickRecord, to: &TickRecord, stride: usize) -> bool {
    for (fc, tc) in from.car_records.iter().zip(&to.car_records) {
        if !car_live(fc) || !car_live(tc) {
            continue;
        }
        if tc.phys.physics_frame != fc.phys.physics_frame + stride as u32 {
            return true;
        }
    }
    false
}

/// `RLKEEPDUP=1` keeps the steps [`has_duplicate_car`] and
/// [`frame_advance_broken`] reject *in* the measurement, so the mass they carry
/// can be re-measured after the fact. They are 0.7% of the suite's steps and 21%
/// of its error mass, so leaving them in inflates every reported mean by ~25%.
fn keep_corrupt_steps() -> bool {
    static KEEP: std::sync::OnceLock<bool> = std::sync::OnceLock::new();
    *KEEP.get_or_init(|| matches!(std::env::var("RLKEEPDUP").as_deref(), Ok("1") | Ok("true")))
}

/// True if the recording's `i -> i + stride` transition is non-physical: any car
/// or the ball jumps farther than [`MAX_PHYS_DISPLACEMENT`], a car's own
/// `physics_frame` does not advance by `stride`, or the tick holds one physical
/// car in two slots.
pub fn has_discontinuity(recording: &Recording, i: usize, stride: usize) -> bool {
    let from = &recording.ticks[i];
    let to = &recording.ticks[i + stride];

    if !keep_corrupt_steps()
        && (frame_advance_broken(from, to, stride)
            || has_duplicate_car(from)
            || has_duplicate_car(to))
    {
        return true;
    }

    for (fc, tc) in from.car_records.iter().zip(&to.car_records) {
        let a: glam::Vec3A = fc.phys.pos.into();
        let b: glam::Vec3A = tc.phys.pos.into();
        if (b - a).length() > MAX_PHYS_DISPLACEMENT {
            return true;
        }
    }

    if !is_ball_sentinel(&to.ball_record) {
        let a: glam::Vec3A = from.ball_record.pos.into();
        let b: glam::Vec3A = to.ball_record.pos.into();
        if (b - a).length() > MAX_PHYS_DISPLACEMENT {
            return true;
        }
    }

    false
}

pub fn make_arena(num_cars: usize) -> (Arena, Vec<usize>) {
    let mut arena = Arena::new(GameMode::Soccar);
    if let Ok(scale) = std::env::var("RL_BALL_HIT_SCALE") {
        let mut mutators = *arena.mutator_config();
        mutators.ball_hit_extra_force_scale = scale
            .parse()
            .expect("RL_BALL_HIT_SCALE must be a finite number");
        assert!(
            mutators.ball_hit_extra_force_scale.is_finite(),
            "RL_BALL_HIT_SCALE must be a finite number"
        );
        arena.set_mutator_config(mutators);
    }
    if let Ok(cadence) = std::env::var("RL_HIT_CADENCE") {
        let mut config = *arena.ball_hit_config();
        config.cadence = match cadence.as_str() {
            "once" => HitCadence::OncePerEpisode,
            "other" => HitCadence::EveryOtherTick,
            "every" => HitCadence::EveryTick,
            _ => panic!("RL_HIT_CADENCE must be once, other, or every"),
        };
        arena.set_ball_hit_config(config);
    }
    // Rocket League never demolishes in this ground truth, so neither may the
    // sim. Across all 440 recordings the observer sets `is_demoed` exactly zero
    // times, and the origin-parking that actually marks a demoed car
    // (`is_car_sentinel`) shows up only in the match replays. The four scripted
    // cases written to probe the threshold settle it: in
    // `car_car_above_demo_speed` a supersonic car hits a stationary one dead
    // centre at 2300 UU/s -- opposite teams, inside every cone, forward-axis
    // speed at the cap -- and RL *bumps* it. The victim leaves at 2272 UU/s
    // with +346 vz and is still flying 200 ticks later. `head_on_demo`,
    // `long_accel_to_demo` and `demo_airborne_victim` do the same.
    //
    // The sim demolishes instead, which freezes the victim for the rest of the
    // window and reads as a ~2300 UU/s error -- the five worst bump events in
    // the suite, 7.3% of all impulse-window error mass.
    //
    // Whether RL's threshold is stricter than ours or demolitions were simply
    // off in the recording session is *not decidable from the recording*: it
    // carries no team field at all, and the teams assigned below are the
    // harness's own index-parity convention, not ground truth. Since the sim
    // cannot infer either, it is told. `RL_DEMOS=on` restores the default for
    // anyone investigating the threshold itself.
    if !matches!(std::env::var("RL_DEMOS").as_deref(), Ok("on")) {
        let mut mutators = *arena.mutator_config();
        mutators.demo_mode = rocketsim::DemoMode::Disabled;
        arena.set_mutator_config(mutators);
    }
    let car_idcs: Vec<usize> = (0..num_cars)
        .map(|i| {
            let team = if (i % 2) == 0 {
                Team::Blue
            } else {
                Team::Orange
            };
            arena.add_car(team, CarBodyConfig::OCTANE)
        })
        .collect();
    (arena, car_idcs)
}

/// Restore recorded public state plus the extra-hit cadence state implied by
/// the immediately preceding frame. Without `previous_tick`, cadence timestamps
/// from an unrelated sampled tick would survive the restore.
pub fn set_state_to_record_tick(
    arena: &mut Arena,
    car_idcs: &[usize],
    tick: &TickRecord,
    previous_tick: Option<&TickRecord>,
    car_controls: &[CarControls],
) {
    for (i, &car_idx) in car_idcs.iter().enumerate() {
        let mut cs = *arena.get_car_state(car_idx);
        let rep_cs: CarState = tick.car_records[i].into();
        // ── core physics ──
        cs.phys = rep_cs.phys;
        // ── jump/flip state machine ──
        cs.is_on_ground = rep_cs.is_on_ground;
        cs.is_jumping = rep_cs.is_jumping;
        cs.is_flipping = rep_cs.is_flipping;
        cs.jump_ticks = rep_cs.jump_ticks;
        cs.flip_time = rep_cs.flip_time;
        cs.has_jumped = rep_cs.has_jumped;
        cs.has_double_jumped = rep_cs.has_double_jumped;
        cs.has_flipped = rep_cs.has_flipped;
        cs.flip_rel_torque = rep_cs.flip_rel_torque;
        // ── controls ──
        cs.prev_controls = rep_cs.prev_controls;
        // ── v2 fields restored from recording ──
        cs.boost = rep_cs.boost;
        cs.is_boosting = rep_cs.is_boosting;
        cs.is_supersonic = rep_cs.is_supersonic;
        cs.handbrake_val = rep_cs.handbrake_val;
        cs.is_demoed = rep_cs.is_demoed;
        cs.demo_respawn_timer = rep_cs.demo_respawn_timer;
        cs.air_time = rep_cs.air_time;
        cs.air_time_since_jump = rep_cs.air_time_since_jump;
        // ── bump cooldown ──
        // RL's per-victim bump cooldown is not in the recording, so it cannot be
        // restored. Left alone it survives the restore and carries over from
        // whatever the arena did on previous steps, which made every bump
        // measurement depend on how many steps had run before it: in
        // `car_car_basic_bump` the bump fired in the first impulse window, then
        // the 0.25 s cooldown suppressed it for the next 30 steps, so the second
        // window — measuring the other car of the same event — saw the physical
        // response alone and read 274 UU/s where the game shows 1302. Clearing
        // it makes every restored step start from the same state.
        cs.bump_cooldown_timer = 0.0;
        cs.bump_other_car_id = usize::MAX;

        // The observer tracks is_flipping until landing; the sim ends it
        // after TORQUE_TIME. If the recording has is_flipping=true with
        // flip_time=0 (free-flight frame), push it past TORQUE_TIME so
        // the sim doesn't re-apply flip torque.
        if cs.is_flipping && cs.flip_time == 0.0 {
            cs.flip_time = rocketsim::consts::car::flip::TORQUE_TIME;
        }

        // The observer only sets is_flipping on the FIRST flip tick and maps
        // double_jumped_or_flipped to has_flipped only while is_flipping, so
        // the sim's flip state collapses after tick 1 and it never applies the
        // flip's z-damping (flip_time in [Z_DAMP_START, Z_DAMP_END]) — the
        // post-flip vertical-velocity then diverges. Reconstruct the
        // persistent flip state: a flip (vs a double jump) has flip_time
        // incrementing past 0, and a flip still IN PROGRESS has a real
        // rotation (the car is tumbling). A finished flip (e.g. a wall-jump
        // that flies straight up, ang_vel≈0) must NOT be re-activated.
        let raw_double_jumped_or_flipped = tick.car_records[i].double_jumped_or_flipped;
        if raw_double_jumped_or_flipped && cs.flip_time > 0.0 {
            // A fresh flip tumbles the car hard around a horizontal axis
            // (ang_vel's world x/y components dominate, magnitude ~2-5.5).
            // A later tumble (auto-roll landing, flip afterglow) or a
            // yaw/tornado spin must NOT be re-activated.
            let av = tick.car_records[i].phys.ang_vel;
            let tumble = (av.x * av.x + av.y * av.y).sqrt();
            let hard_tumble = tumble > 2.0;
            // Only within the flip's active phase (through the z-damp window);
            // beyond that the flip is over and has_flipped would wrongly lock
            // pitch air-control (auto_roll, flip afterglow).
            let active = cs.flip_time <= rocketsim::consts::car::flip::Z_DAMP_END;
            if hard_tumble && active {
                cs.has_flipped = true;
                cs.has_double_jumped = false;
                // Re-open is_flipping only inside the z-damp window, and only
                // for a flip still turning at essentially the car's angular
                // speed cap. `is_flipping` makes the sim multiply `lin_vel.z` by
                // `1 - Z_DAMP_120` = 0.65 *every tick* it is set, which is a 35%
                // cut of vertical velocity, so a false positive here is worth
                // hundreds of uu/s.
                //
                // `RLFLIP=1` measures Rocket League's own decision on the closed
                // vertical axis (an isolated airborne car's vz is gravity plus
                // boost plus this damp and nothing else). Inside the window,
                // restricted to steps where the two hypotheses are separated by
                // more than 20 uu/s so the classification is meaningful, RL
                // damps 38% of 2476 samples -- while the previous gate admitted
                // essentially all of them. Splitting by tumble is what separates
                // them, and the right statistic is *saturation of the angular
                // speed cap*. A flip's dodge torque (`TORQUE_X` 260 /
                // `TORQUE_Y` 224) pins angular speed at `car::MAX_ANG_SPEED` =
                // 5.5 for the whole dodge; air roll, on weaker torque (130/95/
                // 400) against `air_control::DAMPING`, settles below it. So the
                // test is "is the car spinning at the cap", not "is it spinning
                // hard":
                //
                // ```text
                // |ang_vel| >= 5.4999  ->  94.5% damped (n=731)
                //             5.499    ->  84.1%    (n=88)
                //             5.49     ->  86.2%    (n=80)
                //             5.45     ->  28.6%    (n=14)
                //             5.0      ->  17.3%   (n=127)
                //             below    ->   4.9% (n=1436)
                // ```
                //
                // Cost of the damp term over those samples: 330 414 uu/s if we
                // damp whenever the old gate allowed it, 47 429 if we never damp
                // at all, 24 237 gating on `|ang_vel.xy| >= 5.0`, and **16 590**
                // with this saturation test.
                //
                // Use the *full* angular speed, not the world-frame `xy`
                // magnitude the old gate used: they differ by exactly the
                // vertical spin component, so `|ang_vel.xy|` under-reads any
                // flip that carries yaw and misses it. Swapping to the full
                // magnitude is most of the gain here.
                //
                // The cost has a real minimum rather than running to the cap --
                // 18 407 at 5.44, 17 131 at 5.48, 16 590 at 5.49, then back up
                // to 17 224 at 5.495 and 21 531 at 5.4999 as genuine flips
                // sitting a hair under the cap start being excluded.
                //
                // The suite agrees independently, which is the real check on the
                // epsilon: means 2.5956 at 0.05, 2.5951 at 0.02, 2.5953 at 0.01,
                // then 2.5983 at 0.005 and 2.6119 at 0.001.
                //
                // The old `0.0 < up.z < 0.9` condition is gone because it earns
                // nothing once the spin is gated and it was not what the game
                // keys off. Do not reinstate it without measuring. Aligning the
                // spin with the recorded `flip_rel_torque` axis was also tried
                // and is not worth its complexity: as a refinement on top of
                // this it makes no difference, and on its own it is worse
                // (`|ang_vel . axis| >= 5.2` costs 26 153).
                //
                // Nor should the window be widened. It is tempting, since the
                // sim's own rule also damps a falling flipper out to
                // `TORQUE_TIME`, but RL does not: past `Z_DAMP_END` it damps 17
                // of 846 sharp falling samples. Widening it measured +3.56% on
                // the suite.
                const FLIP_DAMP_ANG_SPEED: f32 = rocketsim::consts::car::MAX_ANG_SPEED - 0.01;
                let ang_speed = (av.x * av.x + av.y * av.y + av.z * av.z).sqrt();
                let zd = rocketsim::consts::car::flip::Z_DAMP_START
                    ..=rocketsim::consts::car::flip::Z_DAMP_END;
                if zd.contains(&cs.flip_time) && ang_speed >= FLIP_DAMP_ANG_SPEED {
                    cs.is_flipping = true;
                }
            }
        }

        // Same class of bug for the jump: the RLPR observer reports
        // is_jumping=true + jump_ticks=0, but the velocity field tells us whether
        // the immediate force was ALREADY applied:
        //   * SOLO recordings: the state carries the post-impulse velocity
        //     (vz ≈ 295 UU/s) — restoring jump_ticks=0 would make `update_jump`
        //     re-apply the immediate force, doubling the launch (vz ≈ 590).
        //   * MATCH recordings: the state still has vz ≈ 0 (grounded) — the
        //     impulse is applied DURING this tick, so the sim MUST apply it.
        // Decide from the vertical velocity instead of bumping unconditionally.
        if cs.is_jumping && cs.jump_ticks == 0 && cs.phys.vel.z > 100.0 {
            cs.jump_ticks = 1;
        }

        // Derived from `is_on_ground` rather than from the recording, even
        // though `WheelRecord::has_contact` does carry the real per-wheel flags
        // (an earlier comment here claimed otherwise). Either way the restore is
        // inert: `pre_tick_update` recomputes `wheels_with_contact` and
        // `is_on_ground` from the raycast unconditionally before they are read.
        // It is kept only so the restored state is self-consistent for anything
        // that inspects it before the first step. The recorded flags are
        // compared against the sim as the `wheels_contact` state channel.
        cs.wheels_with_contact = if cs.is_on_ground {
            [true; 4]
        } else {
            [false; 4]
        };

        // The RLPR observer writes air_time_since_jump=0 on every tick (the
        // field is broken/unpopulated), so the old forced 2.0 permanently
        // closed the double-jump/flip window and the sim missed every airborne
        // flip in match replays. Reconstruct it instead: the recording's
        // air_time equals time-since-jump-start while airborne, and the jump's
        // active phase lasts at most jump::MAX_TICKS, so
        // air_time_since_jump ≈ air_time - MAX_TICKS / TICK_RATE. Only jumpers can flip —
        // an airborne car that never jumped keeps the window closed.
        if cs.air_time_since_jump == 0.0 {
            cs.air_time_since_jump = if cs.has_jumped {
                (cs.air_time
                    - rocketsim::consts::car::jump::MAX_TICKS as f32 * rocketsim::consts::TICK_TIME)
                    .max(0.0)
            } else {
                2.0 // > DOUBLEJUMP_MAX_DELAY (1.25)
            };
        }

        // Reset internal fields the recording does NOT capture to defaults so
        // each tick is truly isolated and reproducible. Without this, residual
        // values leak from the previous tick (or from a shard's fresh arena),
        // making single- and multi-threaded runs disagree.
        cs.world_contact_normal = None;
        cs.supersonic_grace_timer = 0.0;
        cs.boosting_time = 0.0;
        cs.time_since_boosted = 0.0;
        cs.is_auto_flipping = false;
        cs.auto_flip_timer = 0.0;
        cs.auto_flip_torque_scale = 0.0;

        cs.controls = car_controls[i];

        arena.set_car_state(car_idx, cs);
    }

    let rep_bs_phys: PhysState = tick.ball_record.into();
    let mut bs = *arena.get_ball_state();
    bs.phys = rep_bs_phys;
    let previous_arena_tick = arena.tick_count().saturating_sub(1);
    bs.ball_hit = previous_tick.map_or(BallHitState::DEFAULT, |previous| {
        let contact_slack = std::env::var("RL_CONTACT_STATE_SLACK")
            .map(|value| {
                value
                    .parse::<f32>()
                    .expect("RL_CONTACT_STATE_SLACK must be a finite number")
            })
            .unwrap_or(0.0);
        assert!(
            contact_slack.is_finite() && contact_slack >= 0.0,
            "RL_CONTACT_STATE_SLACK must be a finite non-negative number"
        );
        let ball_pos = glam::Vec3A::from(previous.ball_record.pos);
        let was_touching = previous.car_records.iter().any(|car| {
            super::residual::ball_touches_car_with_slack(
                ball_pos,
                car.phys.pos.into(),
                glam::Mat3A::from_cols(
                    car.phys.rot.rows[0].into(),
                    car.phys.rot.rows[1].into(),
                    car.phys.rot.rows[2].into(),
                ),
                contact_slack,
            )
        });
        BallHitState {
            last_impulse_tick: previous
                .car_records
                .iter()
                .any(|car| car.hit.has_hit)
                .then_some(previous_arena_tick),
            last_contact_tick: was_touching.then_some(previous_arena_tick),
        }
    });
    arena.set_ball_state(bs);
}

/// Upper loop bound shared by both modes.
pub fn loop_bound(recording: &Recording, max_ticks: Option<usize>) -> usize {
    let stride = recording.stride;
    let raw_max = recording.ticks.len().saturating_sub(stride + 1);
    let n = max_ticks.map_or(raw_max, |m| m.min(raw_max));
    (n / stride) * stride
}

fn controls_for(to_tick: &TickRecord, buf: &mut Vec<CarControls>) {
    for (j, car_record) in to_tick.car_records.iter().enumerate() {
        buf[j] = car_record.prev_controls.into();
    }
}

/// Per-tick restore worker for one contiguous shard of start-tick indices.
///
/// Owns its own arena so shards can run on separate threads. Per-tick mode
/// restores the full recorded state every tick, so ticks are independent —
/// that is what makes the pass embarrassingly parallel.
fn run_per_tick_shard(recording: &Recording, shard: &[usize]) -> Report {
    let num_cars = recording.info.num_cars as usize;
    let stride = recording.stride;
    let (mut arena, car_idcs) = make_arena(num_cars);
    let mut report = Report::new(recording.name.clone(), stride, num_cars, shard.len());
    let mut controls_buf: Vec<CarControls> = vec![CarControls::DEFAULT; num_cars];

    // Warm the cold arena up so Bullet's contact manifolds settle before we
    // measure anything. Re-step the shard's first usable tick and discard the
    // results: state is fully restored every cycle, so repeating one tick
    // settles the manifolds at the very geometry we are about to measure,
    // without consuming (and thereby hiding) any tick in the shard's range.
    if let Some(&first) = shard
        .iter()
        .find(|&&i| !has_discontinuity(recording, i, stride))
    {
        let from_tick = &recording.ticks[first];
        let to_tick = &recording.ticks[first + stride];
        controls_for(to_tick, &mut controls_buf);
        for _ in 0..SHARD_WARMUP_TICKS {
            set_state_to_record_tick(
                &mut arena,
                &car_idcs,
                from_tick,
                first.checked_sub(stride).map(|i| &recording.ticks[i]),
                &controls_buf,
            );
            arena.step_tick();
        }
    }

    for &i in shard {
        // Void non-physical recording discontinuities (goal resets, demos,
        // respawns, logger car-index swaps): a physics step cannot reproduce a
        // teleport, so measuring it would fabricate divergence.
        if has_discontinuity(recording, i, stride) {
            report.ticks_voided += 1;
            continue;
        }

        let from_tick = &recording.ticks[i];
        let to_tick = &recording.ticks[i + stride];
        controls_for(to_tick, &mut controls_buf);

        set_state_to_record_tick(
            &mut arena,
            &car_idcs,
            from_tick,
            i.checked_sub(stride).map(|i| &recording.ticks[i]),
            &controls_buf,
        );

        // Deepest the ball is inside the mesh at the pose we just restored, and
        // how fast it is moving into that surface. Read off the step the shard
        // takes anyway, so this costs nothing.
        let ball_pos_before: glam::Vec3A = from_tick.ball_record.pos.into();
        let ball_vel_before: glam::Vec3A = from_tick.ball_record.lin_vel.into();
        let mut deepest_penetration = f32::NEG_INFINITY;
        let mut ball_approach_speed = 0.0f32;
        for event in arena.step_tick() {
            if let rocketsim::ArenaEvent::BallHitWorld(hit) = event {
                let depth = BALL_RADIUS - (hit.contact_point - ball_pos_before).length();
                if depth > deepest_penetration {
                    deepest_penetration = depth;
                    ball_approach_speed = -ball_vel_before.dot(hit.contact_normal);
                }
            }
        }

        for (j, &car_idx) in car_idcs.iter().enumerate() {
            let real = &to_tick.car_records[j];
            // A demoed car is parked/absent (the logger parks it at origin);
            // measuring it against the sim fabricates divergence.
            if real.is_demoed || is_car_sentinel(&real.phys) {
                continue;
            }
            // A step straddling a jump/flip press carries an arbitrary share of
            // the impulse, set by a sub-frame phase the recording never stored,
            // so a single-step comparison here measures the phase and not the
            // physics. The impulse itself is still gated, over the two-step
            // window where that phase cancels (see `impulse_window.rs`).
            if recording.step_straddles_impulse(i, stride, j) {
                report.impulse_steps_skipped += 1;
                continue;
            }
            let cs: CarState = *arena.get_car_state(car_idx);
            let from_car = &from_tick.car_records[j];
            let delta = compute_delta(&cs.phys, &real.phys);

            let ent = &mut report.entities[j];
            for field in Field::ALL {
                let s = *delta.get(field);
                ent.field_mut(field).add(s.mag, s.signed, i, Some(from_car));
            }
            ent.state.record(&cs, real);
        }

        // Skip the ball when it is parked at the "absent" sentinel; comparing
        // against it would fabricate divergence (see is_ball_sentinel).
        if is_ball_sentinel(&to_tick.ball_record) {
            // Nothing to file, and nothing corrupt either.
        } else if ball_record_impossible(from_tick, to_tick, stride) {
            report.ball_steps_voided += 1;
        } else {
            if ball_embedded_in_mesh(
                deepest_penetration,
                ball_approach_speed,
                rocketsim::consts::TICK_TIME * stride as f32,
            ) {
                report.ball_steps_embedded += 1;
            }
            let ball_state: &BallState = arena.get_ball_state();
            let ball_delta = compute_delta(&ball_state.phys, &to_tick.ball_record);
            let ball_ent = &mut report.entities[report.num_cars];
            for field in Field::ALL {
                let s = *ball_delta.get(field);
                ball_ent.field_mut(field).add(s.mag, s.signed, i, None);
            }
        }

        report.ticks_measured += 1;
    }

    report
}

/// Per-tick restore mode, sharded across `cfg.threads` worker threads.
///
/// Fills a [`Report`] with lean physics deltas + state mismatch counts. This
/// is the gated mode. Each worker owns an arena and a tick shard; results are
/// merged at the end.
pub fn run_per_tick(recording: &Recording, cfg: &HarnessConfig) -> Report {
    let stride = recording.stride;
    let n = loop_bound(recording, None);
    let num_cars = recording.info.num_cars as usize;
    let starts: Vec<usize> = (0..=n).step_by(stride).collect();

    if starts.is_empty() {
        return Report::new(recording.name.clone(), stride, num_cars, 0);
    }

    // Not worth sharding tiny recordings: thread + per-arena setup overhead
    // exceeds the stepping cost. Parallelize only above a tick threshold.
    const PARALLEL_MIN_TICKS: usize = 1024;
    let threads = if starts.len() < PARALLEL_MIN_TICKS {
        1
    } else {
        cfg.threads.clamp(1, starts.len())
    };
    if threads <= 1 {
        let mut single = run_per_tick_shard(recording, &starts);
        reject_embedded_ball_ground_truth(&mut single);
        return single;
    }

    let mut merged = Report::new(recording.name.clone(), stride, num_cars, 0);
    let chunk = starts.len().div_ceil(threads);
    std::thread::scope(|s| {
        let handles: Vec<_> = starts
            .chunks(chunk)
            .map(|shard| s.spawn(move || run_per_tick_shard(recording, shard)))
            .collect();
        for handle in handles {
            merged.merge(handle.join().expect("per-tick shard panicked"));
        }
    });
    reject_embedded_ball_ground_truth(&mut merged);
    merged
}

/// Summary of free-running divergence (informational, not gated).
#[derive(Debug, Clone, Copy, Default)]
pub struct ContinuousSummary {
    pub ticks: u64,
    pub max_pos: f32,
    pub max_pos_tick: usize,
    pub max_vel: f32,
    pub max_vel_tick: usize,
    /// Final position error broken into x/y/z (the compounding direction).
    pub final_pos_err: glam::Vec3A,
    /// Ball divergence (max pos / vel / tick), when the recording has a real
    /// ball (not the parked sentinel).
    pub ball_max_pos: f32,
    pub ball_max_pos_tick: usize,
    pub ball_max_vel: f32,
    pub ball_max_vel_tick: usize,
}

/// Continuous mode: state set once at tick 0, then runs freely. Tracks the
/// focused car's (or car 0's) divergence growth.
pub fn run_continuous(
    recording: &Recording,
    arena: &mut Arena,
    car_idcs: &[usize],
    cfg: &HarnessConfig,
) -> ContinuousSummary {
    let stride = recording.stride;
    let n = loop_bound(recording, None);
    let num_cars = car_idcs.len();
    let mut controls_buf: Vec<CarControls> = vec![CarControls::DEFAULT; num_cars];
    let focus = cfg.car_focus.unwrap_or(0).min(num_cars.saturating_sub(1));

    let mut summary = ContinuousSummary::default();

    for i in (0..=n).step_by(stride) {
        let to_tick = &recording.ticks[i + stride];
        controls_for(to_tick, &mut controls_buf);

        if i == 0 {
            let from_tick = &recording.ticks[i];
            set_state_to_record_tick(arena, car_idcs, from_tick, None, &controls_buf);
        } else {
            for (j, &car_idx) in car_idcs.iter().enumerate() {
                arena.set_car_controls(car_idx, controls_buf[j]);
            }
        }
        arena.step_tick();

        let cs: CarState = *arena.get_car_state(car_idcs[focus]);
        let real = &to_tick.car_records[focus];
        let pos_delta = (cs.phys.pos - glam::Vec3A::from(real.phys.pos)).length();
        let vel_delta = (cs.phys.vel - glam::Vec3A::from(real.phys.lin_vel)).length();
        if pos_delta > summary.max_pos {
            summary.max_pos = pos_delta;
            summary.max_pos_tick = i;
        }
        if vel_delta > summary.max_vel {
            summary.max_vel = vel_delta;
            summary.max_vel_tick = i;
        }
        summary.final_pos_err = cs.phys.pos - glam::Vec3A::from(real.phys.pos);
        summary.ticks += 1;

        let ball_real = &to_tick.ball_record;
        if !is_ball_sentinel(ball_real) {
            let bs: BallState = *arena.get_ball_state();
            let b_pos_delta = (bs.phys.pos - glam::Vec3A::from(ball_real.pos)).length();
            let b_vel_delta = (bs.phys.vel - glam::Vec3A::from(ball_real.lin_vel)).length();
            if b_pos_delta > summary.ball_max_pos {
                summary.ball_max_pos = b_pos_delta;
                summary.ball_max_pos_tick = i;
            }
            if b_vel_delta > summary.ball_max_vel {
                summary.ball_max_vel = b_vel_delta;
                summary.ball_max_vel_tick = i;
            }
        }
    }

    summary
}
