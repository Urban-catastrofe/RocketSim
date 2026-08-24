# How to Run RL Comparison Tests

The harness replays recorded Rocket League ground-truth (`.rlpr`) through the
sim and measures **per-tick physics divergence**. It is built to be driven by a
human or an LLM chasing accuracy: survey the numbers, find the worst offender,
deep-dive it, fix the sim, watch the number drop.

> The harness **auto-detects the tick rate and hard-rejects** anything that is
> not 120 Hz (stride ≠ 1) with an error, because a 240 Hz source is too
> unreliable to sample for rocketsim tests. The current recording set is 120 Hz
> (`stride=1`), so it is accepted.

> ⚠️ `rocketsim/collision_meshes` is a symlink to an out-of-tree asset folder.
> On a checkout with `core.symlinks=false` (Windows default) it lands as a
> 42-byte text file and every case panics with
> `./collision_meshes/ does not exist`. Replace it with a real directory (or a
> junction) holding `soccar/*.cmf` before running anything.
>
> Beware that a plain `git stash` restores the committed symlink stub and
> **deletes the real directory you put there**, so every case silently panics on
> the next run. Scope the stash to source (`git stash push -- rocketsim/src
> rocketsim/tests`) when comparing before/after, and re-check
> `collision_meshes/soccar` afterwards.

## Residual Analysis (RLRESID)

The **residual-force decomposition** (`RLRESID=1|2`) splits each tick's error
into the *game's* per-tick Δvel vs the *sim's*, binned by regime (ground/air,
boost, wall) and by ball contact. A large mean residual in one bin = a wrong
force/impulse for that subsystem. Trace mode (`RLRESID=2`) prints per-tick
Δvel for the ball while it touches a car — the impulse *pattern* (every tick vs
every other tick) is directly visible.

```bash
# Residual decomposition for one case:
RLRESID=1 cargo test -p rocketsim case_car_ball_soft_touch -- --nocapture --test-threads=1

# Per-tick ball Δv trace during contact:
RLRESID=2 cargo test -p rocketsim case_car_ball_soft_touch -- --nocapture --test-threads=1
```

## Hit-Cadence Analysis (RLRESID=3) — v3 recordings only

`RLRESID=3` resolves the **same-tick vs delayed impulse** question. It needs the
v3 hit records (the game's `OnHitBall` events, which capture the ball velocity
*before* the impulse). Comparing `ball_vel_before` against the tick-end ball
velocity (same tick vs next tick) shows which tick carries the impulse — the
ground truth the calibration loop needs.

```bash
RLRESID=3 cargo test -p rocketsim case_car_ball_soft_touch -- --nocapture --test-threads=1
```

Legacy v2 recordings have no hit records and report no events in these modes;
the parser remains version-aware, so they still work in the regular harness.

## Hit-Episode Analysis (RLRESID=7) — v3 recordings only

`RLRESID=7` measures complete car-ball contact episodes rather than individual
ticks. It restores two ticks before the first `OnHitBall` event, free-runs
through the geometrical contact and one delayed frame, then compares the game's
and sim's total ball velocity change. Episodes without two lead-in frames are
reported as unmeasurable because their cadence and contact phase cannot be
reconstructed safely. Each episode also reports the sim's callback ticks,
extra-impulse ticks, geometric gap, and center-velocity closing speed.

```bash
RLRESID=7 cargo test -p rocketsim case_car_ball_pop_stationary -- --nocapture --test-threads=1
```

## Hit-Trigger Audit (RLRESID=11) - v3 recordings only

`RLRESID=11` compares the game's `OnHitBall` labels with the sim's car-ball
callbacks, actual extra-impulse firings, and post-solver contacts. It reports
exact-tick precision and recall, plus episode recall with one frame of phase
tolerance. The latter is the meaningful trigger metric because high-speed
impacts commonly occur between two recorded frames. The `SOLVED CONTACT` row
adds the solver normal impulse, pre/post relative normal velocity, manifold
distance, and RL-vs-sim contact normal/location error. `SWEPT CONTACT` audits
continuous sphere-vs-car-box gaps at 0/1/2/3 UU thresholds. Set
`RLTRIGGER_VERBOSE=1` to print each episode's game geometry, sim contact/fire
ticks, and `(tick, distance, impulse, pre_n, post_n)` solver tuples.

```bash
RLRESID=11 RLTRIGGER_VERBOSE=1 cargo test -p rocketsim case_car_ball_soft_touch -- --nocapture --test-threads=1
```

Harness-only overrides support controlled comparisons without changing
production defaults:

```bash
RL_HIT_CADENCE=other RLRESID=7 cargo test -p rocketsim case_car_ball_soft_touch -- --nocapture --test-threads=1
RL_BALL_HIT_SCALE=0 RLRESID=7 cargo test -p rocketsim case_car_ball_soft_touch -- --nocapture --test-threads=1
RL_CONTACT_STATE_SLACK=1.825 RLGATE=off cargo test -p rocketsim case_car_ball_soft_touch -- --nocapture --test-threads=1
```

## The Car-Ball Impulse Subsystem

The extra hit impulse lives in `rocketsim/src/sim/ball_hit/` as a pure,
config-driven module:

- `config.rs` — `BallHitConfig` (all tunables: z_scale, forward_scale, factor
  curve, max_delta_vel) + `HitCadence` enum (`EveryTick` / `EveryOtherTick` /
  `OncePerEpisode`). The production default is `OncePerEpisode`; legacy C++
  cadence is available as `EveryOtherTick` for calibration.
- `impulse.rs` — `compute_impulse` (pure) + `can_fire` (cadence check).
- `state.rs` — `BallHitState` (last impulse tick, last contact tick).

The impulse is a *pure function of the config* and the hit geometry, so the
residual tool can fit the constants: change one config value, re-run the 3v3
recordings, watch the ball vel p95 move.

**Impulse application (2026-08):** the extra impulse is now applied at the
**end of the contact tick** in `finish_physics_tick` (matching C++
`_FinishPhysicsTick`). Previously it was queued to `pre_tick_update` of the
next tick, which the per-tick restore harness wiped before it ever fired — the
sim's car-ball hits were permanently missing the carried impulse. The default
cadence is `OncePerEpisode` (one impulse per contact, re-armed after the ball
separates). The game often spreads a hit over **2 ticks**, and some three-frame
contact episodes need a second impulse after one skipped frame. For example,
`EveryOtherTick` reduces complete soft-touch episode error from 51.8 to 0.2
UU/s. It cannot be enabled globally: in paired 0.5-second rollouts it improves
soft-touch ball velocity error from 103.10 to 0.14 UU/s but regresses slow push
from 2.90 to 38.93 UU/s. Contact-normal, car-to-ball centerline, and relative-
speed gates do not separate those regimes safely. Delaying that cadence also
produces 60-80 UU/s drift in roof, push, dribble, and pop cases. Per-tick cases dominated by the split
(`car_ball_backwall_car_ball_approach`, `mech_flip_reset_simple`) score
slightly worse per-tick even though the *total* impulse matches the game. The
2-tick split is **not safely modelable by a global cadence or delay** (verified
2026-08): in the per-tick restore harness the 2nd half double-counts (the
restore already contains the game's recorded 2nd-half velocity), while in a
continuous rollout changing the application tick changes contact duration and
can trigger additional impulses. A correct model needs a stronger event-ending
signal than geometric contact, gap, or center-velocity closing speed.

Post-solver observations confirm that a positive Bullet normal impulse is not
the missing event-ending signal: Rocket League can emit `OnHitBall` on cached or
separating contact frames where the sim's normal impulse is zero. The matched
manifold geometry is generally close (roughly 0.9-4 UU location error), while
the exposed sim normal uses the opposite convention from the recording
(`normal_dot = -1`). A global contact-margin increase from 0.02 to 0.03 improves
recall but starts contacts early and loses too much precision, so 0.02 remains
the production threshold.

### Car-Ball Contact Lifetime Deep Dive

Measured 2026-08-24. The apparent lifetime mismatch is primarily a **contact
phase** problem, not a missing manifold-age rule:

- This port does not persist manifold points across ticks. Each narrowphase pass
  builds a new `PersistentManifold`, and the solver clears the manifold list
  after setup. There is no point age or lifetime field to tune.
- The added-contact callback runs before `refresh_contact_points`; Arena records
  it immediately. A point removed by refresh can therefore reach
  `Ball::on_hit` without producing a `contact_solved` observation. The focused
  soft/push episodes did not rely on that discrepancy: their recorded callback
  points also reached the solver.
- Three-frame episodes end with a separating, zero-normal-impulse contact. That
  frame is still a valid Rocket League `OnHitBall` frame, confirming again that
  solver impulse cannot define event lifetime.

The decisive comparison is geometry at onset. In slow-push episode 86, Rocket
League's car-ball gap moves `0.742 -> -0.223` on the first event frame, while
the discrete sim still solves at a positive `0.741` gap. Its first regular plus
extra response is close in velocity, but applying it for the whole frame leaves
the ball roughly 1 UU farther from the car. By the third frame the sim reaches
`2.97` UU and drops contact, while Rocket League remains at `1.37` UU and emits
the second every-other extra impulse. Soft touch is different: it already
overlaps before onset, and sim/game gaps stay within about 0.1 UU through the
episode, so the third callback survives and `EveryOtherTick` reproduces the
total impulse.

Two blunt TOI approximations were tested and removed:

- Two global half-steps improved slow-push exact contact recall from 0.684 to
  0.895, but solved existing contacts twice and worsened episode response.
- Zero contact margin plus four substeps delayed onset into penetration but
  repeatedly solved deep overlap, badly regressing both soft touch and slow
  push. Merely changing margin or substep count is not a valid fix.

A **one-shot car-ball time-of-impact solve** is now implemented for spherical
balls. Approaching sphere/OBB pairs that cross actual touching during the frame
are withheld from start-of-frame dispatch, advanced to the crossing fraction,
generated/solved once, and then integrated through the remainder. Existing
start-of-frame contacts stay on the normal discrete path. The initial solver
consumes gravity and control velocity once; later TOI solves see only contact
velocity changes. Collision filters, disabled contact response, multiple cars,
and impacts redirected by earlier solves are preserved. Snowday's convex puck
continues to use the discrete path.

The focused result is useful but mixed. Slow-push episode 86 now keeps contacts
on ticks `[86, 87, 88]` instead of `[86, 87]`, and its third-frame gap falls from
`2.97` to `1.45` UU. Across all slow-push episodes, however, mean impulse gap is
effectively flat (`28.6 -> 28.5 UU/s`). At 0.5 seconds, mean slow-push ball
velocity error improves `41.22 -> 40.37 UU/s` while mean position error moves
`6.45 -> 6.61 UU`; soft-touch velocity improves `18.43 -> 18.25 UU/s` while
position moves `5.84 -> 6.25 UU`. The full 452-case comparison suite still
passes. Treat TOI as a phase-correct collision primitive, not a replacement for
the unresolved multi-frame `OnHitBall` cadence model.

## Ground-Truth Validation

A recording is the *reference* the sim is graded against, so an impossible one
does not add noise — it inverts the metric, penalising the sim for being right.
`validate.rs` rejects such recordings before measurement, and the case fails with
`UNUSABLE GROUND TRUTH` naming the offending car.

The check in place: RL launches a grounded car at ~300 UU/s along its own up-axis
on the tick jump is pressed, so a grounded press producing no up-axis velocity
within 3 ticks did not happen in physics. A recording is rejected only when
*every* observable press fails — one failure can be legitimate (a car wedged
under the ball), but none of them ever launching is a pinned car. The up-axis
(not world Z) is used so wall, ceiling and upside-down cars are judged by the
direction they would actually launch.

**44 of the original 468 recordings were quarantined to
`test_recordings_invalid/` by this check (2026-08).** They hold jump while
flagged `is_on_ground`, set `has_jumped`, count `jump_time` up — and never move
the car (peak up-axis velocity 0.1–0.5 UU/s). The cars were pinned by a
state-set loop while recording: the inputs reached the state machine but the
physics was overridden. They made flips the worst-scoring mechanism in the suite
(sideflip mean 15.3 UU/s) by charging the sim 300–450 UU/s for correctly
launching a car the recording holds still, and five of them were *passing* the
gate while verifying nothing about jumping. Re-record them before restoring any.

Set `RLNOVALIDATE=1` to bypass the check while investigating a suspect
recording.

## Observer-Semantics Normalisation

The RLPR observer and the sim record some state-machine flags with genuinely
different *meanings*. Restoring such a field verbatim does not add noise: it
drives the sim into a state the recording never described, so the divergence
that gets measured is the harness's fault, not the physics'.
`recording/normalize.rs` translates them once at load time, so the gate, the
residual decomposition, rollouts, deep dives and the C++ side-by-side all agree.

**`is_jumping` is an activation pulse, not a sustained flag.** The observer sets
it for exactly one tick — every one of the suite's 1626 `is_jumping` runs has
length 1 — then clears it while `jump_time` keeps counting and the car keeps
accelerating upward. The sim instead holds it for as long as the jump produces
thrust and applies `jump::ACCEL` only while it is set, so restoring the pulse
verbatim switched the sustained jump accel off on every tick of an ascent but
the first, losing a flat `jump::ACCEL * TICK_TIME` ≈ 12.15 UU/s per tick. That
was visible as a dead-constant 12.167 UU/s error on the interior ticks of every
jump; after normalising, those ticks read 0.05 UU/s.

Two details are load-bearing:

- It must be a **whole-recording** pass. Distinguishing "this jump is still
  running" from "the button was pressed again while `jump_time` happens to still
  be small" needs the control history; a rule using only the current tick
  disagrees with the replayed truth on 6086 ticks of the suite.
- Only a **grounded** pulse arms the flag. The observer also pulses `is_jumping`
  on an airborne *double* jump (71 of the 1626), but RL only ever starts a
  sustained jump from the ground — a double jump is impulse-only — so arming
  there invents thrust neither the game nor the sim has.

The activation tick stays recoverable as `is_jumping && jump_time == 0.0`, which
is how the immediate-force guards identify it.

## Impulse Onset Ticks Cannot Be Gated Per-Tick

Jump and flip impulses are applied by RL at the moment of the button press,
which is **not** frame-aligned: the press lands at some sub-frame phase φ, and
the impulse is split across the two recorded frames that straddle it, in
proportion (1−φ):φ. φ is not recorded anywhere.

The evidence is unambiguous. `physics_frame` increments by exactly 1 per
recorded tick (uniform dt, no dropped frames) and each tick is self-consistent
(Δpos = vel/120), yet the vertical velocity on the jump-onset tick ranges from
**15.2 to 295.6 UU/s across recordings of the identical mechanic** — while the
*total* over the two-frame window is always the full `jump::IMMEDIATE_FORCE`
of 291.67 UU/s. Same story for flips: `car_sideflip_while_turning_left` splits
one 521 UU/s dodge impulse 14%/86% across ticks 23 and 24.

Consequence: **the two steps straddling every jump, double jump and flip press
can never pass a 0.03 UU/s per-tick gate**, no matter how correct the physics is.
The sim fires the whole impulse on the step where the rising edge appears; the
recording spread it over two.

The harness therefore splits the measurement in two:

- The **per-tick pass skips those two steps** (per car), counting them as
  `impulse_steps_skipped=N` on the report header so the blind spot is visible
  rather than silent. `recording/normalize.rs::detect_impulse_onsets` marks the
  onsets from the observer's `is_jumping` / `is_flipping` pulses.
- **`impulse_window.rs` gates the impulse over the two-step window** `onset−1 →
  onset+1`, where φ divides out because the sum across the pair is one whole
  impulse regardless of how it was split. The sim is restored once, at
  `onset−1`, then free-runs both steps on the recorded controls — restoring in
  between would re-impose the recording's arbitrary split. Budget:
  `RL_IMPULSE_TOL` (default 0.1 UU/s, wider than the per-step bar by roughly the
  two steps of ordinary integration error it contains). Grounded jumps that
  overlap geometrically confirmed car-ball contact include one additional tick:
  RL spreads that collision response into the frame after the jump window, so a
  two-step comparison would stop mid-event and misattribute collision phase to
  jump magnitude.

Report lines look like:

```
[case] IMPULSE jump        n=  1 mean= 25.8668 max= 25.8668@t20 over=1/1 budget=0.100
[case] IMPULSE   worst car0 jump @t20: game dv=(+4.4,-50.9,+299.7) |304.1|                  sim dv=(+6.4,-26.5,+307.8) |309.0| vel_err=25.8668 pos_err=0.1421
```

This immediately settles what the per-tick numbers could not: the **flip impulse
is essentially exact** (`car_diag_flip_drive` reads 0.0020 UU/s on a ~500 UU/s
dodge; flip windows have a median of 1.2 UU/s), so the 300–450 UU/s "flip
defect" the per-tick gate used to report was entirely φ. What the window does
show is listed under Known Impulse Defects below.

### Car-Car Bumps Are Sub-Frame Events Too

A bump lands at a sub-frame time exactly like a jump press, so the recorded tick
that carries it is not reproducible from a frame-aligned restore. There is no
flag for it — the observer records no bump event — so
`recording/normalize.rs::detect_bump_onsets` recovers the onset from Rocket
League's own position/velocity **self**-consistency instead.

On an ordinary tick the recording satisfies `pos[t] - pos[t-1] == vel[t] * dt`.
Measured over 477939 non-teleport car/ticks that residual runs p50 = 0.004,
p90 = 0.006, p99 = 0.21 UU. An impulse applied partway through a step has to
break the identity, because part of the step was travelled at the pre-impulse
velocity. On `car_car_basic_bump` the bump tick reads 1.90 UU (bumper) and
4.29 UU (victim) — two to three orders of magnitude above the p90 floor. Solving
`dpos = [v_pre*phi + v_post*(1-phi)] * dt` for the victim gives phi = 0.41.

This is the right criterion rather than a proxy: a bump that lands at phi = 0
leaves the identity intact, and that tick really is reproducible. Only provably
unmeasurable ticks get marked — 510 across the suite, 0.101% of all car/ticks, in
28 of 424 cases, so the gate is not being hollowed out.

Two confounders also break the identity and are excluded first. **Teleports**
(demo respawn, kickoff reset) are caught by a displacement bound. **Dropped
logger frames** are real and had to be found the hard way: `2v2_1` cars 1 and 3
hold `|dpos|` = 38.4 UU for 18 consecutive ticks at exactly 2300 UU/s, which is
precisely `2 * vel * dt`. They are caught by requiring `physics_frame` to advance
by exactly `stride`. Proximity to another car is required on top, so sub-frame
events from other subsystems are not swept up as bumps.

The free-run is what makes bumps measurable at all. A bump fires on geometric
contact, and Bullet detects contact from the positions at the *start* of a step:
on `car_car_basic_bump` the restored frame still leaves a 5.6 UU gap that the
step then closes, so a single restored step cannot produce the bump however
correct the impulse is. Two free-running steps let the first close the gap and
the second fire.

**Fixed alongside it: the bump cooldown leaked across restores.** RL's
per-victim 0.25 s cooldown is not in the recording, so it cannot be restored —
and `set_state_to_record_tick` used to leave it alone, letting it survive from
whatever the arena did on previous steps. Every bump measurement therefore
depended on how many steps had run before it. In `car_car_basic_bump` the bump
fired in the first impulse window, then the cooldown suppressed it for the next
30 steps, so the second window — measuring the *other* car of the same event —
saw the physical response alone and read 274 UU/s where the game shows 1302.
`bump_cooldown_timer` and `bump_other_car_id` are now cleared on restore.

#### What the bump window says

The bump magnitude curves and their input are **correct**. Decomposing
`car_car_basic_bump` by hand (closing 972.9, equal masses, perfectly-inelastic
common velocity 487.4) gives:

| quantity | ground truth | sim's own curve | error |
|---|---|---|---|
| bump forward | 768.1 | `BUMP_VEL_AMOUNT_GROUND(973)` = 764.7 | 0.4% |
| bump upward | 189.9 (gravity-corrected) | `BUMP_UPWARD_VEL_AMOUNT(973)` = 193.3 | 1.8% |
| restitution | +0.060 | `HIT_CAR_COEFS.restitution` = 0.1 | same order |

`car_car_head_on_bump` confirms it independently and pins the curve input as
`attacker.vel . dir_to_victim` rather than the closing speed: predicted 982/996
forward and 248 up against 1087/1154 and 246 actual, effective restitution
0.12–0.14.

With the cooldown leak fixed and the car-car softening removed,
`car_car_basic_bump` reads `vel_err` **6.54** (bumper) and **10.37** (victim),
against 262.69 and 1046.04 before. As with the flip, most of the apparent defect
was measurement.

#### `hit_car_phys_soften_speed` is gone

It scaled the car-car rigid-body response by `500 / |approach|`, justified by a
comment claiming RL resolves car-car almost entirely via the bump impulse and
that "the bullet inelastic response would reverse them".
`car_car_head_on_bump` shows RL going **+1267.9 -> -1153.0** and
**-1249.6 -> +1154.0**. RL *does* reverse them, so the premise was wrong.

The arithmetic was exact about it: at soften=500 the bumper in
`car_car_basic_bump` read `sim dv` −278.5 against the game's −541.2, a ratio of
0.515 — precisely `500/973`. The softening factor *was* the whole bumper-side
error. Measured on the bump window across the 17 `car_car` cases (118 events):

| case | with softening | removed |
|---|---|---|
| **weighted mean** | 531.41 | **421.28** |
| basic_bump | 263.78 | **8.46** |
| head_on_bump | 566.14 | **10.60** |
| diagonal_fast | 725.78 | **122.76** |
| aerial_bump | 770.24 | **177.00** |
| long_multi_bump | 844.28 | **385.58** |
| boost_contest | **427.58** | 832.53 |
| long_boost_headon | **466.07** | 687.18 |

Do not reintroduce it, and do not try to re-fit the constant. Splitting it into
velocity and depenetration terms does nothing — `rhs_penetration` is always zero
here because penetration stays above `SPLIT_IMPULSE_PENETRATION_THRESHOLD`, so
the two configurations are byte-identical. Sweeping the constant gives only a
shallow, contaminated optimum (per-tick total mean 245.87 @500, 239.48 @1000,
232.08 @2000, 232.51 @3000, 236.16 @off), contaminated because the per-tick
metric is dominated by the sub-frame ticks that cannot be measured at all.

#### Open defect: car-car needs continuous collision detection

The two cases that got worse are the only ones where the cars deeply
interpenetrate, and they are a detection problem rather than an impulse-magnitude
one. In `car_car_boost_contest` the closing speed is 4126 UU/s = **34.4 UU per
tick**, comparable to the hitbox depth (120.5 x 86.7 x 38.7). RL applies its
impulse at the sub-frame moment of touching and the cars pass through, ending
85 UU apart — overlapping by 33 — and separating gently at ±650 with +310 vz.
Bullet's discrete detection steps straight into deep overlap and resolves the
accumulated penetration in one go, which is where the ±2300–2400 UU/s window
errors on `boost_contest` and `long_boost_headon` come from.

This is the next thing to fix in the subsystem. The softening was hiding it, at
the cost of every clean impact.

### Known Impulse Defects

1. **Grounded jump reads ~10 UU/s too much vertical velocity.** Every jump
   window overshoots: game `dv.z` 296.4–299.7 against the sim's 307.8–307.9,
   consistently. Median jump-window error 12.75 UU/s.
2. **Lateral speed is not shed during jump-off while turning.** A turning car
   loses far more sideways velocity than the sim allows
   (`car_jump_after_turning_left`: game `dv.y` −50.9, sim −26.5;
   `car_sideflip_while_turning_left`: −70.4 vs −33.8). Consistent with the
   wheels unloading and breaking traction as the car leaves the ground, which
   ties into the slip-friction curve.
3. ~~**The sim sometimes flips where the game jumped.**~~ **Fixed** — the wheel
   raycast was 2.5 UU short, so `is_on_ground` went false early and the press
   fell through to the airborne dodge branch. See *Suspension Travel Is The
   Whole 12 UU* below. Jump-window mean case-worst error dropped 94.8 → 45.2
   UU/s and `car_mech_ceiling_fall_flip` went 590.1 → 3.77.
4. **Sideflip direction is slightly off.** `car_sideflip_while_turning_left`
   matches in magnitude (game 521.0, sim 520.0) but not direction (game
   `(−272.8,−443.8)`, sim `(−263.0,−448.4)`), for 10.8 UU/s of error.

### Suspension Travel Is The Whole 12 UU

The wheel suspension raycast has to span the full `MAX_SUSPENSION_TRAVEL`. C++
RocketSim shortens it by `SUSPENSION_SUBTRACTION`, which is written in BT units
(0.05 BT = 2.5 UU) and therefore eats 2.5 of the 12 UU of travel; the Rust port
copied it faithfully. Wheels in that last 2.5 UU stopped reporting contact while
Rocket League still had them on the ground.

The recordings settle it. `WheelRecord::susp_length` maps exactly onto the sim's
`suspension_length - suspension_rest_length_1` — confirmed geometrically on
`car_mech_ceiling_fall_flip`, where both place the ceiling surface at z ≈ 2048.0
and the logged value tracks position one tick behind (`d(susp)[i] = -d(z)[i-1]`
to four decimals). Against that mapping:

- RL holds wheel contact continuously up to `susp_length` = **11.9999** out of
  12, with a flat sample distribution across 9.5..12.0 — no cutoff at the sim's
  9.5.
- 874371 of the no-contact samples read exactly **12.0**, matching the
  fully-extended value the no-contact branch assigns.
- 13431 in-contact samples sit beyond the sim's old 9.5 limit. 10390 of them are
  on the **flat floor** (`|normal.z| = 1`) and 7496 have all four wheels down —
  this is the suspension-extension phase of every jump and every landing, not a
  ceiling curiosity.

Removing the subtraction from the ray (it stays on the compression-side
`ray_pushback_thresh`, which ground truth says nothing about) improved 38 of 313
case/car velocity rows against 26 regressions, and mean `car.vel` p100 fell
318.99 → 313.64. Two regressions are real rather than fourth-decimal noise, and
both expose defects the fix uncovered rather than causing:

- `car_boost_then_jump` 6.59 → 14.37 UU/s. The sim's `is_on_ground` is now
  *correct* on the offending tick (that mismatch disappeared from the STATE
  line), which means engine force now runs through wheels carrying almost no
  suspension load. Our wheel friction and engine force do not scale with load at
  all. The recordings cannot arbitrate this: `lat_friction`, `long_friction` and
  `engine_force` are identically zero in every `WheelRecord` — the observer
  never populates them.
- `car_ball_ball_hits_tumbling_car` 127.8 → 181.3 UU/s, same tick (t=136),
  `over%` unchanged. A car falls onto the ball at 838 UU/s; the longer ray
  catches the ball one tick earlier and runs suspension against it. The sim
  already over-absorbed that impact by 128 UU/s, so this is an existing
  wheel-versus-dynamic-body defect being amplified, not a new one.

One more mapping note for future work: recorded `susp_length` runs as low as
−23, so RL does **not** clamp compression at `rest1 - MAX_SUSPENSION_TRAVEL`
the way `apply_ray_cast` does. About 1.2% of in-contact samples are below −12.

Double jumps are effectively **uncovered**: an airborne jump press with any stick
input is a flip, so almost every airborne onset classifies as one, and the
`car_double_jump_*` recordings put their second jump outside the measurable
window. Re-record if double-jump accuracy matters.

### Where The Car Error Actually Lives (RLCENSUS)

`RLSEG` splits error by situation but only reports means, and the car velocity
mean is heavy-tailed enough that a mean per regime says almost nothing: grounded
and airborne came out at 4.90 and 4.58 with rms 28 and 33, measured when the
suite mean was 4.78.
`RLCENSUS=1` files every step into exactly one bucket named after its *cause*
and reports the error **mass** (sum of \|err\|) per bucket, so buckets can be
ranked by how much of the total they own.

It is the same measurement as the gate, not a parallel one: it reuses
`has_discontinuity` and `is_car_sentinel`, and its measurable mean reproduces the
gate's to four decimals (4.7793 vs 4.7800 over 464 471 steps). If those ever
diverge, the census filters have drifted from the runner's.

Suite-wide, 2026-08-21, after the flip z-damp fix (suite mean **2.5953 uu/s**
over 461 340 steps), excluding the 1.3% of steps the gate skips as unmeasurable
impulse windows. Masses below are the pre-flip-fix ones; `air_boost` is now
115 742 at a mean of 2.26 and `air_free` 223 229 at 1.73, so `wall_ceiling` is now
the largest bucket:

| bucket | steps | mean | % of mass |
|---|---|---|---|
| `air_boost` | 11.1% | 5.16 | 19.1% |
| `air_free` | 28.0% | 1.93 | 18.0% |
| `wall_ceiling` | 7.4% | 6.30 | 15.6% |
| `drive_throttle` | 23.6% | 1.30 | 10.3% |
| `car_proximity` | 3.4% | 7.40 | 8.4% |
| `ball_contact` | 1.3% | 15.27 | 6.7% |
| `drive_partial` | 2.6% | 5.98 | 5.1% |
| `drive_boost` | 8.9% | 1.61 | 4.8% |
| `drive_handbrake` | 8.3% | 1.73 | 4.8% |
| `wheel_transition` | 0.6% | 14.05 | 2.9% |
| `drive_coast` | 4.2% | 1.81 | 2.5% |
| `touchdown` | 0.3% | 16.93 | 1.5% |
| `liftoff` | 0.2% | 2.91 | 0.2% |

Until 2026-08-21 this table read `car_proximity` 26.7% at a mean of 25.21, which
made it the largest bucket by a wide margin and the obvious next target. Three
quarters of that was the recording holding one car in two slots; see *Three
Quarters of `car_proximity` Is Corrupted Ground Truth* below. `RLKEEPDUP=1`
reproduces the old numbers exactly (464 471 steps, 3.7512) if you need them.

There was a twelfth bucket, `body_scrape`, at 34 steps and 0.04% of mass. It has
been deleted: the field it was built on cannot express what it claimed to
measure, and its steps belong to `wheel_transition`. See
*Chassis Scraping Does Not Exist* below, which also splits `wheel_transition`
into `touchdown` / `liftoff` / `wheel_transition`.

The bucket *membership* changed with that fix as well as the means: a step counts
as `drive_handbrake` when `handbrake_val > 0`, and rebuilding the ramp moved
18 685 mid-ramp steps out of `drive_throttle`, which is most of why that bucket
fell from 2.66 to 1.35. Per-bucket p50/p99 are not pooled here because the census
reports them per recording; the pre-fix values were `car_proximity` 1.75/445,
`drive_throttle` 0.69/33, `air_free` 0.04/17, `air_boost` 0.51/197.

Read p50 against mean, not mean alone. `air_free` and `air_boost` have p50 0.04
and 0.51 — free flight is essentially exact and their mass is entirely a 1-2%
tail, so there is no air-physics coefficient to chase. `drive_throttle` is the
sim's best broad regime (p50 0.69, flat across every speed band). The buckets
worth attention are the ones whose p50 is genuinely large.

**Beware the scripted-vs-match confound.** Splitting each bucket into purposely
scripted single-car recordings versus real-gameplay match recordings separates a
physics defect from an artefact of one recording style:

| bucket | scripted mean | match mean | verdict |
|---|---|---|---|
| `wall_ceiling` | 7.70 (n=5186) | 8.70 (n=29144) | reproduces in both — real defect |
| `drive_throttle` | 2.33 | 2.68 | reproduces in both |
| `air_free` | 1.43 | 2.00 | reproduces in both |
| `drive_handbrake` | 2.64 (n=1117) | 11.40 (n=11462) | 4.3x — match-specific |
| `drive_coast` | 1.54 | 6.57 | 4.3x — match-specific |

Three hypotheses for the handbrake/coast gap were tested and **refuted**, so do
not re-spend effort on them:

- *Non-Octane car bodies in match recordings.* `RecordingInfo::hitbox_rel_min_bt`
  / `_max_bt` are zero in all 424 files (the logger never fills them), but the
  bodies can be fingerprinted from ride height instead: median flat-floor upright
  `pos.z` is 17.00-17.06 and median front `susp_length` -1.98 in every recording,
  match and scripted alike. Every car in the suite is an Octane.
- *Control misalignment.* `RLCTRLOFF=-1|0|+1` shifts which tick supplies the
  step's controls. Offset 0 wins on every bucket and on both groups
  (match mean 5.13 vs 5.46 at -1 and 8.08 at +1), so the harness's assumption —
  tick `T`'s `prev_controls` drove the step `T-1 -> T` — is correct.
The third hypothesis on this list was *not* refuted — it was the cause, and the
refutation was wrong for an instructive reason:

- *Handbrake mid-ramp ticks.* Real play taps the handbrake, so
  `POWERSLIDE_RISE_RATE` could have been implicated. This was dismissed because
  `RLCENSUS=3` produced no `hb-ramp` band at all: recorded `handbrake_val` is
  only ever 0 or 1. **That is a property of the observer, not of Rocket
  League.** The field is logged as the button state; the game ramps the real one.
  A census banded on a field the log cannot represent will always report the
  band as empty, which reads exactly like evidence of absence. See the next
  section — fixing it was worth 17% of the whole suite error, and it explains
  both match-specific rows in the table above.

The handbrake and coast buckets were also the two the scripted-vs-match split
flagged as recording-style artefacts, which was the right read: match play taps
the handbrake constantly, so it lives mid-ramp, while a scripted powerslide holds
the button and spends almost all of its ticks settled. The lateral `magbias`
of +11.37 against a p50 of 10.73 at 1.8k+ — "the sim keeps too much lateral
speed" — was the sim snapping to full grip where the game was still sliding.

That is consistent with — and does not supersede — the bimodality result in
*Lateral Wheel Friction Has No Coulomb Limit* below. The census measures total
per-step error, not the per-tick required multiplier, so it cannot see whether
the residual is bimodal, and the standing instruction not to retune
`LAT_FRICTION` or `HANDBRAKE_LAT_FRICTION_FACTOR` on inference still holds.

#### Fixed: the wheel ray pushback used twice Rocket League's ERP

`wall_ceiling` was the largest car-only bucket surviving every confound check,
and its residual was **directional**: the mean signed residual along the car's
own up axis ran +1.69 (scripted) / +2.48 (match), rising to +3.76 in the 1.8k+
band. The car was being pushed off the surface.

It turned out not to be a wall defect at all. Banding by recorded `susp_length`
(`RLCENSUS=5`, then `=7` for 1-UU bands with the suspension quiet so the damping
term contributes nothing) shows the same ladder on the flat floor as on walls —
the up-axis residual is a function of **suspension compression**, and walls
merely contain a lot of hard-compression steps:

| compression (UU) | steps | up-bias | required spring k | required pushback k |
|---|---|---|---|---|
| 0-1 | 972 | -0.05 | 1.013 | (term inactive) |
| 1-2 | 171 926 | -0.20 | 1.026 | (term inactive) |
| 2-3 | 33 711 | -0.24 | 1.026 | 1.204 |
| 3-4 | 7 379 | +1.21 | 0.910 | 0.815 |
| 4-5 | 2 986 | +3.73 | 0.787 | 0.714 |
| 5-6 | 2 154 | +7.03 | 0.674 | 0.675 |
| 6-7 | 1 553 | +10.13 | 0.602 | 0.686 |
| 7-8 | 872 | +11.20 | 0.616 | 0.726 |
| 8-9 | 523 | +14.40 | 0.568 | 0.710 |
| 9-10 | 220 | +13.49 | 0.630 | 0.766 |
| 10-11 | 99 | +16.98 | 0.569 | 0.728 |

The knee sits at 2.5-3 UU, which is exactly where `extra_pushback` switches on:
`apply_ray_cast` runs it when `suspension_length < rest1 - SUSPENSION_SUBTRACTION`,
and `SUSPENSION_SUBTRACTION` is 0.05 BT = **2.5 UU**. Three candidate mechanisms
were separated by shape:

- **Spring stiffness** — rejected. A stiffness error is proportional at every
  compression, so it would show one flat multiplier; instead the required spring
  multiplier is flat at 1.026 up to the knee and then slides 0.91 -> 0.57.
- **A maximum suspension force** (C++ `Car.cpp:296` sets
  `m_maxSuspensionForce = FLT_MAX` with the comment "Don't think there's a
  limit", and the Rust port dropped the field) — rejected. A clamp needs the
  required multiplier to *fall* with depth; `k_push` is flat, and `k*compression`
  keeps rising rather than levelling off.
- **The pushback term** — fits. One constant multiplier explains every depth from
  3 to 11 UU, and below the knee the term's sensitivity is ~0 as the threshold
  predicts.

Scaling only the **positional** (Baumgarte) half of the pushback — leaving the
velocity half, which is a genuine collision response — fits better than scaling
the whole impulse (-3.98% vs -2.92% on the suite mean), and its optimum is at
half of Bullet's `m_erp`:

| effective ERP | 0.200 (was) | 0.150 | 0.120 | **0.100** | 0.090 | 0.080 | 0.060 |
|---|---|---|---|---|---|---|---|
| suite mean uu/s | 4.7793 | 4.6478 | 4.5981 | **4.5892** | 4.5958 | 4.6056 | 4.6287 |

So `contact_solver_info::RAY_PUSHBACK_ERP = 0.1`. Rocket League's Bullet dates
from 2013-2015 (as the C++ solver-info comment notes) and a factor of exactly two
in an error-reduction parameter is a plausible version difference; 0.1 is also
already the value of `SPLIT_IMPULSE_TURN_ERP`. `resolve_single_collision`'s only
caller is the suspension ray, so the constant lives at that call site — a second
caller would have to pass its own ERP in.

Result over 464 471 measured car steps: suite mean **4.7793 -> 4.5892 uu/s
(-3.98%)**, worst single step unchanged, and no bucket regressed:

| bucket | before | after | |
|---|---|---|---|
| `wall_ceiling` | 8.55 | 7.20 | **-15.8%** (p50 2.96 -> 2.00) |
| `drive_coast` | 3.66 | 3.29 | -10.0% |
| `drive_boost` | 2.42 | 2.20 | -9.0% |
| `drive_throttle` | 2.66 | 2.58 | -3.0% |
| `drive_partial` | 7.54 | 7.32 | -3.0% |
| `ball_contact` | 16.13 | 15.71 | -2.6% |
| `wheel_transition` | 14.64 | 14.45 | -1.3% |
| `drive_handbrake` | 10.63 | 10.50 | -1.2% |

The gate still reads 39 passed / 392 failed: it hard-gates on the *worst* step at
a 0.03 budget, so a broad mean improvement of this size does not flip cases.

Two smaller things appeared to fall out of the compression banding — a spring
~2.5% too weak below the knee (`k_spring` 1.026) and a large negative up-bias on
fast-*extending* steps pointing at `WHEELS_DAMPING_RELAXATION`. **Both were
measured directly and neither is real**; see the next section. Do not re-derive
them from `RLCENSUS` bands: those bands are built on the logged `susp_length`,
whose tick alignment relative to the pose is not resolvable, and a probe's
"required multiplier" mixes in every other error on the step.

### The Suspension Is Solved (RLSUSPDUMP)

Measured 2026-08-20. **The suspension needs no further work.** All four
coefficients are right, and the whole prize from re-fitting them is 0.011 uu/s.

The method is what makes this conclusive, and it generalises. On a flat floor
with an upright car and its wheels down, the *vertical* equation of motion is
closed and scalar: the only vertical impulses are gravity, the sticky force,
boost (through `forward.z`) and the suspension, because the wheel friction
impulse lies in the contact plane. So Rocket League's own suspension impulse for
a step falls straight out of the recording, with no simulation involved:

```text
dv_susp = (vel[i+1].z - vel[i].z) + 1.5 * g * dt - boost_z
```

That is an **absolute measurement of RL's suspension force**, not a "required
multiplier" from a probe sweep — so the constants can be *fitted* rather than
searched, and each one is identified separately instead of trading off against
the others. `RLSUSPDUMP=1` emits one row per usable step (92 169 steps over 201
recordings); the row format and the regression basis are documented in
`suspdump.rs`.

Fitted on the 78 803 rows below the pushback knee, where the spring and the two
damping branches are the only live terms (trimmed least squares — the residual
rms is two orders of magnitude above the median, so a plain fit reports landing
impacts rather than the bulk):

| constant | shipped | fitted | |
|---|---|---|---|
| `SUSPENSION_STIFFNESS` | 500 | **499.81** | right to 0.04% |
| `WHEELS_DAMPING_RELAXATION` | 40 | **40.00** | right to 0.01% |
| `WHEELS_DAMPING_COMPRESSION` | 25 | **25.37** | +1.5% |
| `RAY_PUSHBACK_ERP` | 0.1 | **0.087** | above the knee only |

Residual rms **0.0070 uu/s** over those 78 803 steps. The model reproduces RL
across the full compression range 1.5 -> 13 uu and rate range -260 -> +300 uu/s
with a median error of 0.0067 uu/s, and the sim itself tracks it at 0.0070.

So both apparent leftovers are refuted at the source: the spring is exact, and
`WHEELS_DAMPING_RELAXATION` is exact. Adopting *all* the fitted values moves the
mean |model - RL| only 0.0952 -> 0.0843 uu/s. `RAY_PUSHBACK_ERP` is the one
coefficient with real residual uncertainty — this channel prefers 0.087 — but
the ERP sweep above, which measures the whole suite rather than the flat-floor
vertical channel, is worse on both sides of 0.100 (0.090 -> 4.5958, 0.080 ->
4.6056). It stays at 0.100; the flat-floor preference is noted, not acted on.

What is left is a tail, not a coefficient: 2.06% of steps carry 85.8% of the
remaining vertical error mass, and the *model* misses 806 of those 1895 steps
too, so they are mostly sub-tick events (a wheel meeting the floor part-way
through a tick) rather than a force that is mis-scaled.

**Gotcha this pass exposed.** `set_state_to_record_tick` restores car and ball
state but *not* Bullet's persistent contact manifolds, whose cached impulses are
warm-started at `WARMSTARTING_FACTOR`. A diagnostic that restores and steps only
the ticks it cares about therefore fires a manifold built from a pose hundreds of
ticks old: the first version of this pass skipped non-candidate ticks and
reported a steady +49 uu/s of phantom suspension force on cars alone in an empty
half of the field. **Step every tick.** This is the same reason the sharded
runner needs `SHARD_WARMUP_TICKS`.

Two further traps worth knowing, both of which produced wrong answers first:

- **Do not read the recordings outside the harness.** `car_order::reorder_cars`
  canonicalises the car array because the logger's per-tick order rotates when
  cars pass close, so an external reader cannot even difference one car's
  velocity reliably, let alone line its rows up with the sim's.
- **Do not take the logged `susp_length` as the current pose's suspension
  state.** Its tick alignment is genuinely ambiguous (a direct lag test splits
  49.4%/50.6%). Compute compression from the pose instead — the wheel ray leaves
  the hardpoint along `-up` and meets the floor plane at `hard_point.z / up.z`,
  which matches the log to 0.002 uu on steady cases — and keep the logged block
  only as a gate. Cars on the corner ramps report a near-vertical normal on a
  *rising* surface, which fakes a fast-extending suspension; that gate is what
  rejects them.

### The Handbrake Ramp Is Not In The Recording (RLLATDUMP)

Measured 2026-08-20. **Worth 17% of the whole suite error, and it was a harness
defect rather than a physics one.**

`CarRecord::handbrake_val` is written by the observer as the handbrake *button*,
snapped to 0 or 1 — in `car_drift_powerslide_turn` it drops from 1 to 0 inside a
single tick. Rocket League ramps it, exactly as `Car::update_wheels` does, at
`POWERSLIDE_RISE_RATE` = 5.0 up and `POWERSLIDE_FALL_RATE` = 2.0 down.
`set_state_to_record_tick` restored the snapped value, so the sim jumped to full
lateral grip on the tick the button came up while the game still had almost none
— and stayed wrong for the 60 ticks the ramp takes to run out.

**The method generalises the suspension one to a second axis.** On a flat floor
with an upright car, the body right axis projected into the contact plane carries
*only* lateral wheel friction. Gravity and the sticky force act along the contact
normal (the sticky direction is the normalised sum of the wheel contact normals,
so on a flat floor it is exactly `+z`); `calc_friction_impulses` builds
`forward_dir = normal x axle_dir`, so the engine force, the brake and the whole
longitudinal friction term are exactly perpendicular to the axle *provided the
front and back axles coincide*, which needs `steer_angle == 0`; boost acts along
body forward and is subtracted explicitly; and the car body has zero linear
damping. So

```text
dv_lat = (vel[i+1] - vel[i]) . axle - boost_dv * (forward . axle)
```

is Rocket League's own lateral friction impulse, with no simulation involved. The
sim model for it is only *two* parameters, because `curves::LAT_FRICTION` is the
two-point curve `[(0, 1.0), (1, 0.2)]`:

```text
dv_lat = (1 + (h - 1) * handbrake_val) * sum_w q_w * mu(x_w),   mu(x) = a + b * x
q_w    = BT_TO_UU * dt / 3 * side_impulse_w
```

`RLLATDUMP=1` emits one row per usable step (28 313 steps over 165 recordings);
the row format is documented in `latdump.rs`. Restricting to `steer == 0` costs
most of the hard-cornering population, so this measures the curve and the
handbrake factor, not cornering.

**The ramp is measured, not inferred.** In `car_drift_powerslide_turn` the button
and the logged `handbrake_val` both drop to 0 at tick 100, but the implied
friction factor (RL divided by the wheel sum) climbs steadily afterwards:

| tick | 101 | 102 | 103 | 104 | 105 |
|---|---|---|---|---|---|
| implied factor | 0.1339 | 0.1489 | 0.1640 | 0.1789 | 0.1939 |

That is +0.0150 per tick, every tick, against a predicted
`0.9 * POWERSLIDE_FALL_RATE / 120` = 0.0150. Rebuilding the ramp by integrating
the button and then fitting the handbrake factor off it — a coefficient the
reconstruction was not fitted to — returns **0.1023** against the shipped
`HANDBRAKE_LAT_FRICTION_FACTOR` of 0.1. Both ramp rates check out separately, so
neither constant is absorbing the other:

| population | rows | mean \|err\| logged | mean \|err\| rebuilt |
|---|---|---|---|
| release side (`FALL_RATE` 2.0) | 1341 | 6.34 | **0.77** |
| press side (`RISE_RATE` 5.0) | 300 | 6.45 | **0.68** |
| ramp settled, either end | 10 523 | identical | 0.006–0.056 |

Where the ramp is settled the two agree exactly and the channel is already
accurate to 0.006 uu/s — so the ramp was the whole defect and the friction
constants underneath it were never wrong.

**The fix** is `normalize::normalize_handbrake_val`, which rebuilds the field at
load time by integrating `prev_controls.handbrake`. It belongs in `normalize.rs`
rather than in `runner.rs` for the reason that module exists: the observer and the
sim mean different things by the field, and every consumer — gate, census, deep
dive, rollout, C++ side-by-side — needs the same interpretation. Note that
`compare.rs` already excluded `handbrake_val` from comparison for exactly this
reason; the gap was that it was still being *restored*.

Result over 464 471 measured car steps: suite mean **4.5892 -> 3.8056 uu/s
(-17.1%)**, and no bucket regressed:

| bucket | before | after | |
|---|---|---|---|
| `drive_handbrake` | 10.50 | 1.90 | **-81.9%** (n 12 579 -> 38 249) |
| `drive_throttle` | 2.58 | 1.35 | **-47.5%** (18 685 mid-ramp steps moved out) |
| `drive_coast` | 3.29 | 1.91 | -42.0% |
| `drive_partial` | 7.32 | 6.07 | -17.1% |
| `drive_boost` | 2.20 | 1.70 | -22.7% |
| `wall_ceiling` | 7.20 | 6.41 | -11.0% |
| `wheel_transition` | 14.45 | 12.93 | -10.5% |
| `air_free` / `air_boost` | 1.932 / 5.156 | 1.932 / 5.156 | unchanged, as they must be |

`air_free` and `air_boost` coming out bit-identical is the sanity check: there is
no handbrake in the air. On the lateral channel itself, mean \|sim - RL\| over live
rows falls 1.306 -> 0.549 and p90 3.700 -> 1.150.

The gate reads 44 passed / 392 failed against 39 / 392 before, but the five extra
passes are this section's new unit tests — **no case flipped**. The gate hard-gates
the *worst* step at a 0.03 budget, so a broad mean improvement does not move it.

**Two smaller things this measurement also settles, neither acted on:**

- *The shipped curve is very nearly right.* Fitting the endpoints on the
  handbrake-released rows gives `LAT_FRICTION(0)` = 1.028 and `LAT_FRICTION(1)` =
  0.202 against 1.0 and 0.2. Fitting one free `mu` per slip-ratio bin shows RL
  sitting slightly *above* the shipped straight line in the middle (0.751 vs
  0.680 at x ~ 0.4, the largest gap anywhere), so the true curve is gently
  convex rather than linear. The ~3% excess at zero slip independently reproduces
  the `k_req` 1.030 that the probe method found over 13 000 ordinary driving
  samples, so it is real — it is just small.
- *The port lagged the friction coefficient by one tick.* **Fixed — see the
  next section.** Fitting both alignments said Rocket League does **not** lag:
  median \|resid\| 0.037 with the current tick against 0.064 with the previous
  one, and RL's own logged `friction_curve_input` from the previous tick is
  worst at 0.074.

### The Vehicle Update Ran In The Wrong Order (-1.4%)

Fixed 2026-08-20, acting on the lag the section above measured.

`WheelInfo::calc_friction_impulses` used to be called from inside
`apply_ray_cast`, i.e. from `update_vehicle_first`. It is a product of two
things: the raycast geometry, which that pass produces, and four per-wheel
coefficients — `lat_friction`, `long_friction`, `engine_force`, `brake` — which
`Car::update_wheels` produces *afterwards*. So every friction impulse multiplied
this tick's geometry by last tick's coefficients. C++ RocketSim does the same,
and this is one of the few places where the port now deliberately diverges from
it, because ground truth says Rocket League does not.

There were two stale reads, not one. Besides the coefficients, `apply_ray_cast`
derives each front wheel's `axle_dir` from `steer_angle`, which `update_wheels`
also writes afterwards — and `update_wheels` then computes
`friction_curve_input` along that same stale axle. So the *input* to the friction
curve lagged as well as its output.

**The fix is a reordering, in three parts:**

1. The handbrake ramp and the steer angle moved out of `update_wheels` into a new
   `Car::update_wheel_steering`, called *before* the raycast. Nothing in it needs
   raycast results, so the move is free, and it makes `axle_dir` current.
2. `apply_ray_cast` no longer computes the impulse. `RaycastInfo` gained
   `ground_body_idx` so the hit body can be looked up again later (the
   `&RigidBody` in the raycast result borrows the world and cannot be stored).
3. A new `VehicleRL::update_vehicle_friction` runs after `update_wheels` and
   fills in `RaycastInfo::impulse`.

The placement of step 3 is load-bearing. It must run *after* `update_wheels`, so
the coefficients are current, but *before* `update_jump` / `update_air_torque` /
`update_auto_flip` / `update_auto_roll`, which add non-accumulated impulses and
would otherwise change the contact velocity the friction reads. The sticky force
inside `update_wheels` is safe to sit before it because it is applied with
`accum = true`, which lands in `accum_lin_vel`, and `get_vel_in_local_point`
reads only `lin_vel`.

Result over the same 464 471 steps: suite mean **3.8056 -> 3.7512 uu/s
(-1.43%)**, with every wheel-contact bucket improving and the two airborne ones
bit-identical:

| bucket | before | after | |
|---|---|---|---|
| `drive_handbrake` | 1.896 | 1.731 | -8.7% |
| `drive_boost` | 1.704 | 1.615 | -5.2% |
| `drive_coast` | 1.912 | 1.813 | -5.2% |
| `wheel_transition` | 12.931 | 12.434 | -3.8% |
| `drive_throttle` | 1.353 | 1.309 | -3.3% |
| `wall_ceiling` | 6.411 | 6.295 | -1.8% |
| `drive_partial` | 6.067 | 5.982 | -1.4% |
| `air_free` / `air_boost` | 1.932 / 5.156 | 1.932 / 5.156 | unchanged, as they must be |

`air_free` and `air_boost` are the sanity check again: no wheel contact means no
friction impulse, so a change to friction ordering must not touch them. The gate
goes 44 -> 45 passing with **no case regressing**
(`car_ball_ball_hits_reversing_car` flips to passing).

### Lateral Wheel Friction Has No Coulomb Limit

The item long listed here as "the shared slip-friction curve" (powerslide
under-brakes, drift over-brakes) is **not a miscalibrated curve**. Measured
2026-08-20; the old framing should not be reused.

> **Superseded in part.** This section predates the closed-axis measurement in
> the section above, and its central open question — the bimodal handbrake
> residual — is now answered: the discrete state difference it correctly
> identified was the unlogged `handbrake_val` ramp. Its conclusions about the
> *curve* still stand and are independently confirmed (the ~3% excess at low
> slip shows up in both methods). Treat the probe method here as history: a
> "required multiplier" absorbs every other error on the step, which is what made
> the residual look unexplainable by any available predictor.

`WheelRecord::friction_curve_input` is live — 912 368 non-zero samples across 126
of the 424 files — and it is exactly the x-axis of `curves::LAT_FRICTION`.
Recomputing the sim's own formula from recorded `rot`/`vel`/`ang_vel` reproduces
it to mean |err| = 0.0017 over 4405 flat-ground back-wheel samples (0.000024 on
pure-steer cases), so **the curve input is correct**. Like `susp_length` it lags
position by one tick, which conveniently means the value recorded at `t+1` is the
one RL used for the step `t → t+1`.

The error turns out to be **independent of that input**: corr(|k−1|, fci) =
+0.027. Binning by `friction_curve_input` gives a misleading picture — bin by
*absolute lateral slip speed* instead.

How to measure it. On flat ground the world Δvel projected onto the car's right
axis (`dLat`) is *purely* wheel lateral friction, and it is exactly linear in
`lat_friction`: a two-point probe (temporarily scaling `lat_friction` by k = 1
and k = 2) gives `dLat(k) = k·A + C` with C ≈ 0 (|C/dLat| < 0.02). So the
required multiplier `k_req = (dLat_real − C)/A` is directly measurable per tick
from a `RLDEEP` dump. Set `RLTICK` to half the recording length and
`RLDEEP_RADIUS` to its full length to dump every tick.

Measured first on two single-car drift recordings, then **re-measured over the
match recordings** (`3v3` + `2v2_match`, 6 and 4 cars, 19 186 usable samples).
The wider set contradicts part of the drift-only result, so only the match
numbers below should be quoted.

Handbrake off, by mean lateral slip speed (match recordings, median `k_req`):

| mean lateral slip (UU/s) | n | `k_req` med | p25 |
|---|---|---|---|
| < 256 | 15 372 | 1.030 | 1.019 |
| 256–512 | 707 | 0.807 | 0.672 |
| 512–1024 | 143 | 1.024 | 0.705 |

So: a real but **milder** over-production of lateral force in the 256–512 band,
and **no** effect at 512–1024. The drift-only pass reported `k_req` 0.532 and
0.256 for those two bands with `lat_speed × k_req` pinned near 185, and
concluded RL's lateral force "peaks around 130–260 UU/s and then declines".
**That does not replicate** and should not be relied on — it was specific to the
post-handbrake-release phase of two drift recordings. There is also a small
systematic: the sim under-produces lateral force by ~3% (`k_req` 1.030) across
more than 13 000 ordinary driving samples.

**Whether any friction limit scales with normal load is not identifiable from
these recordings**, and the reason is structural rather than a filtering choice:
a car driving on the ground sits at its equilibrium ride height, so the load
proxy is pinned — over 17 003 samples it runs p05 = 1.961, p50 = 1.971,
p95 = 2.672. A constant cap and a load-proportional cap are therefore
*observationally equivalent* for lateral friction, and the question that looked
like it was blocking a fix cannot be answered with this data. (It still matters
for the low-load *longitudinal* case, `car_boost_then_jump` — a regime absent
from this sample.)

The dominant residual is **handbrake at fci < 0.3**, where the sim is ~5× too
weak over 959 samples. Critically that population is **bimodal, not scattered**:
p25 ≈ 1.0, median ≈ 5.1, p75 ≈ 6.1 — the sim is either exact or off by ~5×. A
bimodal split implies a discrete state difference, not a curve to retune. None of
the available predictors explains it: fci (R² = 0.05), lateral slip speed, normal
load, wheel-spin longitudinal slip (R² = 0.048), throttle sign, boost,
supersonic, wheel count, or steering (the best discrete separator, `|steer| = 1`,
only splits 0.79 vs 0.44).

**This is now solved, and the reasoning above was right.** The discrete state was
the true `handbrake_val`: the sim ran at the settled factor 0.1 while the game was
mid-ramp somewhere between 0.1 and 1.0, which is a ratio of up to 10x and a median
near 5. It resisted every predictor because the predictor that explains it was not
in the recording at all. p25 = 1.0 is the settled population; the ~5x mode is the
ramp. See "The Handbrake Ramp Is Not In The Recording" above.

Above fci 0.3 with handbrake the sim is **exact**, and this is what validates the
whole method: the measurement recovers the sim's own `HANDBRAKE_LAT_FRICTION_FACTOR`
of 0.100 to three decimals (n = 205, p25–p75 = 0.099–0.100). The negatives above
are therefore real, not measurement failure.

**Do not retune `LAT_FRICTION` or the handbrake curve on this evidence.** Fitting
a smooth curve to a bimodal residual would lower mean error while encoding the
wrong mechanism. The unblock is the observer: `lat_friction`, `long_friction` and
`engine_force` are dead fields that the logger already has slots for. Populating
them and re-recording would expose RL's actual per-wheel coefficient directly and
turn this entire subsystem from inference into measurement.

Both are **faithful to C++**: `RLConst.h:483` really does define
`HANDBRAKE_LAT_FRICTION_FACTOR_CURVE = {{0, 0.1f}}`, and
`btVehicleRL.cpp:306 calcFrictionImpulses` has no cap either. These are upstream
modelling gaps, not Rust port slips — don't go hunting for a transcription error.

Ground truth decoded alongside: `WheelRecord::spin_speed` is **live** (non-zero
in 97.9% of wheel samples) and is the wheel's angular velocity in rad/s —
`spin_speed × wheel_radius` recovers the forward contact speed, fitting 12.60
against the true 12.5 for the front pair and 15.04 against 15.0 for the back.
The within-pair asymmetry is just inner/outer radius in a turn, which
independently confirms both the radii and the left/right wheel assignment. RL
tracks wheel angular velocity; the sim has no wheel-spin state at all, though
adding it does *not* explain the handbrake residual. `lat_friction`,
`long_friction`, `engine_force` and `steer_amount` are the genuinely dead
fields.

Tooling: `RLDEEP_RADIUS` is no longer capped at 240, so a single run can dump
every tick of a recording (`RLTICK=<len/2> RLDEEP_RADIUS=<len>`), which is what
the per-tick calibration sweep needs. The sweep itself scales `lat_friction` by
k = 1 and k = 2 behind a temporary probe in `Car::pre_tick_update`; it is not
committed.

### Chassis Scraping Does Not Exist (RLTOUCH)

The census had a `body_scrape` bucket, meant for a car dragging its chassis with
no wheel down - rolled onto a side or the roof. It scored a mean of 21 uu/s over
34 steps with a near-deterministic downward bias, which read like the cleanest
single-mechanism defect anywhere in the dataset. It was an artefact of the field
it was defined on.

`RLTOUCH=2` surveys that field, `PhysRecord::has_world_contact`, over the whole
suite. Three results, all unambiguous:

| measurement | result |
|---|---|
| car-ticks with **no** wheel in contact where `has_world_contact` is true | **0** of 193 339 |
| steady wheel-contact ticks where it is true | 254 252 of 271 516 (93.6%) |
| **touchdown** ticks (0 wheels -> 1+) where it is true | 40 of 1383 (**2.9%**) |
| logged `world_contact_point` away from the origin | **0** of 254 252 |
| logged `world_contact_normal` not exactly `+z` | **0** of 254 252 |
| *control:* wheel-contact ticks with a **`WheelRecord`** normal off the flat floor | 43 270 of 271 516 (15.9%) |
| *control:* wheel-contact ticks with `min_contact_z < 0.9` | 37 421 (13.8%) |
| *control:* wheel-contact ticks with a wheel normal `z < 0` (ceiling) | 4 521 |

The last three rows are the control, and they are why this is a statement about
one field rather than about the observer. The **`WheelRecord`** contact normals
are a different field from the `PhysRecord` one and they are **live**: 15.9% of
contact ticks are off the flat floor, and `min_contact_z < 0.9` - the exact
condition `wall_ceiling` is defined on - holds on 37 421 of them, which is the
census's 34 330 plus the steps earlier buckets claim. **`wall_ceiling` does not
share this defect.** Only the three `PhysRecord` world-contact fields are dead.

Counts are approximate to about 2%: under parallel test threads a small fraction
of `println!` lines are lost from the captured stdout, so repeat runs vary
(437 k-465 k car-ticks). The *ratios* are stable across runs, and the exact
zeroes are exact - a lost line can only lower a count, never turn a nonzero into
a zero.

So the flag never reports a chassis-only contact - it tracks *wheel* contact, and
it lags it by a tick. The bucket condition was
`nw_from == 0 && (from.has_world_contact || to.has_world_contact)`, and since the
`from` term is dead the bucket could only fire through `to`. Its 34 steps were
therefore the 3% of landing touchdowns where the lagging flag happened to be on
time - a biased sample of a completely different mechanism, wearing the name of
one this dataset never records. **Two more observer fields are dead as well:**
`world_contact_point` is the origin and `world_contact_normal` is exactly `+z` on
every one of 254 252 logged contacts, including the suite's thousands of wall and
ceiling steps. Infer nothing from any of the three.

The bucket is gone. `wheel_transition` now splits three ways instead, on what
actually changed, and the split is worth having:

| bucket | steps | mean | note |
|---|---|---|---|
| `touchdown` (0 -> 1+) | 1204 | **16.93** | gaining contact |
| `wheel_transition` (n -> m, both > 0) | 2891 | 14.05 | count shifting while in contact |
| `liftoff` (1+ -> 0) | 1029 | **2.91** | losing contact |

Total mass is unchanged (suite mean 3.7512 before and after - the steps only
changed label). **Acquiring contact is 5.8x worse than releasing it.** Losing
contact is nearly free, at the level of ordinary partial-contact driving.

#### What Actually Goes Wrong On A Landing

`RLTOUCH=1` measures the touchdown tick on the closed vertical axis. At the start
of such a step no wheel is in contact, so there is no suspension force, no sticky
force and no wheel friction, and the car body has zero linear damping - RL's own
contact impulse is `(vel[i+1].z - vel[i].z) + g*dt - boost_z`, straight from the
recording. Each row also carries the pose-derived wheel ray length at **both**
ends of the step and how many wheels each pose puts within `MAX_SUSPENSION_TRAVEL`.

On 476 flat-floor near-upright touchdown steps:

- **The sim's contact decision is exactly its own raycast, as designed.** The
  wheel count the sim ended the step with equals the count the *start*-of-step
  pose predicts on **473 of 473** rows. The instrument reproduces the sim
  bit-for-bit, so any disagreement below is RL's, not measurement error.
- **RL's is not.** On the 291 steps where the start-of-step pose puts no wheel
  within 12 uu, RL has already applied a non-gravity vertical impulse on **90%**
  of them (mean +6.0 uu/s) while the sim found no contact at all on **0 of 291**.
  That population carries **66% of the touchdown tick's error mass**.
- The mirror case is real but small: 13 steps where the sim engages and RL does
  not, and there the sim's dv is `-2.707` - exactly the sticky force
  (`0.5 * g * dt`). Gentle settling makes the sim stick a tick early. 1.4% of mass.

So RL responds to geometry the start-of-tick pose has not reached. The obvious
fix - evaluate the suspension one tick later in phase, at the end-of-step pose -
**is wrong, and the measurement says so before any of it gets written.**
Transcribing the sim's own suspension model (`update_suspension` term for term,
plus the sticky force) and evaluating it at each pose:

| phase | sum \|model - RL\| over the 476 steps |
|---|---|
| start-of-step pose (what the sim does) | 3027 |
| the sim as it actually stands | 2980 |
| end-of-step pose (one tick early) | **7454** |

The end-pose model overshoots by 2.5x on exactly the rows that motivate it - RL
+61.9 where it predicts +132.8, RL +44.5 where it predicts +101.3. RL's arrival
impulse is a *fraction* of a full tick of suspension force, which is what a
contact beginning part-way through the tick looks like, not a whole-tick force
moved sideways in time. (The model transcription itself is sound: evaluated at
the start pose it reproduces the sim's measured `dv_z` with p50 -0.004 and mean
\|residual\| 0.409 uu/s.)

#### Sub-Tick Contact Onset Was Tried And Does Not Pay

The implied fix is sub-tick onset: lengthen the wheel ray by the distance the
hardpoint will descend this tick, and when the hit lies beyond the travel limit
charge the wheel only the fraction of a tick it was actually in contact for -
mean compression running from `-MAX_SUSPENSION_TRAVEL` (fully extended, which is
what first contact means) to wherever the closure reaches, with
`update_suspension`'s stiffness, damping branch, force scale and zero-clamp
otherwise untouched. It was implemented and measured. **It made the suite worse
by +0.42%** (3.7512 -> 3.7669) and it is reverted. Two reasons, both worth
keeping:

1. **It fires on far wheels while near wheels already have contact.** `predF` is
   a per-car count but the onset is per-wheel, so on a pitched landing the
   trailing wheel gets an onset impulse on top of the leading wheel's ordinary
   suspension. On those steps the sim was **already right** (RL +64.2 against the
   sim's +62.0), and the onset tripled it. Restricting the onset to cars with no
   wheel in contact at all fixes this, and is clearly correct - never augment a
   case that already matches.
2. **Even correctly scoped it overshoots by 2x.** On the 152 no-wheel-contact
   steps where it fires, `sim/RL` has p25 1.41, **p50 1.98**, p75 2.48, and 113 of
   152 overshoot by more than 1.5x. Net error moves only -3.0%, because "0 where
   RL says 10.2" is replaced by "17.6 where RL says 10.2". On the other 137 steps
   it stays silent and RL's own mean is +0.58, so those were already fine.

The factor of two is real and repeatable but has **no mechanism behind it**. It is
not the spring ramp (mean compression already handles that), not velocity decay
during the tick (the damping time constant is ~24 ticks, so 2.5% over the
fraction), and not the tilt factor (which reduces the term). Halving the impulse
would fit one constant to 152 rows against an unknown cause - the same mistake
the `LAT_FRICTION` bimodality warned about, where the missing variable was not in
the recording at all. **Do not add the 0.5.**

Sizing says stop here regardless: `touchdown` is 1.2% of suite mass, so even a
*perfect* landing tick is worth 1.2% of the suite mean, and the correctly scoped
onset captures 3% of that. The measurement infrastructure (`RLTOUCH=1`, the
`touchdown` bucket) is kept because it scores any future attempt in one run, and
the 2x overshoot is the number that attempt should start from.

### Three Quarters of `car_proximity` Is Corrupted Ground Truth (RLCARC)

`car_proximity` led the census at 26.7% of the suite's measurable mass, and it is
selected by *centre* distance under 240 UU. That is a net, not a contact test:
two Octanes 240 UU apart have 120 UU of clear air between them. `RLCENSUS=8`
bands the bucket by the real hitbox gap, from the 15-axis SAT separation in
`census::hitbox_separation`, and by the cause each step would have had if no car
were nearby:

| gap band | steps | mean | % of `car_proximity` mass | % of suite mass |
|---|---|---|---|---|
| `touch` (hitboxes overlap) | 4 036 | 96.89 | 82.9% | 22.4% |
| `0-10` | 2 678 | 10.06 | 5.7% | 1.5% |
| `10-40` | 3 304 | 5.40 | 3.8% | 1.0% |
| `40-100` | 6 434 | 3.44 | 4.7% | 1.3% |
| `100+` | 2 299 | 6.00 | 2.9% | 0.8% |

So the bucket is genuinely about contact -- every non-touching band runs a mean
of 3.4 to 10.1, which is ordinary driving error. It is also not the *bump*:
`detect_bump_onsets` already diverts every step whose recorded position/velocity
identity breaks, i.e. every sub-frame impulse, into the `impulse` bucket that the
census total excludes. What is left had to be sustained overlap being pushed
apart across a whole frame.

It is neither. `RLCARC=1` measures the pair on a closed axis -- two cars of equal
mass take the same gravity, so gravity cancels exactly in their *relative*
velocity, and `((v_b' - v_a') - (v_b - v_a)) . n` is RL's own contact impulse with
no simulation involved. What that dump actually shows is that **1 736 of 2 255
overlapping pairs sit at centre separations below 38.66 UU**, the smallest Octane
hitbox dimension and therefore closer than two hitboxes can be in *any*
orientation. On 91% of them the separation equals exactly one tick of travel
along the car's own velocity (`|d - speed * dt|` p50 = **0.005 UU**, float
noise), and the pairing is structured rather than scattered: in `2v2_3` the pairs
`[0,2]` and `[1,3]` account for 174 steps each.

The raw records (`RLCARC=3`) say what it is. At tick 356 of `2v2_1` the four car
slots carry `physics_frame` **357, 358, 358, 357** -- the cars are on different
physics frames inside one recorded tick. Slot 1 holds slot 0's car advanced one
frame (identical `fwd` and `up` to three decimals, velocity within 6 UU/s,
position offset exactly one frame of travel along its own velocity), and the car
that was really in slot 1 the tick before, 3 200 UU away, has vanished from the
array. Slots 2 and 3 do the same. Four slots, two real cars, each duplicated a
frame apart.

`car_order::reorder_cars` is not the cause and cannot repair it: its matching is
bijective, so when the array holds one car twice it is *forced* to file the
copies under two different canonical slots, and it has no car to put in the
vanished one. This is the same logger defect `detect_bump_onsets` documents from
the other side -- "`2v2_1` cars 1 and 3 hold `|dpos|` = 38.4 UU for 18
consecutive ticks at exactly 2300 UU/s, precisely `2 * vel * dt`" -- caught there
per car via the `physics_frame` advance and here as disagreement *between* cars.

`RLCARC=2` surveys it. The split is total:

| recordings | intra-tick frame desync |
|---|---|
| every scripted `car_car_*` / `mech_*` | **0.0000**, spread 0 |
| `2v2_4` | 0.047 |
| `2v2_5` | 0.152 |
| `2v2_3` | 0.421 |
| `2v2_match` | 0.475 |
| `2v2_match_3` | 0.548 |
| `2v2_6` | 0.552 |
| `2v2` | 0.652 |
| `2v2_2` | 0.914 |
| `2v2_1` | 0.926 |
| `3v3` | **0.937** |

A one-frame stagger between two *distant* cars is harmless, which is why the
desync fraction alone is not a defect measure -- voiding every desynced tick
would throw away most of the match-replay coverage. It matters in two specific
ways, and `RLCENSUS=9` bands every bucket by both:

- **`bad-adv`** -- the measured car's own `physics_frame` advance across the step
  is not `stride`. The stagger flipped mid-step, so the ground truth moved two or
  three frames while the sim was asked for one. 0.37% of steps.
- **`dup-other`** -- the measured car advanced cleanly, but the tick holds a
  duplicated car, so the geometry every car-car force is computed from is
  mis-registered. 0.35% of steps.

| category | steps | % steps | mean | % of suite mass |
|---|---|---|---|---|
| `clean` | 429 855 | 99.28% | **3.08** | 78.7% |
| `dup-other` | 1 517 | 0.35% | **174.01** | 15.7% |
| `bad-adv` | 1 614 | 0.37% | **58.31** | 5.6% |

**0.7% of steps carry 21% of the suite's error mass, and none of it is physics.**
Voiding them takes the suite mean to **2.9981, down 20.1% from 3.7512**,
and `car_proximity` collapses: **76.2% of its mass is junk**, its clean mean is
7.38 rather than 24.99, and its share falls from 27.1% to 8.4%. It is no longer
the largest bucket. On clean steps the ranking is `air_boost` 20.0% (mean 5.16),
`wall_ceiling` 16.3% (6.31), `air_free` 15.0% (1.94), `drive_throttle` 10.8%
(1.31), then `car_proximity` 8.4%.

Every single-car bucket is 0.0% junk, which is the detector validating itself:
the defect only exists in multi-car match replays, and a collapsed array puts two
slots in the same place, so those car-steps are all claimed by the 240 UU
proximity net before any other bucket sees them.

#### The duplicate test needs both halves

Impossible proximity alone is not enough. `car_car_boost_contest` and
`car_car_long_boost_headon` each put two cars 6 steps deep inside one another
during a head-on boost collision. That is a real physical state RL is resolving,
their mean error there is an ordinary 10 UU/s rather than 174, and both
recordings are scripted with a uniform `physics_frame` on every tick. Requiring
non-uniform frames *and* impossible proximity removes exactly those 124 units of
false-positive mass and nothing else (`dup-other` mass 264 091 -> 263 966).

Non-uniform frames alone are far too aggressive, per the table above. Use the
conjunction.

#### Fixed: these steps are voided (-20.1%)

`runner::has_discontinuity` now rejects a step when the tick holds a duplicated
car or when any live car's own `physics_frame` advance is not `stride`. That is
the right home for it: `has_discontinuity` is the single tick-wide void shared by
the runner, the census, the C++ comparison pass and the impulse windows, so all
four inherit the fix without a second definition to keep in step.

Result over the suite: **3.7512 -> 2.9981 uu/s (-20.1%)**, 3 131 of 464 471 steps
voided, gate unchanged at 45 passing. Every single-car bucket's mass is identical
to the last digit (`air_boost` 264 715, `air_free` 249 381, `wall_ceiling`
216 116, `drive_partial` 70 978), which is the check that this removed only
multi-car corruption: `car_proximity` loses 75.4% of its mass while the other
buckets lose only the far-away cars that happened to share a collapsed tick.

`RLKEEPDUP=1` puts them back and reproduces the old totals exactly (464 471
steps, 3.7512). Use it to re-measure the artifact, never to report a number.

Both tests must stay tick-wide. The frame-advance break is per car, but the array
collapse is whole-array -- when it happens every slot is affected in the same
tick -- so voiding per car would leave the mis-registered geometry in place for
the cars that still advanced cleanly.

Counts move a few percent between runs because parallel test threads drop about
2% of captured stdout lines; the ratios above are stable and the exact zeroes
stay exact.

## The Accuracy Bar

A case **passes** when no single simulated step diverges from the recording by
more than **0.03** in the field's native unit — 0.03 UU of position and
0.03 UU/s of velocity — for *every* entity on *every* tick. The gate is a
maximum, not a percentile (`percentile = 1.0`): one bad step fails the case.

Because the per-tick pass restores ground truth before every step, this measures
single-step physics error in isolation, which is exactly "per step divergence".

This is a deliberately hard bar. As of the last full survey **32 of the 424
valid cases pass**; contactless motion (aerials, air roll, free-flight and
slow-rolling ball) sits at 0.006–0.025, while anything involving a collision or
an impulse spikes on the contact tick — a plain `ball_bounce_ground` keeps 97.4%
of ticks under 0.03 UU but hits 3.18 UU on the bounce. Most of the remaining
failures are a *single* tick: see "Impulse Onset Ticks" above for why jump and
flip cases cannot clear the bar on the two frames that straddle a button press. Use `RLGATE=off` with loosened `RL_*_TOL` when you need a metric that
discriminates progress rather than a pass/fail.

## Quick Start

```bash
# From external/rocketsim-v3-rust

# SURVEY: measure everything, never fail, no deep dives. Start here.
RLGATE=off cargo test -p rocketsim -- --nocapture --test-threads=1

# GATE: enforce the physics budgets (fails on divergence). Per-case deep dive.
cargo test -p rocketsim -- --nocapture --test-threads=1

# One case by name (snake_case of the .rlpr filename):
cargo test -p rocketsim case_drive_5 -- --nocapture --test-threads=1

# Pattern match:
cargo test -p rocketsim jump -- --nocapture --test-threads=1
```

`--test-threads=1` is required (the sim init is global). `--nocapture` shows
the report lines.

## The Two Channels

**Physics channel (gated).** Per tick we compare sim vs recording for each
entity (every car + the ball) and each field: `pos`, `vel`, `ang_vel`,
`rot_fwd`, `rot_up`. `pos` and `vel` are **hard-gated**: the test fails if
their percentile exceeds the budget. The others are **soft** (reported only).

**State channel (reported, never gated).** Discrete flags (`is_jumping`,
`has_flipped`, …) are reported as a *mismatch rate*. These are state-machine
*timing* bugs, a different fix from integration errors, so they don't fail the
test.

`wheels_contact` is the sharpest flag in that channel and the one to look at
first for anything involving the ground. RLPR stores per-wheel contact (contrary
to a since-corrected comment in `runner.rs`), and RL's own grounding rule is
exactly the sim's: across 510015 samples, recorded `on_ground` equals
`wheel_count >= 3` at the same tick index in **100.0000%** of cases, with no
exceptions. That makes wheel contact strictly upstream of `is_on_ground`, and
measurably sharper — it caught 2.132% of ticks against `is_on_ground`'s 0.672%
on the pre-fix ray, a 3.2× higher hit rate, because most wheel disagreements
never cross the 3-of-4 threshold. Fixing the ray length halved both (to 1.065%
and 0.340%). The companion `wheel_count |dn|` figure on the STATE timers line
gives the magnitude.

The comparison is per **pair** (front count, back count), not an exact four-bit
mask, because when the channel was built the recording's left/right order within
a pair could not be pinned down: `{0,2}` and `{1,3}` are the two common side-lift
masks in near-equal numbers (3148 vs 3108) and `steer_amount` is never populated,
so the front pair cannot be identified from it. Front/back grouping *is* certain,
and a per-pair count is what `n >= 3` turns on, so a left/right lift swap was the
one blind spot.

That blind spot is now closed. The `ang_vel × wheel_delta` term in the
`friction_curve_input` formula breaks the left/right symmetry, and recomputing
fci for the back pair under both assignments separates them 25:1 (mean |err|
0.0025 vs 0.0625 on `car_powerslide`): **wheel index 2 is the −y side and index 3
the +y side**, with back offset x = −33.75, |y| = 29.50, z = 20.755. Tightening
`wheels_contact` to an exact four-bit mask is therefore possible; it has not been
done yet.

## Reading a Report Line

```
[drive_5] car_0 vel : p=15.06 max=58.4@t196 mean=6.5 rms=11.0 bias=2.4 over=100% budget=0.03 -> FAIL
```

| token | meaning |
|---|---|
| `p` | the gated percentile of the per-tick error magnitude (default p100, i.e. the max) |
| `max@tN` | worst single-tick error and the tick index it happened at |
| `mean` / `rms` | average magnitude / root-mean-square magnitude |
| `bias` | magnitude of the **mean signed error vector** (a `bias_vec` line below gives direction). Non-zero bias = a wrong constant/formula; scatter with ~0 bias = integration precision |
| `over` | % of ticks exceeding the budget |
| `budget` | the tolerance for this field |

**Bias vs rms is the key diagnostic.** `bias ≈ rms` → systematic error, fix a
constant or formula. `bias << rms` → scattered error, usually integration
order/quantization, often acceptable.

## Env-Var Knobs

| Env var | Meaning | Default |
|---|---|---|
| `RLGATE` | `strict` \| `off` — enforce physics budgets | `strict` |
| `RLNOVALIDATE` | `1` to skip the ground-truth validity check | off |
| `RLDEEP` | `fail` \| `always` \| `never` — when to emit a deep dive | `fail` |
| `RLDEEP_RADIUS` | context ticks on *each side* of the deep-dive center | `15` |
| `RLCAR` | focus one car index (`0`…), or `all` | `all` |
| `RLTICK` | deep-dive a specific tick instead of the worst | worst |
| `RLSEG` | `1` to also print per-situation breakdowns | off |
| `RLCENSUS` | `1` error-mass census by cause; `2` also lists each bucket's worst steps; `3` sub-bands by speed + handbrake state; `4` by `friction_curve_input`; `5` by suspension compression; `6` by compression x compression rate; `7` 1-UU compression bands with a quiet suspension | off |
| `RLCTRLOFF` | shift which tick the census reads controls from (`-1`/`0`/`+1`), to re-verify control alignment | `0` |
| `RLLATDUMP` | `1` to emit one `LATROW` per flat-floor zero-steer step: RL's own lateral friction impulse and the sim's, with per-wheel slip ratio and side impulse. See `latdump.rs` | off |
| `RLBALL` | `1` census of ball velocity error by contact cause; `2` liveness survey of the ball's world-contact fields plus a position-vs-velocity audit of the recording itself; `3` per-step `BALLDUMP`; `4` `BALLBOUNCE`, judging a bounce by its outcome across a free rollout with tick alignment. See `ballcensus.rs` | off |
| `RLFLIP` | `1` to emit one `FLIPZ` row per isolated airborne flip step: RL's own z-damp decision on the closed vertical axis, with `flip_time`, tumble, `up.z` and both boost orderings. Filter to `0.35*|vzF| > 20` before reading. See `flipdamp.rs` | off |
| `RLCENSUS` (`10`) | band every bucket by the recorded jump/flip state and `flip_time` phase | off |
| `RLCARC` (`4`) | survey car-identity transpositions after an array collapse (`CARCSWAP`). Zero suite-wide; kept as a ruled-out hypothesis | off |
| `RLKEEPDUP` | `1` to keep the duplicated-car and broken-frame-advance steps *in* the measurement instead of voiding them. They are 0.7% of steps and 21% of error mass, so this inflates every mean by ~25%; for re-measuring the artifact only | off |
| `RLCARC` | `1` to emit one `CARC` row per overlapping car pair: RL's own contact impulse on the closed relative-velocity axis and the sim's; `2` to survey intra-tick `physics_frame` agreement (`CARCSYNC`/`CARCADV`); `3` with `RLCARC_REC=<name> RLCARC_T=<lo>:<hi>` for a raw per-car dump. See `carcontact.rs` | off |
| `RLTOUCH` | `1` to emit one `TOUCHROW` + `TOUCHSUSP` per landing-touchdown step: RL's own contact impulse and the sim's, with the wheel ray geometry at both ends of the step; `2` to survey the `has_world_contact` field instead. See `touchdown.rs` | off |
| `RLSUSPDUMP` | `1` to emit one `SUSPROW` per flat-floor step: RL's own suspension impulse and the sim's, with per-wheel compression and rate. See `suspdump.rs` | off |
| `RLTHREADS` | worker threads for the per-tick pass | all cores (≤16) |
| `RLCONT` | `1` to also run the sequential continuous (compounding) pass | off |
| `RLCPP` | `1` to also replay through the **C++ RocketSim** (`rocketsim_rs` bindings) and report side-by-side. Requires `--features cpp-compare` | off |
| `RLROLL` | rollout mode: restore at sampled ticks, free-run `1s`/`2s`/`<ticks>` with recorded controls, report the error growth curve | off |
| `RLROLL_STRIDE` | distance (ticks) between rollout start ticks | `120` |
| `RL_POS_TOL` / `RL_VEL_TOL` / `RL_ANGVEL_TOL` / `RL_ROT_TOL` | budget overrides | 0.03 / 0.03 / 0.5 / 0.02 |
| `RL_PERCENTILE` | gate percentile (0..1); `1.0` = gate on the worst step | `1.0` |

## C++ RocketSim Comparison (RLCPP)

The original C++ RocketSim (via the `rocketsim_rs` crates.io bindings) can run
the exact same per-tick restore pass — and rollout pass — as a reference:
whatever C++ nails and our port misses points straight at a bug in our port,
and whatever both sims miss equally is a shared RocketSim limitation, not a
porting bug.

The C++ sim is compiled on first use (bullet3 + RocketSim, a few minutes) and
is **opt-in** so normal test runs stay fast:

```bash
# Build + run with the C++ reference pass:
RLGATE=off RLCPP=1 cargo test -p rocketsim --features cpp-compare case_3v3 -- --nocapture --test-threads=1
```

Output: the normal report lines, then the same lines under the `|cpp` report
name, then a `CPP-COMPARE` ranking (ratio > 1 = C++ more accurate at p95).

Caveats:
- The C++ default arena uses `noBallRot=true`: the ball's rotation matrix is
  never integrated, so C++ "ball rot" error is 0 by construction. The harness
  annotates those rows instead of ranking them.
- Demoed/parked cars are parked far below the arena in the C++ pass (restoring
  several of them at the origin makes bullet's solver produce NaN).
- The C++ pass never gates; it is informational only.

### The Ball Was Never Censused, And Is Mostly Right (RLBALL)

The ball is gated on the same per-tick bar as the cars and carries **461 of
the suite's 1009 gate violations** against 548 for every car combined, but
`census.rs` only ever files *car* velocity error, so where the ball's error
lived was simply unknown. `RLBALL` is the ball's census.

Two things make the ball a cleaner subject than the car. **143 recordings have
no car in them at all**, so the system closes to gravity, drag and the arena
mesh. And free flight is already exact -- `ball_very_slow` scores 0.009 uu/s,
`ball_spin` 0.002, `ball_near_*` 0.001 -- so drag and gravity are right and
anything left is a contact.

#### Both other members of the contact triple are dead, again

`RLBALL=2` surveys the ball's own `has_world_contact` / `world_contact_point` /
`world_contact_normal`, which nobody had checked: the `RLTOUCH=2` survey behind
[[world-contact-fields-are-dead]] walked `car_records` only.

| measurement | result |
|---|---|
| ball ticks | 133 280 |
| `has_world_contact` true | 13 904 (10.4%) |
| of the 2 801 kinematic bounces, flagged | 827 (**29.5%**) |
| `world_contact_normal` nonzero | 13 904 |
| ...of those, exactly `+z` | **13 904 (all of them)** |
| `world_contact_point` nonzero | 126 807 (95%) |

So the normal is dead in the same way it is on cars. The *point* is nonzero,
which is new -- but it is always the ball's own position with `z` zeroed, on a
floor contact and 250 UU up a corner alike. **Infer nothing from any of the
three, on the ball either.**

#### But the ball's frames are clean, unlike the cars'

`runner::frame_advance_broken` walks `car_records` and nothing else, so the
ball's `physics_frame` is validated nowhere in the harness. It turns out not to
matter: bucketing on it fires on **zero** steps suite-wide. The ball's ground
truth advances exactly `stride` every time, so the defect that owned three
quarters of `car_proximity` ([[match-replays-duplicate-cars]]) has no analogue
here.

#### 97% of ball steps are already exact

| | steps | share |
|---|---|---|
| all measured ball steps | 155 739 | |
| error > 1 uu/s | 4 906 | 3.15% |
| error > 10 uu/s | 3 840 | **2.47%** |
| error > 100 uu/s | 2 532 | 1.63% |

Every ball bucket is bimodal, so read the counts and never the means: the
`free` bucket has p50 **0.006** and a max of **5 269**. A mean of 7.7 uu/s
there describes nothing that happens.

#### Measure a bounce by its outcome, not by one tick of it

This is the important methodological point, and it is the same trap as
[[car-car-bump-is-a-subframe-event]]. `ball_corner_fast_from_center` crosses the
corner at 4 340 uu/s, i.e. **36 UU per tick**. Rocket League splits the bounce
across two ticks; the sim, restored to RL's state every tick, reports `np=0` --
no contact at all -- on both, because RL's own resolution keeps the ball ~13 UU
short of the sim's contact threshold at every tick boundary. The per-tick bar
charges that phase difference the **entire** impulse, twice.

`RLBALL=4` instead free-runs the sim across the whole bounce and compares the
outgoing velocity, allowing a shift of up to 3 ticks:

| population | n | err, no shift | aligned | mean impulse | accurate <50 |
|---|---|---|---|---|---|
| world bounces, scripted | 696 | 66 | 61 | 1 122 | **83%** |
| ...no-car `ball_*` only | 599 | 67 | 61 | 1 042 | **83%** |
| car-ball hits, scripted | 154 | 102 | 93 | 971 | 63% |
| ...`car_ball_*` only | 102 | 77 | 64 | 1 063 | 65% |
| world bounces, match replays | 1 044 | 1 291 | 1 239 | 1 556 | 31% |
| car-ball hits, match replays | 427 | 437 | 383 | 1 532 | 23% |

**The ball's core bounce physics is right.** Restitution 0.6, friction 0.35,
drag, spin and the 6.0 rad/s angular cap all check out: by scenario family the
flat-geometry cases are perfect -- `ceiling` 0% bad at a mean outcome error of
**0.9 uu/s against a 2 377 uu/s impulse**, `fast` 0% at 1.9 vs 2 661, `bounce`
0% at 2.2 vs 2 254, `max` 0% at 0.7, `roll` 0% at 7.3, `wall` 7% at 14.3.

#### What is left is the goal frame

| subject | n | bad >=50 | %bad | mean aligned err | mean sim contacts |
|---|---|---|---|---|---|
| `goal` | 138 | 48 | **35%** | **172.3** | 21.5 |
| `crossbar` | 4 | 2 | 50% | 312.7 | 2.0 |
| `post` | 5 | 2 | 40% | 52.8 | 2.6 |
| `corner` | 158 | 20 | 13% | 28.0 | 6.3 |
| `wall` | 67 | 5 | 7% | 14.3 | 3.8 |
| `chaos` | 53 | 2 | 4% | 4.3 | 1.6 |

Of the 104 genuinely-wrong bounces in the no-car recordings, **36 sit inside
the goal mouth and carry 60% of the total residual error**. The mechanism is
visible in one dump. `ball_goal_crossbar_drop_from_above` enters the goal at
`v = (0, 1487, -979)` and grazes the crossbar's rounded underside; RL removes
**241** uu/s of `+y`, then 301 on the next tick, then stops. The sim answers
with a near-pure `-y` wall normal `(0, -0.999, -0.044)` and reverses the ball
outright, `+1487 -> -360` -- roughly 19% of a bounce, delivered as 100% of one.
The sim's contact count explodes there too: up to **304 `BallHitWorld` events in
a 10-tick window** at `ball_goal_post_base_left`.

Note the contact normals in `BALLDUMP` are the *pre*-adjustment values.
`ArenaContactTracker::callback` deliberately pushes its record **before**
`adjust_internal_edge_contacts` mutates the manifold point, so what the dump
prints is the raw narrowphase normal and not necessarily what the solver used.
Do not conclude the internal-edge machinery is missing from a tilted normal in
a dump -- it is present and is a faithful port of
`btAdjustInternalEdgeContacts`.

#### Match replay ball records cannot grade anything

`RLBALL=2` also audits the recording against *itself*, with no simulation
involved. Bullet integrates `pos += vel * dt` with the post-step velocity, so
across a recorded step `(pos_to - pos_from) * 120` must equal `vel_to`:

| family | recs | ticks | bounces | inconsistent | worst | mean err |
|---|---|---|---|---|---|---|
| match replays | 10 | 98 819 | 2 013 | 2 036 (2.06%) | **631 156** | 169.578 |
| `ball_*` scripted | 143 | 42 297 | 879 | 416 (0.98%) | 2 313 | 2.199 |
| `car_ball_*` scripted | 84 | 15 636 | 356 | 269 (1.72%) | 1 912 | 2.940 |
| `mech_*` scripted | 17 | 4 633 | 94 | 66 (1.42%) | 1 207 | 2.582 |

**Read the magnitude, not the rate.** All four families disagree on roughly one
step per bounce, which is expected and is itself a finding: RL applies the
contact impulse *after* integrating the transform, so on a contact tick the
identity legitimately fails by about one impulse. The scripted worst cases
(2 313, 1 912, 1 207) are exactly that size. The match replays' **631 156 uu/s**
is not -- the ball's own speed cap is 6 000. At `2v2` t4178-t4182 the recorded
velocity flips sign three times in five ticks (`+2825 -> -2432 -> +955 ->
-2442`) while the recorded position moves smoothly and no car is within 900 UU
and no surface within 21 UU.

So **score the ball on the 244 scripted recordings and not on the match
replays**, which hold 78% of its raw error mass and cannot support any of it.
This is not the same defect as [[match-replays-duplicate-cars]] -- that one is
the *car* array collapsing; this is the ball's own velocity channel.

### The Flip Z-Damp Was Firing On Almost Everything (-13.4%, RLFLIP)

The largest single win so far. `RLCENSUS=10` bands every bucket by the recorded
jump/flip state, and one band carried **6.6% of the whole suite's error mass** by
itself:

| band | n | mean | p50 | p90 | p99 | bias fwd | bias up |
|---|---|---|---|---|---|---|---|
| `air_boost/djf/ft.15-.4` | 6 157 | 28.04 | 0.56 | 146.5 | 266 | -13.2 | -15.7 |
| `air_free/djf/ft.15-.4` | 13 072 | 7.38 | 4.66 | 6.32 | 145 | +0.08 | -0.17 |
| `air_boost/djf/ft0-.15` | 4 113 | 2.47 | 0.56 | 0.56 | 45.8 | | |
| `air_boost/djf/ft.4-.8` | 5 438 | 2.44 | 0.56 | 6.58 | 14.1 | | |
| `air_boost/djf/ft.8+` | 11 426 | 1.19 | 0.56 | 0.56 | 4.8 | | |

p50 0.56 against a mean of 28 is bimodal: most steps exact, an eighth wrong by
150-270 uu/s. And the window is not arbitrary -- `flip::Z_DAMP_START` is 0.15.
Inside `update_double_jump_or_flip` the sim does

```text
if is_flipping && flip_time in [Z_DAMP_START, TORQUE_TIME]
               && (lin_vel.z < 0.0 || flip_time < Z_DAMP_END)
{ lin_vel.z *= 1.0 - Z_DAMP_120; }      // *= 0.65, every tick
```

a 35% cut of vertical velocity per tick. Whether it fires depends entirely on
`is_flipping`, which the recording carries only as a one-tick activation pulse,
so `set_state_to_record_tick` has to reconstruct it. The old reconstruction
re-opened the flag whenever `|ang_vel.xy| > 2.0` and `0 < up.z < 0.9` inside the
window -- which is nearly every airborne tumbling car.

#### The closed channel

An isolated airborne car (no wheel contact, no ball, no car within 240 UU) has a
fully determined `vz`: gravity, boost along forward, and this damp. Ordering in
`pre_tick_update` is damp, then `update_boost`, then Bullet integrates gravity, so

```text
no damp:  vz_to == vz_from        + boost_z - g*dt
damped:   vz_to == vz_from * 0.65 + boost_z - g*dt
```

`RLFLIP=1` emits both residuals per step (and both boost orderings) and reads
Rocket League's own decision off the recording with no simulation involved.

#### Read it only where the two hypotheses are separated

**This is the trap, and it cost a wrong fix.** The two predictions differ by
`0.35 * vz_from`. Near the apex of a jump `vz_from` is small, the difference
collapses below float noise, and whichever residual happens to be marginally
smaller wins -- so *every* near-apex step gets a coin-flip label. Reading the
unfiltered table says RL damps 96.0% of falling steps out to `TORQUE_TIME`, which
is entirely that artifact:

| `\|vz_from\|` gate | `[0.21,0.65)` falling, damp% |
|---|---|
| none | 97.5% |
| > 50 | 3.9% |
| > 150 | 0.5% |

Acting on the unfiltered version -- widening the reconstruction window to
`TORQUE_TIME` and dropping the `up.z` lower bound -- measured **+3.56%** on the
suite and was reverted. Filter to `0.35 * |vz_from| > 20 uu/s` before reading any
of these rows.

#### What Rocket League actually does

On the 77 511 sharp samples:

| `flip_time` | vz | n resolved | damp% |
|---|---|---|---|
| < 0.15 | either | 7 462 | 0.1% |
| **0.15-0.21** | vz < 0 | 256 | **51.6%** |
| **0.15-0.21** | vz >= 0 | 2 220 | **36.0%** |
| 0.21-0.65 | vz < 0 | 846 | 2.0% |
| 0.21-0.65 | vz >= 0 | 9 563 | 0.0% |
| > 0.65 | either | 52 317 | 0.0% |

So the window really is `[0.15, 0.21]` and nothing else -- the sim's `vel.z < 0`
extension out to 0.65 does not correspond to anything RL does. Inside the window
RL damps only **38%**, while the old gate damped essentially all of it. The
dominant failure was the sim damping when RL did not: 1 545 steps at mean 110.5.

#### Saturation of the angular speed cap is the discriminator

First cut, on `|ang_vel.xy|` against `car::MAX_ANG_SPEED` = 5.5:

| tumble | n | damp% |
|---|---|---|
| <= 2.0 | 374 | 2.4% |
| 2.0-5.0 | 1 313 | 14.7% |
| 5.0-5.45 | 476 | 89.1% |
| >= 5.45 | 313 | 97.4% |

`tumble >= 5.0` costs **24 237** against 330 414 for damping on every step the
old gate allowed and 47 429 for never damping. The `up.z` test earns nothing once
the spin is gated (24 126 with it) and is removed.

But `|ang_vel.xy|` is the wrong statistic twice over. It is a *world*-frame
projection, so it under-reads by exactly the vertical spin component and misses
any flip carrying yaw; and "spinning hard" is not the physical criterion. A
flip's dodge torque (`TORQUE_X` 260 / `TORQUE_Y` 224) **saturates** the angular
speed cap for the whole dodge, while air roll on weaker torque (130/95/400
against `air_control::DAMPING`) settles below it. Testing the full angular speed
for saturation:

| `\|ang_vel\|` | n | damp% |
|---|---|---|
| >= 5.4999 | 731 | 94.5% |
| 5.499-5.4999 | 88 | 84.1% |
| 5.49-5.499 | 80 | 86.2% |
| 5.45-5.49 | 14 | 28.6% |
| 5.0-5.45 | 127 | 17.3% |
| < 5.0 | 1 436 | 4.9% |

Cost falls to **16 590** at `>= MAX_ANG_SPEED - 0.01`, a further -31.6%. It is a
real minimum, not a run to the cap: 18 407 at 5.44, 17 131 at 5.48, 16 590 at
5.49, back up to 17 224 at 5.495 and 21 531 at 5.4999 as genuine flips sitting a
hair under the cap start being excluded. The suite agrees independently, which is
the check that matters on the epsilon -- means 2.5956 at 0.05, 2.5951 at 0.02,
2.5953 at 0.01, 2.5983 at 0.005, 2.6119 at 0.001.

Aligning the spin with the recorded `flip_rel_torque` axis was tried too and is
not worth its complexity: as a refinement on top of the saturation test it makes
no difference, and on its own it is worse (`|ang_vel . axis| >= 5.2` costs
26 153). The axis is live and correct -- a clean side dodge reads `align` = 1.000
-- it just adds nothing the magnitude does not already carry.

#### Result

**Suite mean 2.9981 -> 2.5953, -13.4%** over the two steps (the tumble gate gave
-12.7% and the saturation refinement a further -0.83%), gate unchanged at 45
passing.

| bucket | mass before | mass after | change |
|---|---|---|---|
| `air_boost` | 264 715 | 115 742 | **-56%** (mean 5.16 -> 2.26) |
| `air_free` | 249 381 | 223 229 | -10.5% |
| `car_proximity` | 116 208 | 104 913 | -9.7% |
| `wall_ceiling` | 216 116 | 216 243 | +0.06% |

Everything grounded is untouched to the digit, which is the check that this only
moved airborne physics.

Note the closed channel over-predicted the second step badly -- it measured
-31.6% on the damp term but the suite moved -0.83%. That is not a contradiction:
the channel measures `vz` alone on the isolated airborne population, while the
census measures full 3-D `|dv|` over every air step, and after the first fix the
flip residual was already a small share of it. **Size a refinement on the suite,
not on the channel that found it.**

#### Still open

The saturation test is 86-95% right above the threshold and 5% below it, so what
remains is the genuinely ambiguous middle: a flip whose spin has already been
pulled off the cap by air control, and a double jump that is not a flip at all.
`has_flip` and the `is_flipping` pulse are dead on every one of these steps, so
the recording carries no direct signal; `flip_rel_torque` is live but adds nothing
beyond the magnitude. Absent a new field this looks close to the floor.

Worth remembering that `RLCENSUS=3` shows the airborne error concentrated on the
`hb-ramp` band, where a handbrake means nothing physically. That is not a
handbrake defect -- the powerslide button *is* the air-roll button, so mid-ramp
airborne is simply a proxy for "the player is air-rolling", which is what made the
old gate misfire.

### Why The 2 s Rollout Collapses, And Where The Signal Is

The duplicated-car fix gives a clean controlled experiment on this, because
`RLKEEPDUP=1` reproduces the pre-fix behaviour exactly. Same command
(`RLROLL=2s RLROLL_STRIDE=10`), same starts, one variable. Car velocity error,
n-weighted across ~47 000 rollouts:

| horizon | with corrupt steps | voided | change |
|---|---|---|---|
| t1 | 7.10 | 6.36 | **-10.4%** |
| t30 (0.25 s) | 44.08 | 41.44 | -6.0% |
| t60 (0.5 s) | 81.96 | 79.18 | -3.4% |
| t120 (1.0 s) | 184.70 | 182.93 | -1.0% |
| t180 (1.5 s) | 301.34 | 301.36 | **+0.0%** |
| t240 (2.0 s) | 414.50 | 414.49 | **-0.0%** |

Car position is the same story more sharply: t1 0.25 -> 0.07 (-72%), t30 8.41 ->
5.71 (-32%), t60 21.47 -> 17.98 (-16%), t120 -3.4%, then t180 and t240 identical
to five significant figures.

**A 20% improvement in per-tick accuracy buys exactly nothing at 2 s.** That is
the answer to why the long window collapses: by 1.5 s the trajectory retains no
information about how accurate the steps were. The error has reached the scale of
the quantity itself -- 414 uu/s against car speeds around 1400 -- so it is
saturation, and no per-tick fix can move it. Do not score a physics change on a
1 s or 2 s rollout mean; it is measuring the Lyapunov exponent, not the physics.

**The usable horizon is t1 to t60.** The fix's benefit decays smoothly from -72%
at t1 to -3% at t120 and zero beyond, so 0.25-0.5 s is where a per-tick change is
still visible with the horizon's extra sensitivity to *timing* rather than just
magnitude.

#### What actually drives the compounding

Splitting car velocity error by recording family, at t60 where sample retention
is still 67% or better:

| family | t1 | t30 | t60 | t240 |
|---|---|---|---|---|
| `car_*` (single car, scripted) | 2.6 | 13.6 | **19.0** | 22.7 |
| `mech_*` (single car + ball) | 2.4 | 19.3 | **38.0** | 403.6 |
| `car_car_*` (two cars, scripted) | 7.3 | 41.0 | **83.9** | 427.6 |
| `2v2*` / `3v3` (match) | 6.8 | 44.4 | **84.2** | 417.5 |

**At half a second: one car alone is 19 uu/s, add a ball and it is 38, add a
second car and it is 84.** So the compounding is driven by contact events, not by
a systematic force error in free motion -- which agrees with free flight being
essentially exact per-tick. Note `car_*` here *excludes* `car_car_*`; an earlier
measurement globbed them together, which is why it read the single-car family as
converging with the rest.

Do not read the `car_*` row past t60. Its n falls from 4322 to 816 at t180 and
300 at t240 because single-car recordings are short, and the error *decreases*
from 49.9 to 22.7 across that -- pure survivorship, only the calmest recordings
are long enough to reach 2 s. The other three families keep 33 000+, 1 440 and
228 samples respectively.

#### `car_car_*` at t30-t60 is the instrument for car-car work

It is the one place with all four properties at once: provably clean ground truth
(every scripted recording is at exactly 0.0000 frame desync), a healthy sample
(n = 2 130 at t30, 2 028 at t60), a horizon where per-tick accuracy still shows,
and sensitivity to contact *timing* that the per-tick census bucket does not have.
The per-tick `car_proximity` bucket is only 8.4% of mass at a mean of 7.40; this
sees the same physics at 41-84 uu/s.

#### Ruled out: identity relabelling after a collapse

Worth recording because it was the obvious suspect and it is wrong. The array
collapse leaves one canonical slot holding a copy of another car, so that slot's
track in `car_order::reorder_cars` carries the wrong position; when the real car
reappears the matcher could plausibly leave the two permanently transposed. A
rollout would be devastated by that -- it restores once and runs, so after a
transposition it compares against a different physical car thousands of UU away --
while per-tick measurement would be blind, since both ends of every later step are
relabelled consistently and the single transition step is voided by the 100 UU
displacement test.

`RLCARC=4` counts transpositions across the whole suite: **zero.** Out of 4 843
ticks where some but not all live cars jump more than 100 UU, 7 are full resets
and none is a two-car swap. Those 4 843 are the collapse *boundaries* -- a slot
receiving a copy of a car 3 200 UU away jumps hugely, then jumps back -- and
`has_discontinuity` was already voiding them on displacement. The fix above caught
the smooth *interior* of each collapse, which is what displacement could not see.

## Rollout Mode (RLROLL) — compounding errors

The per-tick pass restores full state every tick, so it only ever measures one
physics step in isolation. Rollout mode restores the recorded state at sampled
start ticks, then free-runs the sim for N ticks with the recording's controls,
measuring every step — an error-vs-time **growth curve** per entity+field.
Flat curve ≈ stable sim; climbing curve ≈ a compounding bug. This is how you
trace extended failures that only appear after several ticks.

```bash
# 1-second rollouts starting every 120 ticks of the recording:
RLGATE=off RLROLL=1s cargo test -p rocketsim case_3v3 -- --nocapture --test-threads=1

# 2-second rollouts, denser starts, combined with the C++ reference:
RLGATE=off RLROLL=2s RLROLL_STRIDE=60 RLCPP=1 \
  cargo test -p rocketsim --features cpp-compare case_3v3 -- --nocapture --test-threads=1
```

`ROLL` lines show the mean error at sampled horizons
(`t1 t5 t15 t30 t60 t120 …`) plus `worst_end=<mag>@start<tick>` — the start
tick of the worst rollout, ready for a deep dive:
`RLDEEP=always RLTICK=<start> cargo test case_<name>`.

With `RLCPP=1` the C++ sim runs the same rollouts and a `ROLLOUT-CPP-COMPARE`
table ranks the final-horizon errors. On 3v3 both sims compound at nearly the
same rate (ratio ≈ 1.0), i.e. the long-horizon divergence is a shared RocketSim
limit, not a port bug.

## Deep Dive (failure mode)

When a case fails (or `RLDEEP=always`), the harness re-drives a fresh arena
over a window around the worst tick and prints, per tick: the physics deltas,
the situation flags on **both** sides (`pred!real` when they disagree), and the
controls applied — plus a field-by-field `pred vs real` snapshot at the center.

```bash
# Deep-dive the worst tick of car 2 in a 3v3 recording:
RLDEEP=always RLCAR=2 cargo test -p rocketsim case_drive -- --nocapture --test-threads=1

# Deep-dive a specific tick you spotted in the report:
RLDEEP=always RLTICK=534 cargo test -p rocketsim case_jump_1 -- --nocapture --test-threads=1
```

The case name is the "UUID": `case_<name>` maps 1:1 to `<name>.rlpr`.

This is how you localize a bug. Example from a jump recording: airborne ticks
show `vel_e≈0.007` (perfect), then at the landing tick `ground` flips 0→1 and
`vel_e` jumps to 142 — the snapshot shows the Z-velocity mismatch, pointing
straight at the landing/jump-impulse transition rather than general physics.

## Per-Situation Breakdown

`RLSEG=1` splits each field's error by the recording's own regime flags
(`grounded`/`airborne`, `boosting`, `jumping`, `flipping`, `supersonic`). This
localizes error to a subsystem: large `vel` error only while `grounded`+
`boosting` ⇒ ground friction/boost; only while `airborne` ⇒ air control.

```bash
RLGATE=off RLSEG=1 cargo test -p rocketsim case_drive_5 -- --nocapture --test-threads=1
```

## Per-Car Testing

By default every car (and the ball) is measured and gated. To isolate one car
in a multi-car recording (e.g. something looks fishy on car 2 of a 3v3):

```bash
RLCAR=2 RLGATE=off cargo test -p rocketsim case_drive -- --nocapture --test-threads=1
```

## Performance / Long Replays / Parallelism

The measurement hot path computes raw deltas directly (no per-tick hash maps),
and the per-tick pass is **embarrassingly parallel**: because state is restored
from ground truth every tick, ticks are independent, so the range is sharded
across `RLTHREADS` workers, each owning its own arena, and the stats are merged.

Measured for a 20-minute (144k tick) 3v3 replay, `cargo test` profile:

| threads | time | |
|---|---|---|
| 1 | ~44 s | |
| 8 | ~6.5 s | 6.8x |
| 16 | ~5 s | 8.8x |

Single-car replays are far cheaper (~75 MB to parse, ~1 s per pass). Memory is
≈ file size (≈520 B/tick per car), so a 3v3 20-min recording is ≈380 MB.

The continuous pass is sequential by nature (errors compound), so it is opt-in
via `RLCONT=1` to keep the default fast path to one parallel pass.

### Determinism

- The **gate (percentile)** is bit-identical across thread counts and runs.
- `RLTHREADS=1` is fully deterministic for every reported number.
- Every tick in range is measured at every thread count. Each shard warms its
  cold arena by re-stepping its own first tick and discarding the result, so
  manifold settling costs no coverage. (It used to consume the first two
  *measured* ticks of each shard, which hid real divergence — including a
  7.5 UU/s jump-impulse error at t=0 that let a case pass at `max=0.014` — and
  is what made the numbers move with the thread count.)
- Under multiple threads, `mean`/`rms` still carry a tiny amount of variance:
  bullet keeps stateful suspension/solver state across `step_tick` that a
  state-restore does not reset, so the arena state entering a shard-start tick
  differs from a mid-shard tick. This does **not** affect the gate or `max`. For
  bit-exact reproduction of a specific number, use `RLTHREADS=1`.
- The deep dive always re-drives single-threaded, so it is deterministic.

`examples/bench_long_replay.rs` reproduces this loop to benchmark any
tick/car/thread combination:
```bash
cargo run -p rocketsim --profile test --example bench_long_replay -- --ticks 144000 --cars 6 --threads 16
```

## Workflow To Improve Accuracy

1. `RLGATE=off cargo test ... --nocapture` — survey. Sort by `vel`/`pos` p95.
2. Pick the worst case + field. Note `bias` vs `rms` and run `RLSEG=1` to find
   the regime.
3. `RLDEEP=always RLCAR=<i> cargo test case_<name>` — read the window + center
   snapshot to find the physical cause.
4. Fix the sim.
5. Re-run; confirm that case's p95 dropped. Repeat until the gate is green.

## Recording Ground Truth (Windows + cross-compiled DLL)

### Build the logger (Linux → Windows cross-compile)

```bash
cd tools/bakkesmod_physics_logger
cmake --build build-clang
cp build-clang/PhysicsLogger.dll \
  "/home/simme/Games/Heroic/Prefixes/Rocket League/.../plugins/"
```

Or native MSVC build on Windows:

```powershell
cd tools\bakkesmod_physics_logger
cmake -B build -G "Visual Studio 17 2022" -A x64
cmake --build build --config Release
```

### Record in Rocket League

BakkesMod console (F6):
```
plugin load PhysicsLogger
physlog_record_replay       # start observer
...play normally...
physlog_stop_replay         # writes .json
physlog_write_rlpr output   # converts to .rlpr (auto-versioned)
```

RLPR files output to `%APPDATA%\bakkesmod\bakkesmod\data\physics_ground_truth\`.

Copy them here:
```bash
cp <windows-path>/*.rlpr external/rocketsim-v3-rust/rocketsim/tests/rl_comparison_test/test_recordings/
```

Then rebuild and test:
```bash
rm -rf target/debug/build/rocketsim-* && cargo test -p rocketsim
```

Record at **120 Hz** to match the sim tick rate.

## Dealing with Corrupted Recordings

Old recordings (pre-v4 logger) may have:
- **Garbage rotation matrices** — caught by Rust's orthogonality/unit-length
  assertions at parse time. Move to `test_recordings_corrupt/`.
- **`jump_time` / `flip_time` always 0** — old logger didn't track timing.
  Re-record with the current logger.

## Test Organization

```
rocketsim/tests/
  mod.rs                          → entry point
  rl_comparison_test/
    mod.rs                        → thin orchestrator (run case, gate, dive)
    config.rs                     → env-var knobs (RLDEEP, RLCAR, RLTICK, …)
    tolerance.rs                  → per-field physics budgets
    validate.rs                   → ground-truth validity checks
    impulse_window.rs             → two-step jump/flip impulse windows
    recording/normalize.rs        → observer-semantics → sim-semantics
    stats.rs                      → Field/Segment vocab + running/percentile stats
    measure.rs                    → lean per-tick deltas (hot path, no alloc)
    state.rs                      → state-machine mismatch-rate channel
    report.rs                     → aggregation, gate eval, printing
    deep_dive.rs                  → N-tick context-window dump
    runner.rs                     → state restore + arena driving loop
    compare.rs                    → rich pred-vs-real snapshot (deep dive)
    recording/
      mod.rs                      → RLPR binary parser + stride detection
      cpp_records.rs              → binary struct layouts + CarRecord→CarState
      data_reader.rs              → binary reader helpers
      tick_record.rs              → per-tick record container
    test_recordings/*.rlpr        → ground truth recordings (auto-discovered)
    test_recordings_corrupt/      → old recordings with bad rotation data
    test_recordings_invalid/      → rejected by validate.rs (pinned cars)
```

## Recent Physics Changes (2024–2025)

### 2025-07 — Test infrastructure fixes (Reasonix session)
- **Rotation matrix transpose bug** — `From<PhysRecord> for PhysState` used
  `from_cols(column(0), column(1), column(2))` which transposed the stored
  rows-as-axis-vectors into columns-as-X-components. Every direction-dependent
  force (throttle, air control, jump, boost) was applied along the wrong axis.
  Fixed by passing rows directly: `from_cols(rows[0], rows[1], rows[2])`.
  This was the root cause of the massive per-tick velocity errors (60–1000+).
- **`euler_to_mat3` Up-row sign errors** — The C++ logger's rotation matrix
  computation had `+sr·sy` and `−sr·cy` in the Up row where UE's FRotationMatrix
  uses `−sr·sy` and `+sr·cy`. Fixed in `RlprWriter.h`, DLL rebuilt.
- **Init race** — `HAS_INITIALIZED_LOCK.set(())` was called before mesh loading
  completed. If loading failed, subsequent callers saw `is_initialized() == true`
  but `ARENA_COLLISION_SHAPES` was `None`. Fixed by deferring the set().
- **Jump state machine** — `is_jumping` now ends immediately on landing
  (`!is_on_ground` guard in Phase B), matching RL behavior.
- **Wheel raycast clearing** — `Car::set_state` now calls
  `reset_wheel_suspension()` on all wheels to prevent stale contact normals
  from being used for friction computation after state restore.

### 2024 — Original v4 changes
- **Jump impulse moved to activation tick** — no longer delayed by 1 tick after
  press. Applied immediately in `update_jump`'s activation block with
  `jump_time = TICK_TIME` to skip double-fire on the next tick.
- **`update_double_jump_or_flip` now checks `!is_on_ground`** — prevents
  double-jump/flip code from firing on the first ground-based jump press.
- **State restoration fixes** — `set_state_to_record_tick` restores
  `is_on_ground`, `prev_controls`, `has_flipped`, `has_double_jumped`.
- **`IMMEDIATE_FORCE` = 875/3 ≈ 292** (unchanged) — RL spreads this over 2
  ticks (~108 + ~184) via the logger, but the test restores state between
  ticks so the spread doesn't persist.

Clean rebuild after adding new recordings:

```bash
rm -rf target/debug/build/rocketsim-* && cargo test -p rocketsim
```
