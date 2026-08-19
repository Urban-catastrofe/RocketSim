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

> ⚠️ The current recordings are v2 (no hit records). Re-record with the v3
> logger (`tools/bakkesmod_physics_logger`, `RlprWriter.h` now writes v3). The
> parser is version-aware — v2 recordings still work.

## The Car-Ball Impulse Subsystem

The extra hit impulse lives in `rocketsim/src/sim/ball_hit/` as a pure,
config-driven module:

- `config.rs` — `BallHitConfig` (all tunables: z_scale, forward_scale, factor
  curve, max_delta_vel) + `HitCadence` enum (`EveryTick` / `EveryOtherTick` /
  `OncePerEpisode`). The default preserves legacy behavior exactly.
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
separates), which beats the C++'s `EveryOtherTick` on sustained-contact cases
(`car_ball_soft_touch` 6.5 vs C++ 55.8, dribble 13.8 vs 17.7). Tradeoff: the
game spreads each hit over **2 ticks** (~35/65 split) in the recording, while
the sim (and C++) fire once — so per-tick cases dominated by that spread
(`car_ball_backwall_car_ball_approach`, `mech_flip_reset_simple`) score
slightly worse per-tick even though the *total* impulse matches the game. The
2-tick split is **not modelable** (verified 2026-08): in the per-tick restore
harness the 2nd half double-counts (the restore already contains the game's
recorded 2nd-half velocity), and in the continuous pass it has a negligible
trajectory effect (a 1000 UU/s impulse delivered in one tick vs split 20/80
over two ticks converges in velocity immediately, with a ~1.7 UU max position
offset — measured with a dedicated test). Modeling it would not improve
accuracy.

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
  two steps of ordinary integration error it contains).

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
mask, because the recording's left/right order within a pair cannot be pinned
down: `{0,2}` and `{1,3}` are the two common side-lift masks in near-equal
numbers (3148 vs 3108), `steer_amount` is never populated so the front pair
cannot be identified from it, and the sim's own `right_dir_2d` naming
contradicts the right-handed forward/up convention. Front/back grouping *is*
certain, and a per-pair count is what `n >= 3` turns on. A left/right lift swap
is the one blind spot.

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
