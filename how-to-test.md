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

## The Accuracy Bar

A case **passes** when no single simulated step diverges from the recording by
more than **0.03** in the field's native unit — 0.03 UU of position and
0.03 UU/s of velocity — for *every* entity on *every* tick. The gate is a
maximum, not a percentile (`percentile = 1.0`): one bad step fails the case.

Because the per-tick pass restores ground truth before every step, this measures
single-step physics error in isolation, which is exactly "per step divergence".

This is a deliberately hard bar. As of the last full survey (468 cases,
683k entity-tick samples) **41 cases pass**; contactless motion (aerials, air
roll, free-flight and slow-rolling ball) sits at 0.006–0.025, while anything
involving a collision or a car-ball impulse spikes on the contact tick — a plain
`ball_bounce_ground` keeps 97.4% of ticks under 0.03 UU but hits 3.18 UU on the
bounce. Use `RLGATE=off` with loosened `RL_*_TOL` when you need a metric that
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
- Under multiple threads, *secondary* stats (mean/rms/max) carry a tiny amount
  of variance: the bullet physics vehicle keeps stateful suspension/solver
  state across `step_tick` that a state-restore does not reset, so a shard's
  first ticks run on a "cold" arena. This does **not** affect the gate. For
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
