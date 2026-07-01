# How to Run RL Comparison Tests

## Quick Start

```bash
# From external/rocketsim-v3-rust
# Run all tests (each test prints per-tick + continuous stats):
cargo test -p rocketsim -- --show-output --color never --test-threads=1

# Run by category:
cargo test -p rocketsim -- jump -- --show-output --color never
cargo test -p rocketsim -- drive -- --show-output --color never
cargo test -p rocketsim -- air -- --show-output --color never
```

## Run a Single Recording

```bash
# By exact name (snake_case of the .rlpr filename)
cargo test -p rocketsim -- case_jump -- --show-output --color never

# Pattern match:
cargo test -p rocketsim -- drive_5 -- --show-output --color never
```

## Understanding Results

Each `.rlpr` file in `test_recordings/` generates a test case automatically
(build.rs scans the directory at compile time). Each test runs **two modes**
and prints a STAT line:

### Per-tick mode (state restored every tick)
1. Restores arena state from the recording's tick N
2. Sets controls from tick N+1
3. Steps the physics by one tick
4. Compares the result to the recording's tick N+1

This mode isolates single-tick physics accuracy. The STAT line shows:
- `max=X@t=Y` — worst norm_error and the tick where it occurred
- `avg` — average norm_error across all ticks
- `v` — maximum raw velocity delta (UU/s) between sim and RL
- `p` — maximum raw position delta (UU)

### Continuous mode (state set only at tick 0, then runs freely)
The same as per-tick but state is never restored after tick 0 — errors compound.
Capped runs at 120/240/480/960 ticks show how fast divergence grows.

A `norm_error` is the RMS of all per-field relative errors:

| norm_error | Meaning |
|---|---|
| **< 1.0** | **PASS** — within threshold |
| **1.0–2.0** | Close — small physics differences |
| **2.0–10.0** | Moderate — jump/landing divergence |
| **> 10.0** | Significant — bug or missing feature |

### Per-tick error budget (post transpose fix, 2025-07)

Most tests have per-tick max < 10. The remaining ~50–160 norm_error spikes come from:
- **Jump activation velocity** (~50): RL's 2-tick impulse split (~108+184 UU/s)
  can't be expressed as a single-tick impulse when state is restored every tick
- **Airborne jump state timing** (~4): `is_jumping` persists while `controls.jump`
  is held in the sim; RL ends it on liftoff
- **Drive friction** (~4–115): small lateral velocity overestimation during
  ground driving, most visible in long supersonic coasting scenarios

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
    mod.rs                        → test runner + per-tick comparison
    compare.rs                    → per-field comparison (pos, vel, rot, etc.)
    recording/
      mod.rs                      → RLPR binary parser
      cpp_records.rs              → binary struct layouts + CarRecord→CarState
      data_reader.rs              → binary reader helpers
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
- **Test harness improvements**:
  - `wheels_with_contact` derived from recording's `is_on_ground`
  - `jump_time` convention conversion (RL's 0 → sim's MIN_TIME)
  - `air_time_since_jump` closed to prevent false flip activations
  - Added continuous mode with capped runs (120/240/480/960 ticks)

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
