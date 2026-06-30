# How to Run RL Comparison Tests

## Quick Start

```bash
# From external/rocketsim-v3-rust
cargo test -p rocketsim -- --nocapture --color never 2>&1 | grep "norm_error\|\.\.\. ok"

# PASS/FAIL summary:
cargo test -p rocketsim -- --nocapture --color never 2>&1 | grep -E "^test |^test result"

# Sorted norm errors (physics divergence per recording):
cargo test -p rocketsim -- --nocapture --color never 2>&1 | grep "norm_error" | sort -t= -k2 -n
```

## Run a Single Recording

```bash
# By exact name (snake_case of the .rlpr filename)
cargo test -p rocketsim -- rl_comparison_test::case_jump_test -- --nocapture --color never

# Pattern match:
cargo test -p rocketsim -- jump -- --nocapture --color never
cargo test -p rocketsim -- drive -- --nocapture --color never
cargo test -p rocketsim -- ball_touch -- --nocapture --color never
```

## Understanding Results

Each `.rlpr` file in `test_recordings/` generates a test case automatically
(build.rs scans the directory at compile time). The test:

1. Restores arena state from the recording's tick N (including jump/flip state)
2. Uses controls from tick N+1's recording
3. Steps the physics by one tick
4. Compares the result to the recording's tick N+1

**Known issue:** `accum_lin_vel` is not cleared by `set_car_state()` between
ticks, causing force carryover on the second tick after jump activation.
This inflates norm_error for the tick immediately following a jump.

A `norm_error` is the RMS of all per-field relative errors:

| norm_error | Meaning |
|---|---|
| **< 1.0** | **PASS** — within threshold |
| **1.0–2.0** | Close — small physics differences |
| **2.0–10.0** | Moderate — jump/landing divergence |
| **> 10.0** | Significant — bug or missing feature |

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

## Recent Physics Changes (2024)

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
