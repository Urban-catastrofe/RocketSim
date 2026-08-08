# CHANGES — KiLearnBox fork of RocketSim (Rust)

This crate is a Rust port of <https://github.com/ZealanL/RocketSim> (C++,
originally by ZealanL, ported to Rust by VirxEC). It is vendored into
KiLearnBox as the **Rust RocketSim backend** and now carries a number of
KiLearnBox-specific changes for chasing real-RL accuracy against the RLPR
replay suite (470 cases, 120 Hz, BakkesMod logger).

- Version: **3.0.0** (fork line; upstream C++ RocketSim is v2.2.1)
- Fork base: `Urban-catastrofe/RocketSim` @ `v3-rust` branch
- Divergence: 11 commits (`my-mods` branch) + uncommitted physics fixes below
- Accuracy harness: `rocketsim/tests/rl_comparison_test/` (per-tick,
  state-restore comparison against `.rlpr` recordings; C++ side-by-side via
  the `cpp-compare` feature and `RLCPP=1`, using the crates.io `rocketsim_rs`
  prebuilt as the C++ reference).

## Accuracy fixes (most recent first)

### ball: clamp angular velocity to BALL_MAX_ANG_SPEED (6.0 rad/s)
`rocketsim/src/sim/ball/base.rs` — `finish_physics_tick` now clamps the ball's
angular velocity to `consts::ball::MAX_ANG_SPEED` (6.0) and its linear velocity
to `mutator_config.ball_max_speed` (6000 UU/s), matching C++
`Ball::_FinishPhysicsTick`. Without the clamp, wall/backboard/goal contact
friction overshot the real spin by up to 8 rad/s in a single tick (e.g. rolling
down the backboard pinned at the real 6.0 clamp jumped to 14.06). Result: all
468 ball ang_vel p95 cases now match C++ exactly
(`car_ball_backboard_clear` 3.26 → 0.0000, `ball_into_goal` 0.82 → 0.011).

### car: wheel friction on non-static contacts (wall dashes, flip resets)
`rocketsim/src/sim/car/base.rs` — `update_wheels` now computes lat/long
friction for ANY wheel raycast hit (including dynamic bodies like the ball),
matching C++ `Car::_UpdateWheels`. Previously the Rust skipped non-static
contacts, leaving the wheel friction at its default of 1.0.
`rocketsim/src/bullet/dynamics/vehicle/wheel_info.rs` — the wheel lat/long
friction defaults are now 0.0 (matching C++ `btVehicleRL`), so the first
contact tick applies zero friction instead of full. Together these remove the
~13.6 UU/s sideways shove on the upside-down car during flip-reset / wall-dash
wheel-ball contact. `mech_flip_reset_simple` car_0 vel p95 13.83 → 1.556
(C++ 1.544); aggregate car-vel p95 sum now **below** C++ (3249.5 vs 3276.5).
Wall-dash flip/curve impulses remain byte-identical to C++ (shared limits).

## Earlier changes (from the `my-mods` branch + prior work)

- **Car-car physical softening** (`solver_constraint.rs`, `consts.rs`):
  head-on car-car resolves via bump impulse only (speed-dependent
  `= 500/|approach|` clamp [0,1]); `RL_CAR_SOFTEN` env override.
- **Jump immediate-force timing** (`runner.rs` gate): the `jump_time=TICK_TIME`
  bump fires only when `cs.phys.vel.z > 100` (matches) vs unconditional.
- **Car-ball extra impulse** (`ball/base.rs`, `ball_hit/config.rs`): applied at
  `finish_physics_tick` end-of-tick; default cadence `OncePerEpisode`.
- **CMake Rust backend** (`KILEARN_USE_ROCKETSIM_RUST`): `rocketsim-bridge`
  FFI remaps (`supersonic_grace_timer`, `ball_hit.last_impulse_tick`,
  `bump_other_car_id`), `set_mutator_config`/`set_gravity` re-added,
  `BallArena::predict_ball_into` + 4-arg `PredictBall` out-buffer.
- **Testing infrastructure**: per-tick RLPR harness, deep-dive (RLDEEP),
  residual decomposition (RLRESID), C++ comparison (RLCPP=1), 470 recordings.

## How to run the harness

```bash
# One case (deep dive around the worst ticks)
RLGATE=off RLDEEP=always cargo test -p rocketsim case_car_ball_backboard_clear -- --nocapture --test-threads=1

# Full survey
RLGATE=off cargo test -p rocketsim case_ -- --test-threads=8 --nocapture

# C++ side-by-side (requires the cpp-compare feature)
RLGATE=off RLCPP=1 cargo test -p rocketsim --features cpp-compare case_mech_flip_reset_simple -- --nocapture --test-threads=1
```