mod arena_wrapper;
mod conversions;

use arena_wrapper::{RsArena, RsBallArena};
use conversions::*;

#[cxx::bridge]
pub mod ffi {
    // ==================== Shared Types ====================
    // These are visible to both Rust and C++, passed by value.

    #[derive(Clone, Copy, Debug, Default)]
    struct FfiVec {
        x: f32,
        y: f32,
        z: f32,
    }

    #[derive(Clone, Copy, Debug, Default)]
    struct FfiRotMat {
        forward_x: f32,
        forward_y: f32,
        forward_z: f32,
        right_x: f32,
        right_y: f32,
        right_z: f32,
        up_x: f32,
        up_y: f32,
        up_z: f32,
    }

    #[derive(Clone, Copy, Debug, Default)]
    struct FfiPhysState {
        pos: FfiVec,
        rot_mat: FfiRotMat,
        vel: FfiVec,
        ang_vel: FfiVec,
    }

    #[derive(Clone, Copy, Debug, Default)]
    struct FfiCarControls {
        throttle: f32,
        steer: f32,
        pitch: f32,
        yaw: f32,
        roll: f32,
        jump: bool,
        boost: bool,
        handbrake: bool,
    }

    #[derive(Clone, Copy, Debug, Default)]
    struct FfiCarState {
        phys: FfiPhysState,
        controls: FfiCarControls,
        prev_controls: FfiCarControls,
        is_on_ground: bool,
        wheels_with_contact_0: bool,
        wheels_with_contact_1: bool,
        wheels_with_contact_2: bool,
        wheels_with_contact_3: bool,
        has_jumped: bool,
        has_double_jumped: bool,
        has_flipped: bool,
        flip_rel_torque: FfiVec,
        jump_time: f32,
        flip_time: f32,
        is_flipping: bool,
        is_jumping: bool,
        air_time: f32,
        air_time_since_jump: f32,
        boost_amount: f32,
        time_since_boosted: f32,
        is_boosting: bool,
        boosting_time: f32,
        is_supersonic: bool,
        supersonic_time: f32,
        handbrake_val: f32,
        is_auto_flipping: bool,
        auto_flip_timer: f32,
        auto_flip_torque_scale: f32,
        bump_cooldown_timer: f32,
        has_world_contact: bool,
        world_contact_normal: FfiVec,
        is_demoed: bool,
        demo_respawn_timer: f32,
        bump_cooldown_other_car_idx_valid: bool,
        bump_cooldown_other_car_idx: u32,
        last_extra_hit_tick_valid: bool,
        last_extra_hit_tick: u64,
    }

    #[derive(Clone, Copy, Debug, Default)]
    struct FfiBallState {
        phys: FfiPhysState,
        hs_y_target_dir: i8,
        hs_cur_target_speed: f32,
        hs_time_since_hit: f32,
        ds_charge_level: i32,
        ds_accumulated_hit_force: f32,
        ds_y_target_dir: i8,
        ds_has_damaged: bool,
        ds_last_damage_tick: u64,
        tick_count_since_kickoff: u64,
        last_extra_hit_tick_valid: bool,
        last_extra_hit_tick: u64,
    }

    #[derive(Clone, Copy, Debug, Default)]
    struct FfiBoostPadState {
        cooldown: f32,
        is_active: bool,
    }

    #[derive(Clone, Copy, Debug, Default)]
    struct FfiBoostPadConfig {
        pos: FfiVec,
        is_big: bool,
    }

    #[derive(Clone, Copy, Debug, Default)]
    struct FfiCarInfo {
        idx: u32,
        team: u8,
    }

    #[derive(Clone, Copy, Debug, Default)]
    struct FfiCarBodyConfig {
        hitbox_size: FfiVec,
        hitbox_pos_offset: FfiVec,
        front_wheel_radius: f32,
        front_suspension_rest: f32,
        front_connection_offset: FfiVec,
        back_wheel_radius: f32,
        back_suspension_rest: f32,
        back_connection_offset: FfiVec,
        three_wheels: bool,
        dodge_deadzone: f32,
    }

    #[derive(Clone, Copy, Debug, Default)]
    struct FfiArenaConfig {
        mem_weight_mode: u8,
        min_pos: FfiVec,
        max_pos: FfiVec,
        max_aabb_len: f32,
        no_ball_rot: bool,
    }

    #[derive(Clone, Copy, Debug, Default)]
    struct FfiMutatorConfig {
        gravity: FfiVec,
        car_mass: f32,
        ball_mass: f32,
        ball_max_speed: f32,
        ball_drag: f32,
        jump_accel: f32,
        jump_immediate_force: f32,
        boost_accel_ground: f32,
        boost_accel_air: f32,
        boost_used_per_second: f32,
        respawn_delay: f32,
        bump_cooldown_time: f32,
        car_max_boost_amount: f32,
        car_spawn_boost_amount: f32,
        boost_pad_amount_small: f32,
        boost_pad_amount_big: f32,
        boost_pad_cooldown_big: f32,
        boost_pad_cooldown_small: f32,
        ball_hit_extra_force_scale: f32,
        bump_force_scale: f32,
        bump_requires_front_hit: bool,
        ball_radius: f32,
        unlimited_flips: bool,
        unlimited_double_jumps: bool,
        recharge_boost_enabled: bool,
        recharge_boost_per_second: f32,
        recharge_boost_delay: f32,
        demo_mode: u8,
        enable_team_demos: bool,
        goal_base_threshold_y: f32,
    }

    #[derive(Clone, Copy, Debug, Default)]
    struct FfiCarHitBallEvent {
        car_idx: u32,
        contact_point: FfiVec,
        extra_hit_vel: FfiVec,
    }

    #[derive(Clone, Copy, Debug, Default)]
    struct FfiCarHitCarEvent {
        bumper_car_idx: u32,
        victim_car_idx: u32,
        contact_point: FfiVec,
        is_demo: bool,
    }

    #[derive(Clone, Copy, Debug, Default)]
    struct FfiBallHitInfo {
        is_valid: bool,
        relative_pos_on_ball: FfiVec,
        ball_pos: FfiVec,
        extra_hit_vel: FfiVec,
        tick_count_when_hit: u64,
        tick_count_when_extra_impulse_applied: u64,
    }

    // ==================== Opaque Rust type ====================
    extern "Rust" {
        type RsArena;
        type RsBallArena;
    }

    // ==================== Bridge Functions ====================
    extern "Rust" {
        // Initialization
        fn rs_init(collision_meshes_path: &str, silent: bool) -> bool;

        // Arena lifecycle
        fn rs_arena_new(game_mode: u8) -> Box<RsArena>;
        fn rs_arena_new_with_config(game_mode: u8, config: &FfiArenaConfig) -> Box<RsArena>;
        fn rs_arena_new_prediction(game_mode: u8, tick_rate_scale: u32) -> Box<RsArena>;

        // Arena operations
        fn rs_arena_step(arena: &mut RsArena, ticks: u32);
        fn rs_arena_add_car(arena: &mut RsArena, team: u8, config: &FfiCarBodyConfig) -> u32;
        fn rs_arena_num_cars(arena: &RsArena) -> u32;
        fn rs_arena_num_boost_pads(arena: &RsArena) -> u32;
        fn rs_arena_is_ball_scored(arena: &RsArena) -> bool;
        fn rs_arena_reset_to_random_kickoff(arena: &mut RsArena, seed: i64);
        fn rs_arena_tick_count(arena: &RsArena) -> u64;
        fn rs_arena_game_mode(arena: &RsArena) -> u8;

        // Car state
        fn rs_arena_get_car_state(arena: &RsArena, car_idx: u32) -> FfiCarState;
        fn rs_arena_set_car_state(arena: &mut RsArena, car_idx: u32, state: &FfiCarState);
        fn rs_arena_set_car_controls(arena: &mut RsArena, car_idx: u32, controls: &FfiCarControls);
        fn rs_arena_get_car_info(arena: &RsArena, car_idx: u32) -> FfiCarInfo;
        fn rs_arena_get_car_body_config(arena: &RsArena, car_idx: u32) -> FfiCarBodyConfig;

        // Ball state
        fn rs_arena_get_ball_state(arena: &RsArena) -> FfiBallState;
        fn rs_arena_set_ball_state(arena: &mut RsArena, state: &FfiBallState);

        // Boost pad state
        fn rs_arena_get_boost_pad_state(arena: &RsArena, idx: u32) -> FfiBoostPadState;
        fn rs_arena_set_boost_pad_state(arena: &mut RsArena, idx: u32, state: &FfiBoostPadState);
        fn rs_arena_get_boost_pad_config(arena: &RsArena, idx: u32) -> FfiBoostPadConfig;

        // Config
        fn rs_arena_get_mutator_config(arena: &RsArena) -> FfiMutatorConfig;
        fn rs_arena_set_mutator_config(arena: &mut RsArena, config: &FfiMutatorConfig);

        // Events (from last step)
        fn rs_arena_get_car_hit_ball_events(arena: &RsArena) -> Vec<FfiCarHitBallEvent>;
        fn rs_arena_get_car_hit_car_events(arena: &RsArena) -> Vec<FfiCarHitCarEvent>;

        // BallHitInfo (synthesized per-car, for Player::UpdateFromCar compatibility)
        fn rs_arena_get_ball_hit_info(arena: &RsArena, car_idx: u32) -> FfiBallHitInfo;

        // Ball-only arena (lightweight, for ball trajectory prediction)
        fn rs_ball_arena_new(game_mode: u8, tick_rate: f32) -> Box<RsBallArena>;
        fn rs_ball_arena_step(arena: &mut RsBallArena, ticks: u32);
        fn rs_ball_arena_get_ball_state(arena: &RsBallArena) -> FfiBallState;
        fn rs_ball_arena_set_ball_state(arena: &mut RsBallArena, state: &FfiBallState);
    }
}

// ==================== Bridge Function Implementations ====================

fn rs_init(collision_meshes_path: &str, silent: bool) -> bool {
    rocketsim::init(collision_meshes_path, silent).is_ok()
}

fn rs_arena_new(game_mode: u8) -> Box<RsArena> {
    let gm = u8_to_game_mode(game_mode);
    let arena = rocketsim::Arena::new(gm);
    Box::new(RsArena::new(arena, 1))
}

fn rs_arena_new_with_config(game_mode: u8, config: &ffi::FfiArenaConfig) -> Box<RsArena> {
    let gm = u8_to_game_mode(game_mode);
    let cfg = ffi_to_arena_config(gm, config);
    let arena = rocketsim::Arena::new_with_config(cfg);
    Box::new(RsArena::new(arena, 1))
}

fn rs_arena_new_prediction(game_mode: u8, tick_rate_scale: u32) -> Box<RsArena> {
    let gm = u8_to_game_mode(game_mode);
    let arena = rocketsim::Arena::new(gm);
    Box::new(RsArena::new(arena, tick_rate_scale))
}

fn rs_arena_step(arena: &mut RsArena, ticks: u32) {
    arena.step(ticks);
}

fn rs_arena_add_car(arena: &mut RsArena, team: u8, config: &ffi::FfiCarBodyConfig) -> u32 {
    let t = u8_to_team(team);
    let cfg = ffi_to_car_body_config(config);
    let idx = arena.arena.add_car(t, cfg);
    arena.ensure_ball_hit_info_size();
    idx as u32
}

fn rs_arena_num_cars(arena: &RsArena) -> u32 {
    arena.arena.num_cars() as u32
}

fn rs_arena_num_boost_pads(arena: &RsArena) -> u32 {
    arena.arena.num_boost_pads() as u32
}

fn rs_arena_is_ball_scored(arena: &RsArena) -> bool {
    arena.arena.is_ball_scored()
}

fn rs_arena_reset_to_random_kickoff(arena: &mut RsArena, seed: i64) {
    let seed_opt = if seed < 0 { None } else { Some(seed as u64) };
    arena.arena.reset_to_random_kickoff(seed_opt);
    // Reset all ball hit infos on arena reset
    for info in &mut arena.ball_hit_infos {
        *info = arena_wrapper::SynthBallHitInfo::default();
    }
}

fn rs_arena_tick_count(arena: &RsArena) -> u64 {
    arena.arena.tick_count()
}

fn rs_arena_game_mode(arena: &RsArena) -> u8 {
    game_mode_to_u8(arena.arena.game_mode())
}

fn rs_arena_get_car_state(arena: &RsArena, car_idx: u32) -> ffi::FfiCarState {
    // New API: get_car_state() returns &CarState — clone to produce owned value
    let state = arena.arena.get_car_state(car_idx as usize).clone();
    car_state_to_ffi(&state)
}

fn rs_arena_set_car_state(arena: &mut RsArena, car_idx: u32, state: &ffi::FfiCarState) {
    let rs_state = ffi_to_car_state(state);
    arena.arena.set_car_state(car_idx as usize, rs_state);
    // Reset ball hit info when state is set (matches C++ tickCountSinceUpdate behavior)
    if (car_idx as usize) < arena.ball_hit_infos.len() {
        arena.ball_hit_infos[car_idx as usize] = arena_wrapper::SynthBallHitInfo::default();
    }
}

fn rs_arena_set_car_controls(arena: &mut RsArena, car_idx: u32, controls: &ffi::FfiCarControls) {
    // New API: set_car_controls() takes owned CarControls — convert and pass by value
    let ctrl = ffi_to_controls(controls);
    arena.arena.set_car_controls(car_idx as usize, ctrl);
}

fn rs_arena_get_car_info(arena: &RsArena, car_idx: u32) -> ffi::FfiCarInfo {
    // New API: get_car_info() returns &CarInfo — copy fields out
    let info = arena.arena.get_car_info(car_idx as usize);
    ffi::FfiCarInfo {
        idx: info.idx as u32,
        team: team_to_u8(info.team),
    }
}

fn rs_arena_get_car_body_config(arena: &RsArena, car_idx: u32) -> ffi::FfiCarBodyConfig {
    // New API: get_car_info() returns &CarInfo — access config through it
    let info = arena.arena.get_car_info(car_idx as usize);
    car_body_config_to_ffi(&info.config)
}

fn rs_arena_get_ball_state(arena: &RsArena) -> ffi::FfiBallState {
    // New API: get_ball_state() returns &BallState — clone to produce owned value
    let state = arena.arena.get_ball_state().clone();
    ball_state_to_ffi(&state)
}

fn rs_arena_set_ball_state(arena: &mut RsArena, state: &ffi::FfiBallState) {
    let rs_state = ffi_to_ball_state(state);
    arena.arena.set_ball_state(rs_state);
}

fn rs_arena_get_boost_pad_state(arena: &RsArena, idx: u32) -> ffi::FfiBoostPadState {
    let state = arena.arena.get_boost_pad_state(idx as usize);
    ffi::FfiBoostPadState {
        cooldown: state.cooldown,
        is_active: state.is_active(),
    }
}

fn rs_arena_set_boost_pad_state(arena: &mut RsArena, idx: u32, state: &ffi::FfiBoostPadState) {
    let rs_state = rocketsim::BoostPadState {
        cooldown: state.cooldown,
    };
    arena.arena.set_boost_pad_state(idx as usize, rs_state);
}

fn rs_arena_get_boost_pad_config(arena: &RsArena, idx: u32) -> ffi::FfiBoostPadConfig {
    // New API: get_boost_pad_config() returns &BoostPadConfig — copy fields out
    let config = arena.arena.get_boost_pad_config(idx as usize);
    ffi::FfiBoostPadConfig {
        pos: vec3a_to_ffi(config.pos),
        is_big: config.is_big,
    }
}

fn rs_arena_get_mutator_config(arena: &RsArena) -> ffi::FfiMutatorConfig {
    // New API: mutator_config() returns &MutatorConfig — clone to produce owned value
    let config = arena.arena.mutator_config().clone();
    mutator_config_to_ffi(&config)
}

fn rs_arena_set_mutator_config(arena: &mut RsArena, config: &ffi::FfiMutatorConfig) {
    let new_config = ffi_to_mutator_config(config);
    arena.arena.set_mutator_config(new_config);
}

fn rs_arena_get_car_hit_ball_events(arena: &RsArena) -> Vec<ffi::FfiCarHitBallEvent> {
    arena.last_car_hit_ball_events.clone()
}

fn rs_arena_get_car_hit_car_events(arena: &RsArena) -> Vec<ffi::FfiCarHitCarEvent> {
    arena.last_car_hit_car_events.clone()
}

fn rs_arena_get_ball_hit_info(arena: &RsArena, car_idx: u32) -> ffi::FfiBallHitInfo {
    let idx = car_idx as usize;
    if idx < arena.ball_hit_infos.len() {
        let info = &arena.ball_hit_infos[idx];
        ffi::FfiBallHitInfo {
            is_valid: info.is_valid,
            relative_pos_on_ball: info.relative_pos_on_ball,
            ball_pos: info.ball_pos,
            extra_hit_vel: info.extra_hit_vel,
            tick_count_when_hit: info.tick_count_when_hit,
            tick_count_when_extra_impulse_applied: info.tick_count_when_extra_impulse_applied,
        }
    } else {
        ffi::FfiBallHitInfo::default()
    }
}

// ==================== Ball-only Arena ====================

fn rs_ball_arena_new(game_mode: u8, tick_rate: f32) -> Box<RsBallArena> {
    let gm = u8_to_game_mode(game_mode);
    Box::new(RsBallArena::new(gm, tick_rate))
}

fn rs_ball_arena_step(arena: &mut RsBallArena, ticks: u32) {
    arena.step(ticks);
}

fn rs_ball_arena_get_ball_state(arena: &RsBallArena) -> ffi::FfiBallState {
    // New API: get_ball_state() returns &BallState — clone
    let state = arena.arena.get_ball_state().clone();
    ball_state_to_ffi(&state)
}

fn rs_ball_arena_set_ball_state(arena: &mut RsBallArena, state: &ffi::FfiBallState) {
    let rs_state = ffi_to_ball_state(state);
    arena.arena.set_ball_state(rs_state);
}
