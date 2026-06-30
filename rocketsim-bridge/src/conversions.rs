use glam::{Mat3A, Vec3A};

use crate::ffi;

// Vec3A <-> FfiVec
pub fn vec3a_to_ffi(v: Vec3A) -> ffi::FfiVec {
    ffi::FfiVec {
        x: v.x,
        y: v.y,
        z: v.z,
    }
}

pub fn ffi_to_vec3a(v: &ffi::FfiVec) -> Vec3A {
    Vec3A::new(v.x, v.y, v.z)
}

// Mat3A <-> FfiRotMat
// glam Mat3A: x_axis=forward, y_axis=right, z_axis=up (column-major)
// C++ RotMat: forward, right, up (each a Vec)
pub fn mat3a_to_ffi(m: Mat3A) -> ffi::FfiRotMat {
    ffi::FfiRotMat {
        forward_x: m.x_axis.x,
        forward_y: m.x_axis.y,
        forward_z: m.x_axis.z,
        right_x: m.y_axis.x,
        right_y: m.y_axis.y,
        right_z: m.y_axis.z,
        up_x: m.z_axis.x,
        up_y: m.z_axis.y,
        up_z: m.z_axis.z,
    }
}

pub fn ffi_to_mat3a(m: &ffi::FfiRotMat) -> Mat3A {
    Mat3A::from_cols(
        Vec3A::new(m.forward_x, m.forward_y, m.forward_z),
        Vec3A::new(m.right_x, m.right_y, m.right_z),
        Vec3A::new(m.up_x, m.up_y, m.up_z),
    )
}

// PhysState conversions
pub fn phys_to_ffi(p: &rocketsim::PhysState) -> ffi::FfiPhysState {
    ffi::FfiPhysState {
        pos: vec3a_to_ffi(p.pos),
        rot_mat: mat3a_to_ffi(p.rot_mat),
        vel: vec3a_to_ffi(p.vel),
        ang_vel: vec3a_to_ffi(p.ang_vel),
    }
}

pub fn ffi_to_phys(p: &ffi::FfiPhysState) -> rocketsim::PhysState {
    rocketsim::PhysState {
        pos: ffi_to_vec3a(&p.pos),
        rot_mat: ffi_to_mat3a(&p.rot_mat),
        vel: ffi_to_vec3a(&p.vel),
        ang_vel: ffi_to_vec3a(&p.ang_vel),
    }
}

// CarControls conversions
pub fn controls_to_ffi(c: &rocketsim::CarControls) -> ffi::FfiCarControls {
    ffi::FfiCarControls {
        throttle: c.throttle,
        steer: c.steer,
        pitch: c.pitch,
        yaw: c.yaw,
        roll: c.roll,
        jump: c.jump,
        boost: c.boost,
        handbrake: c.handbrake,
    }
}

pub fn ffi_to_controls(c: &ffi::FfiCarControls) -> rocketsim::CarControls {
    rocketsim::CarControls {
        throttle: c.throttle,
        steer: c.steer,
        pitch: c.pitch,
        yaw: c.yaw,
        roll: c.roll,
        jump: c.jump,
        boost: c.boost,
        handbrake: c.handbrake,
    }
}

// CarState conversions
// NOTE: New rocketsim API returns &CarState, so the bridge clones internally.
// Two fields were removed from CarState in rocketsim v0.2.0:
//   - bump_cooldown_other_car_idx (Option<usize>)
//   - last_extra_hit_tick (Option<u64>)
// The FFI struct still carries them for C++ ABI compatibility; we always
// send default values (false/0) since the Rust side no longer tracks them.
pub fn car_state_to_ffi(s: &rocketsim::CarState) -> ffi::FfiCarState {
    let (has_world_contact, world_contact_normal) = match s.world_contact_normal {
        Some(n) => (true, vec3a_to_ffi(n)),
        None => (false, ffi::FfiVec { x: 0.0, y: 0.0, z: 0.0 }),
    };

    ffi::FfiCarState {
        phys: phys_to_ffi(&s.phys),
        controls: controls_to_ffi(&s.controls),
        prev_controls: controls_to_ffi(&s.prev_controls),
        is_on_ground: s.is_on_ground,
        wheels_with_contact_0: s.wheels_with_contact[0],
        wheels_with_contact_1: s.wheels_with_contact[1],
        wheels_with_contact_2: s.wheels_with_contact[2],
        wheels_with_contact_3: s.wheels_with_contact[3],
        has_jumped: s.has_jumped,
        has_double_jumped: s.has_double_jumped,
        has_flipped: s.has_flipped,
        flip_rel_torque: vec3a_to_ffi(s.flip_rel_torque),
        jump_time: s.jump_time,
        flip_time: s.flip_time,
        is_flipping: s.is_flipping,
        is_jumping: s.is_jumping,
        air_time: s.air_time,
        air_time_since_jump: s.air_time_since_jump,
        boost_amount: s.boost,
        time_since_boosted: s.time_since_boosted,
        is_boosting: s.is_boosting,
        boosting_time: s.boosting_time,
        is_supersonic: s.is_supersonic,
        supersonic_time: s.supersonic_time,
        handbrake_val: s.handbrake_val,
        is_auto_flipping: s.is_auto_flipping,
        auto_flip_timer: s.auto_flip_timer,
        auto_flip_torque_scale: s.auto_flip_torque_scale,
        bump_cooldown_timer: s.bump_cooldown_timer,
        has_world_contact,
        world_contact_normal,
        is_demoed: s.is_demoed,
        demo_respawn_timer: s.demo_respawn_timer,
        // Removed fields — always send defaults for C++ ABI compat
        bump_cooldown_other_car_idx_valid: false,
        bump_cooldown_other_car_idx: 0,
        last_extra_hit_tick_valid: false,
        last_extra_hit_tick: 0,
    }
}

pub fn ffi_to_car_state(s: &ffi::FfiCarState) -> rocketsim::CarState {
    let world_contact_normal = if s.has_world_contact {
        Some(ffi_to_vec3a(&s.world_contact_normal))
    } else {
        None
    };

    rocketsim::CarState {
        phys: ffi_to_phys(&s.phys),
        controls: ffi_to_controls(&s.controls),
        prev_controls: ffi_to_controls(&s.prev_controls),
        is_on_ground: s.is_on_ground,
        wheels_with_contact: [
            s.wheels_with_contact_0,
            s.wheels_with_contact_1,
            s.wheels_with_contact_2,
            s.wheels_with_contact_3,
        ],
        has_jumped: s.has_jumped,
        has_double_jumped: s.has_double_jumped,
        has_flipped: s.has_flipped,
        flip_rel_torque: ffi_to_vec3a(&s.flip_rel_torque),
        jump_time: s.jump_time,
        flip_time: s.flip_time,
        is_flipping: s.is_flipping,
        is_jumping: s.is_jumping,
        air_time: s.air_time,
        air_time_since_jump: s.air_time_since_jump,
        boost: s.boost_amount,
        time_since_boosted: s.time_since_boosted,
        is_boosting: s.is_boosting,
        boosting_time: s.boosting_time,
        is_supersonic: s.is_supersonic,
        supersonic_time: s.supersonic_time,
        handbrake_val: s.handbrake_val,
        is_auto_flipping: s.is_auto_flipping,
        auto_flip_timer: s.auto_flip_timer,
        auto_flip_torque_scale: s.auto_flip_torque_scale,
        bump_cooldown_timer: s.bump_cooldown_timer,
        world_contact_normal,
        is_demoed: s.is_demoed,
        demo_respawn_timer: s.demo_respawn_timer,
        // bump_cooldown_other_car_idx and last_extra_hit_tick removed in v0.2.0
    }
}

// BallState conversions
pub fn ball_state_to_ffi(s: &rocketsim::BallState) -> ffi::FfiBallState {
    let (last_hit_valid, last_hit_val) = match s.last_extra_hit_tick {
        Some(t) => (true, t),
        None => (false, 0u64),
    };
    ffi::FfiBallState {
        phys: phys_to_ffi(&s.phys),
        hs_y_target_dir: s.hs_info.y_target_dir,
        hs_cur_target_speed: s.hs_info.cur_target_speed,
        hs_time_since_hit: s.hs_info.time_since_hit,
        ds_charge_level: s.ds_info.charge_level as i32,
        ds_accumulated_hit_force: s.ds_info.accumulated_hit_force,
        ds_y_target_dir: s.ds_info.y_target_dir,
        ds_has_damaged: s.ds_info.last_damage_tick.is_some(),
        ds_last_damage_tick: s.ds_info.last_damage_tick.unwrap_or(0),
        tick_count_since_kickoff: s.tick_count_since_kickoff,
        last_extra_hit_tick_valid: last_hit_valid,
        last_extra_hit_tick: last_hit_val,
    }
}

pub fn ffi_to_ball_state(s: &ffi::FfiBallState) -> rocketsim::BallState {
    use rocketsim::{DropshotInfo, HeatseekerInfo};

    rocketsim::BallState {
        phys: ffi_to_phys(&s.phys),
        hs_info: HeatseekerInfo {
            y_target_dir: s.hs_y_target_dir,
            cur_target_speed: s.hs_cur_target_speed,
            time_since_hit: s.hs_time_since_hit,
        },
        ds_info: DropshotInfo {
            charge_level: s.ds_charge_level.clamp(0, u8::MAX as i32) as u8,
            accumulated_hit_force: s.ds_accumulated_hit_force,
            y_target_dir: s.ds_y_target_dir,
            last_damage_tick: if s.ds_has_damaged {
                Some(s.ds_last_damage_tick)
            } else {
                None
            },
        },
        last_extra_hit_tick: if s.last_extra_hit_tick_valid {
            Some(s.last_extra_hit_tick)
        } else {
            None
        },
        tick_count_since_kickoff: s.tick_count_since_kickoff,
    }
}

// GameMode conversions
pub fn u8_to_game_mode(v: u8) -> rocketsim::GameMode {
    match v {
        0 => rocketsim::GameMode::Soccar,
        1 => rocketsim::GameMode::Hoops,
        2 => rocketsim::GameMode::Heatseeker,
        3 => rocketsim::GameMode::Snowday,
        4 => rocketsim::GameMode::Dropshot,
        5 => rocketsim::GameMode::TheVoid,
        _ => rocketsim::GameMode::Soccar,
    }
}

pub fn game_mode_to_u8(gm: rocketsim::GameMode) -> u8 {
    match gm {
        rocketsim::GameMode::Soccar => 0,
        rocketsim::GameMode::Hoops => 1,
        rocketsim::GameMode::Heatseeker => 2,
        rocketsim::GameMode::Snowday => 3,
        rocketsim::GameMode::Dropshot => 4,
        rocketsim::GameMode::TheVoid => 5,
    }
}

pub fn u8_to_team(v: u8) -> rocketsim::Team {
    match v {
        0 => rocketsim::Team::Blue,
        1 => rocketsim::Team::Orange,
        _ => rocketsim::Team::Blue,
    }
}

pub fn team_to_u8(t: rocketsim::Team) -> u8 {
    match t {
        rocketsim::Team::Blue => 0,
        rocketsim::Team::Orange => 1,
    }
}

// CarBodyConfig conversions
pub fn ffi_to_car_body_config(c: &ffi::FfiCarBodyConfig) -> rocketsim::CarBodyConfig {
    rocketsim::CarBodyConfig {
        hitbox_size: ffi_to_vec3a(&c.hitbox_size),
        hitbox_pos_offset: ffi_to_vec3a(&c.hitbox_pos_offset),
        front_wheels: rocketsim::WheelPairConfig {
            wheel_radius: c.front_wheel_radius,
            suspension_rest_length: c.front_suspension_rest,
            connection_point_offset: ffi_to_vec3a(&c.front_connection_offset),
        },
        back_wheels: rocketsim::WheelPairConfig {
            wheel_radius: c.back_wheel_radius,
            suspension_rest_length: c.back_suspension_rest,
            connection_point_offset: ffi_to_vec3a(&c.back_connection_offset),
        },
        three_wheels: c.three_wheels,
        dodge_deadzone: c.dodge_deadzone,
    }
}

pub fn car_body_config_to_ffi(c: &rocketsim::CarBodyConfig) -> ffi::FfiCarBodyConfig {
    ffi::FfiCarBodyConfig {
        hitbox_size: vec3a_to_ffi(c.hitbox_size),
        hitbox_pos_offset: vec3a_to_ffi(c.hitbox_pos_offset),
        front_wheel_radius: c.front_wheels.wheel_radius,
        front_suspension_rest: c.front_wheels.suspension_rest_length,
        front_connection_offset: vec3a_to_ffi(c.front_wheels.connection_point_offset),
        back_wheel_radius: c.back_wheels.wheel_radius,
        back_suspension_rest: c.back_wheels.suspension_rest_length,
        back_connection_offset: vec3a_to_ffi(c.back_wheels.connection_point_offset),
        three_wheels: c.three_wheels,
        dodge_deadzone: c.dodge_deadzone,
    }
}

// ArenaConfig conversions
// `FfiArenaConfig` does not carry game_mode or mutators — those come from the
// caller. Mutators default to `MutatorConfig::new(game_mode)`; callers that
// want non-default mutators should call `set_mutator_config` after construction.
pub fn ffi_to_arena_config(
    game_mode: rocketsim::GameMode,
    c: &ffi::FfiArenaConfig,
) -> rocketsim::ArenaConfig {
    rocketsim::ArenaConfig {
        game_mode,
        mutators: rocketsim::MutatorConfig::new(game_mode),
        mem_weight_mode: match c.mem_weight_mode {
            1 => rocketsim::ArenaMemWeightMode::Light,
            _ => rocketsim::ArenaMemWeightMode::Heavy,
        },
        min_pos: ffi_to_vec3a(&c.min_pos),
        max_pos: ffi_to_vec3a(&c.max_pos),
        max_aabb_len: c.max_aabb_len,
        no_ball_rot: c.no_ball_rot,
        custom_boost_pads: None,
        rng_seed: None,
    }
}

// MutatorConfig conversions
pub fn ffi_to_mutator_config(m: &ffi::FfiMutatorConfig) -> rocketsim::MutatorConfig {
    rocketsim::MutatorConfig {
        gravity: ffi_to_vec3a(&m.gravity),
        car_mass: m.car_mass,
        ball_mass: m.ball_mass,
        ball_max_speed: m.ball_max_speed,
        ball_drag: m.ball_drag,
        jump_accel: m.jump_accel,
        jump_immediate_force: m.jump_immediate_force,
        boost_accel_ground: m.boost_accel_ground,
        boost_accel_air: m.boost_accel_air,
        boost_used_per_second: m.boost_used_per_second,
        respawn_delay: m.respawn_delay,
        bump_cooldown_time: m.bump_cooldown_time,
        car_max_boost_amount: m.car_max_boost_amount,
        car_spawn_boost_amount: m.car_spawn_boost_amount,
        boost_pad_amount_small: m.boost_pad_amount_small,
        boost_pad_amount_big: m.boost_pad_amount_big,
        boost_pad_cooldown_big: m.boost_pad_cooldown_big,
        boost_pad_cooldown_small: m.boost_pad_cooldown_small,
        ball_hit_extra_force_scale: m.ball_hit_extra_force_scale,
        bump_force_scale: m.bump_force_scale,
        ball_radius: m.ball_radius,
        unlimited_flips: m.unlimited_flips,
        unlimited_double_jumps: m.unlimited_double_jumps,
        recharge_boost_enabled: m.recharge_boost_enabled,
        recharge_boost_per_second: m.recharge_boost_per_second,
        recharge_boost_delay: m.recharge_boost_delay,
        demo_mode: match m.demo_mode {
            1 => rocketsim::DemoMode::OnContact,
            2 => rocketsim::DemoMode::Disabled,
            _ => rocketsim::DemoMode::Normal,
        },
        enable_team_demos: m.enable_team_demos,
        goal_base_threshold_y: m.goal_base_threshold_y,
    }
}

pub fn mutator_config_to_ffi(c: &rocketsim::MutatorConfig) -> ffi::FfiMutatorConfig {
    ffi::FfiMutatorConfig {
        gravity: vec3a_to_ffi(c.gravity),
        car_mass: c.car_mass,
        ball_mass: c.ball_mass,
        ball_max_speed: c.ball_max_speed,
        ball_drag: c.ball_drag,
        jump_accel: c.jump_accel,
        jump_immediate_force: c.jump_immediate_force,
        boost_accel_ground: c.boost_accel_ground,
        boost_accel_air: c.boost_accel_air,
        boost_used_per_second: c.boost_used_per_second,
        respawn_delay: c.respawn_delay,
        bump_cooldown_time: c.bump_cooldown_time,
        car_max_boost_amount: c.car_max_boost_amount,
        car_spawn_boost_amount: c.car_spawn_boost_amount,
        boost_pad_amount_small: c.boost_pad_amount_small,
        boost_pad_amount_big: c.boost_pad_amount_big,
        boost_pad_cooldown_big: c.boost_pad_cooldown_big,
        boost_pad_cooldown_small: c.boost_pad_cooldown_small,
        ball_hit_extra_force_scale: c.ball_hit_extra_force_scale,
        bump_force_scale: c.bump_force_scale,
        bump_requires_front_hit: false, // removed from rocketsim v0.2.0, always false
        ball_radius: c.ball_radius,
        unlimited_flips: c.unlimited_flips,
        unlimited_double_jumps: c.unlimited_double_jumps,
        recharge_boost_enabled: c.recharge_boost_enabled,
        recharge_boost_per_second: c.recharge_boost_per_second,
        recharge_boost_delay: c.recharge_boost_delay,
        demo_mode: match c.demo_mode {
            rocketsim::DemoMode::Normal => 0,
            rocketsim::DemoMode::OnContact => 1,
            rocketsim::DemoMode::Disabled => 2,
        },
        enable_team_demos: c.enable_team_demos,
        goal_base_threshold_y: c.goal_base_threshold_y,
    }
}
