#![allow(dead_code)]

use std::ops::{Index, IndexMut};

use glam::{Mat3A, Vec3A};
use rocketsim::{CarControls, CarState, PhysState, consts::TICK_RATE};
#[repr(C)]
#[derive(Debug, Copy, Clone, PartialEq)]
pub struct VecRecord {
    pub x: f32,
    pub y: f32,
    pub z: f32,
}
impl VecRecord {
    pub fn new(x: f32, y: f32, z: f32) -> VecRecord {
        VecRecord { x, y, z }
    }
    pub fn length(&self) -> f32 {
        (self.x.powi(2) + self.y.powi(2) + self.z.powi(2)).sqrt()
    }
}
impl From<VecRecord> for Vec3A {
    fn from(val: VecRecord) -> Self {
        Vec3A::new(val.x, val.y, val.z)
    }
}
impl Index<usize> for VecRecord {
    type Output = f32;
    fn index(&self, index: usize) -> &f32 {
        match index {
            0 => &self.x,
            1 => &self.y,
            2 => &self.z,
            _ => unreachable!(),
        }
    }
}
impl IndexMut<usize> for VecRecord {
    fn index_mut(&mut self, index: usize) -> &mut f32 {
        match index {
            0 => &mut self.x,
            1 => &mut self.y,
            2 => &mut self.z,
            _ => unreachable!(),
        }
    }
}

#[repr(C)]
#[derive(Debug, Copy, Clone, PartialEq)]
pub struct Mat3Record {
    pub rows: [VecRecord; 3],
}
impl Mat3Record {
    pub fn column(&self, idx: usize) -> VecRecord {
        VecRecord::new(self.rows[0][idx], self.rows[1][idx], self.rows[2][idx])
    }
    pub fn forward(&self) -> VecRecord {
        self.column(0)
    }
    pub fn right(&self) -> VecRecord {
        self.column(1)
    }
    pub fn up(&self) -> VecRecord {
        self.column(2)
    }
}

#[repr(C)]
#[derive(Debug, Copy, Clone, PartialEq)]
pub struct WheelRecord {
    // ── v1 fields ──
    pub susp_length: f32,
    pub susp_rel_vel: f32,

    pub has_contact: bool,
    pub contact_normal: VecRecord,

    pub steer_amount: f32,
    pub engine_force: f32,
    pub brake: f32,

    pub lat_friction: f32,
    pub long_friction: f32,
    pub extra_pushback: f32,

    // ── v2 fields (zero in v1 recordings) ──
    pub spin_speed: f32,
    pub friction_curve_input: f32,
}

#[repr(C)]
#[derive(Debug, Copy, Clone, PartialEq)]
pub struct ControlsRecord {
    pub throttle: f32,
    pub steer: f32,
    pub pitch: f32,
    pub yaw: f32,
    pub roll: f32,
    pub jump: bool,
    pub boost: bool,
    pub handbrake: bool,
}
impl From<ControlsRecord> for CarControls {
    fn from(val: ControlsRecord) -> Self {
        CarControls {
            throttle: val.throttle,
            steer: val.steer,
            pitch: val.pitch,
            yaw: val.yaw,
            roll: val.roll,
            jump: val.jump,
            boost: val.boost,
            handbrake: val.handbrake,
        }
    }
}

#[repr(C)]
#[derive(Debug, Copy, Clone, PartialEq)]
pub struct PhysRecord {
    pub physics_frame: u32,

    pub pos: VecRecord,
    pub rot: Mat3Record,
    pub lin_vel: VecRecord,
    pub ang_vel: VecRecord,

    pub has_world_contact: bool,
    pub world_contact_point: VecRecord,
    pub world_contact_normal: VecRecord,
}
impl From<PhysRecord> for PhysState {
    fn from(phys_record: PhysRecord) -> Self {
        // Verify rotation matrix is sane.
        // If all rows are zero (padding record for a missing car), use identity.
        let rows_all_zero = phys_record.rot.rows[0].length() < 1e-6
            && phys_record.rot.rows[1].length() < 1e-6
            && phys_record.rot.rows[2].length() < 1e-6;

        if !rows_all_zero {
            for i in 0..3 {
                let c_len = phys_record.rot.rows[i].length();
                assert!((1.0 - c_len).abs() < 1e-6);

                // Dirs should be 90 degrees from other row dirs
                assert!(
                    Vec3A::dot(
                        phys_record.rot.rows[i].into(),
                        phys_record.rot.rows[(i + 1) % 3].into()
                    )
                    .abs()
                        < 1e-6
                );
            }
        }

        PhysState {
            pos: phys_record.pos.into(),
            vel: phys_record.lin_vel.into(),
            ang_vel: phys_record.ang_vel.into(),
            // The RLPR stores axis vectors in rows (row 0=forward, 1=right, 2=up).
            // Mat3A::from_cols expects column vectors, so pass rows directly —
            // column 0 becomes forward, column 1=right, column 2=up.
            // If all rows are zero (padding record), fall back to identity.
            rot_mat: if rows_all_zero {
                Mat3A::IDENTITY
            } else {
                Mat3A::from_cols(
                    phys_record.rot.rows[0].into(),
                    phys_record.rot.rows[1].into(),
                    phys_record.rot.rows[2].into(),
                )
            },
        }
    }
}

/// v3: the game's `OnHitBall` event for one tick (zero when the car did not
/// hit the ball this tick). `ball_vel_before`/`car_vel_before` are the
/// velocities at the hit event — i.e. BEFORE the impulse — so comparing them
/// against the tick-end velocity in [`PhysRecord`] isolates the impulse timing.
#[repr(C)]
#[derive(Debug, Copy, Clone, PartialEq)]
pub struct HitRecord {
    pub has_hit: bool,
    pub _pad: [u8; 3],
    pub ball_vel_before: VecRecord,
    pub car_vel_before: VecRecord,
    pub hit_normal: VecRecord,
    pub hit_location: VecRecord,
    pub rel_vel_mag: f32,
    pub closing_speed: f32,
}

impl Default for HitRecord {
    fn default() -> Self {
        Self {
            has_hit: false,
            _pad: [0; 3],
            ball_vel_before: VecRecord::new(0., 0., 0.),
            car_vel_before: VecRecord::new(0., 0., 0.),
            hit_normal: VecRecord::new(0., 0., 0.),
            hit_location: VecRecord::new(0., 0., 0.),
            rel_vel_mag: 0.,
            closing_speed: 0.,
        }
    }
}

#[repr(C)]
#[derive(Debug, Copy, Clone, PartialEq)]
pub struct CarRecord {
    // ── v1 fields ──
    pub phys: PhysRecord,

    pub is_on_ground: bool,
    pub is_jumping: bool,
    pub is_flipping: bool,
    pub jump_time: f32,
    pub flip_time: f32,
    pub has_jumped: bool,
    pub double_jumped_or_flipped: bool,
    pub has_flip: bool,
    pub flip_rel_torque: VecRecord,

    pub boost_amount: f32,

    pub is_touching_ball: bool,

    pub prev_controls: ControlsRecord,

    pub wheels: [WheelRecord; 4],

    // ── v2 fields (zero in v1 recordings) ──
    pub is_boosting: bool,
    pub is_supersonic: bool,
    pub is_demoed: bool,
    pub handbrake_val: f32,
    pub demo_respawn_timer: f32,
    pub air_time: f32,
    pub air_time_since_jump: f32,

    // ── v3 field (zero in v2 recordings) ──
    pub hit: HitRecord,
}

/// The v2 car-record layout (identical to [`CarRecord`] minus the trailing
/// [`HitRecord`]). Used to parse v2 recordings after the v3 format bump.
#[repr(C)]
#[derive(Debug, Copy, Clone, PartialEq)]
pub struct CarRecordV2 {
    pub phys: PhysRecord,

    pub is_on_ground: bool,
    pub is_jumping: bool,
    pub is_flipping: bool,
    pub jump_time: f32,
    pub flip_time: f32,
    pub has_jumped: bool,
    pub double_jumped_or_flipped: bool,
    pub has_flip: bool,
    pub flip_rel_torque: VecRecord,

    pub boost_amount: f32,

    pub is_touching_ball: bool,

    pub prev_controls: ControlsRecord,

    pub wheels: [WheelRecord; 4],

    pub is_boosting: bool,
    pub is_supersonic: bool,
    pub is_demoed: bool,
    pub handbrake_val: f32,
    pub demo_respawn_timer: f32,
    pub air_time: f32,
    pub air_time_since_jump: f32,
}

/// Early v2 recordings included per-body impulse diagnostics in `PhysRecord`
/// and predate the extra wheel and car-state fields in the final v2 layout.
#[repr(C)]
#[derive(Debug, Copy, Clone)]
pub struct LegacyImpulseRecordV2 {
    pub lin_impulse: VecRecord,
    pub ang_impulse: VecRecord,
    pub impulse_type: u8,
    pub is_accum: bool,
    pub _pad: [u8; 2],
}

#[repr(C)]
#[derive(Debug, Copy, Clone)]
pub struct LegacyPhysRecordV2 {
    pub physics_frame: u32,
    pub pos: VecRecord,
    pub rot: Mat3Record,
    pub lin_vel: VecRecord,
    pub ang_vel: VecRecord,
    pub has_world_contact: bool,
    pub world_contact_point: VecRecord,
    pub world_contact_normal: VecRecord,
    pub impulse_records_data: [LegacyImpulseRecordV2; 8],
    pub num_impulse_records: u32,
}

#[repr(C)]
#[derive(Debug, Copy, Clone)]
pub struct LegacyWheelRecordV2 {
    pub susp_length: f32,
    pub susp_rel_vel: f32,
    pub has_contact: bool,
    pub contact_normal: VecRecord,
    pub steer_amount: f32,
    pub engine_force: f32,
    pub brake: f32,
    pub lat_friction: f32,
    pub long_friction: f32,
    pub extra_pushback: f32,
}

#[repr(C)]
#[derive(Debug, Copy, Clone)]
pub struct LegacyCarRecordV2 {
    pub phys: LegacyPhysRecordV2,
    pub is_on_ground: bool,
    pub is_jumping: bool,
    pub is_flipping: bool,
    pub jump_time: f32,
    pub flip_time: f32,
    pub has_jumped: bool,
    pub double_jumped_or_flipped: bool,
    pub has_flip: bool,
    pub flip_rel_torque: VecRecord,
    pub boost_amount: f32,
    pub is_touching_ball: bool,
    pub prev_controls: ControlsRecord,
    pub wheels: [LegacyWheelRecordV2; 4],
}

impl From<LegacyPhysRecordV2> for PhysRecord {
    fn from(value: LegacyPhysRecordV2) -> Self {
        Self {
            physics_frame: value.physics_frame,
            pos: value.pos,
            rot: value.rot,
            lin_vel: value.lin_vel,
            ang_vel: value.ang_vel,
            has_world_contact: value.has_world_contact,
            world_contact_point: value.world_contact_point,
            world_contact_normal: value.world_contact_normal,
        }
    }
}

impl From<LegacyWheelRecordV2> for WheelRecord {
    fn from(value: LegacyWheelRecordV2) -> Self {
        Self {
            susp_length: value.susp_length,
            susp_rel_vel: value.susp_rel_vel,
            has_contact: value.has_contact,
            contact_normal: value.contact_normal,
            steer_amount: value.steer_amount,
            engine_force: value.engine_force,
            brake: value.brake,
            lat_friction: value.lat_friction,
            long_friction: value.long_friction,
            extra_pushback: value.extra_pushback,
            spin_speed: 0.0,
            friction_curve_input: 0.0,
        }
    }
}

impl From<LegacyCarRecordV2> for CarRecord {
    fn from(value: LegacyCarRecordV2) -> Self {
        Self {
            phys: value.phys.into(),
            is_on_ground: value.is_on_ground,
            is_jumping: value.is_jumping,
            is_flipping: value.is_flipping,
            jump_time: value.jump_time,
            flip_time: value.flip_time,
            has_jumped: value.has_jumped,
            double_jumped_or_flipped: value.double_jumped_or_flipped,
            has_flip: value.has_flip,
            flip_rel_torque: value.flip_rel_torque,
            boost_amount: value.boost_amount,
            is_touching_ball: value.is_touching_ball,
            prev_controls: value.prev_controls,
            wheels: value.wheels.map(Into::into),
            is_boosting: false,
            is_supersonic: false,
            is_demoed: false,
            handbrake_val: 0.0,
            demo_respawn_timer: 0.0,
            air_time: 0.0,
            air_time_since_jump: 0.0,
            hit: HitRecord::default(),
        }
    }
}

impl From<CarRecordV2> for CarRecord {
    fn from(v2: CarRecordV2) -> Self {
        let CarRecordV2 {
            phys,
            is_on_ground,
            is_jumping,
            is_flipping,
            jump_time,
            flip_time,
            has_jumped,
            double_jumped_or_flipped,
            has_flip,
            flip_rel_torque,
            boost_amount,
            is_touching_ball,
            prev_controls,
            wheels,
            is_boosting,
            is_supersonic,
            is_demoed,
            handbrake_val,
            demo_respawn_timer,
            air_time,
            air_time_since_jump,
        } = v2;
        Self {
            phys,
            is_on_ground,
            is_jumping,
            is_flipping,
            jump_time,
            flip_time,
            has_jumped,
            double_jumped_or_flipped,
            has_flip,
            flip_rel_torque,
            boost_amount,
            is_touching_ball,
            prev_controls,
            wheels,
            is_boosting,
            is_supersonic,
            is_demoed,
            handbrake_val,
            demo_respawn_timer,
            air_time,
            air_time_since_jump,
            hit: HitRecord::default(),
        }
    }
}
impl From<CarRecord> for CarState {
    fn from(phys_record: CarRecord) -> Self {
        // Map double_jumped_or_flipped:
        // - If currently flipping (is_flipping=true), the second action was a flip
        // - Otherwise it was a double jump
        let double_jumped = phys_record.double_jumped_or_flipped && !phys_record.is_flipping;
        let flipped = phys_record.double_jumped_or_flipped && phys_record.is_flipping;

        Self {
            phys: phys_record.phys.into(),
            boost: phys_record.boost_amount,
            controls: phys_record.prev_controls.into(),
            prev_controls: phys_record.prev_controls.into(),
            is_on_ground: phys_record.is_on_ground,
            is_jumping: phys_record.is_jumping,
            is_flipping: phys_record.is_flipping,
            flip_rel_torque: phys_record.flip_rel_torque.into(),
            // NOTE: must round, NOT truncate — RL's accumulated f32 lands
            // below the integer multiple on ~54% of recorded ticks
            // (e.g. 41.99999928), and truncating would drop a whole tick.
            jump_ticks: (phys_record.jump_time * TICK_RATE).round() as u32,
            flip_time: phys_record.flip_time,
            has_jumped: phys_record.has_jumped,
            has_double_jumped: double_jumped,
            has_flipped: flipped,
            // ── v2 fields (zero in v1 recordings → fall back to defaults) ──
            is_boosting: phys_record.is_boosting,
            is_supersonic: phys_record.is_supersonic,
            is_demoed: phys_record.is_demoed,
            handbrake_val: phys_record.handbrake_val,
            demo_respawn_timer: phys_record.demo_respawn_timer,
            air_time: phys_record.air_time,
            air_time_since_jump: phys_record.air_time_since_jump,
            ..Default::default()
        }
    }
}

#[repr(C)]
#[derive(Debug, Copy, Clone, PartialEq)]
pub struct RecordingInfo {
    pub num_cars: u32,
    pub hitbox_rel_min_bt: VecRecord,
    pub hitbox_rel_max_bt: VecRecord,
}
