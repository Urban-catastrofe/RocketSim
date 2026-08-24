use glam::Vec3A;

use crate::{
    bullet::dynamics::rigid_body::RigidBody,
    sim::consts::{BT_TO_UU, UU_TO_BT, quantize},
};

/// An extension of `Vec3A::signum` that keeps zero-components as zero (regardless of sign bit).
#[must_use]
fn vec3a_sign_3(v: Vec3A) -> Vec3A {
    let signum = v.signum();
    Vec3A::select(v.cmpeq(Vec3A::ZERO), Vec3A::ZERO, signum)
}

enum VecQuantizeMode {
    Position,
    Velocity,
}

/// UE3-networking-style quantization of vectors.
#[must_use]
fn quantize_vec_ue3(vec: Vec3A, scale: f32, quantize_mode: VecQuantizeMode) -> Vec3A {
    match quantize_mode {
        VecQuantizeMode::Position => (vec * scale).round() / scale,
        VecQuantizeMode::Velocity => {
            let inv_scale = 1.0 / scale;
            let mut rounded = Vec3A::ZERO;
            for i in 0..3 {
                let i_val = (vec[i] * scale) as i32;
                rounded[i] = (i_val as f32) * inv_scale;
            }

            const OFFSET_CORRECT_FRAC: f32 = 0.1;
            let offset_mag = OFFSET_CORRECT_FRAC * inv_scale;

            rounded + (vec3a_sign_3(rounded) * offset_mag)
        }
    }
}

/// Quantizes the position, linear velocity, and angular velocity of a rigid body.
pub fn quantize(body: &mut RigidBody) {
    let lin_before = body.lin_vel;
    let ang_before = body.ang_vel;

    let new_pos = quantize_vec_ue3(
        body.get_world_pos() * BT_TO_UU,
        quantize::POS_SCALE,
        VecQuantizeMode::Position,
    ) * UU_TO_BT;
    let new_vel = quantize_vec_ue3(
        body.lin_vel * BT_TO_UU,
        quantize::VEL_SCALE,
        VecQuantizeMode::Velocity,
    ) * UU_TO_BT;
    let new_ang_vel = quantize_vec_ue3(
        body.ang_vel * BT_TO_UU,
        quantize::ANG_VEL_SCALE,
        VecQuantizeMode::Velocity,
    ) * UU_TO_BT;

    body.set_world_pos(new_pos);
    body.set_lin_vel(new_vel);
    body.set_ang_vel(new_ang_vel);

    // Quantization is not a force, but it does change velocity, so the impulse
    // ledger has to carry it or the accounting identity blames the constraint
    // solver for the rounding. Unlike the clamps this fires on nearly every
    // tick, so it is recorded unconditionally.
    let lin_delta = body.lin_vel - lin_before;
    let ang_delta = body.ang_vel - ang_before;
    body.record_impulse("Quantize", lin_delta, ang_delta, false);
}
