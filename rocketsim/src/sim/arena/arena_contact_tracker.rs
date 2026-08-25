use std::mem;

use crate::{
    bullet::{
        collision::{
            dispatch::internal_edge_utility::adjust_internal_edge_contacts,
            narrowphase::{
                manifold_point::ManifoldPoint,
                persistent_manifold::{
                    ContactAddedCallback, ContactSolveInfo, SpecialContactSolveInfo,
                },
            },
        },
        dynamics::rigid_body::RigidBody,
    },
    consts::{self, BT_TO_UU},
    sim::UserInfoTypes,
};

// An instance of a contact event
#[derive(Debug, Copy, Clone)]
pub(crate) struct ContactRecord {
    pub is_swap: bool,
    pub rb_idx_a: usize,
    pub rb_idx_b: usize,
    pub manifold_point: ManifoldPoint,
}

/// Solver output for one car-ball manifold point from the last simulated tick.
#[derive(Debug, Copy, Clone)]
pub struct CarBallContactInfo {
    pub car_idx: usize,
    /// Point on the ball, in Unreal units.
    pub contact_point: glam::Vec3A,
    /// Contact normal pointing from the car toward the ball.
    pub contact_normal: glam::Vec3A,
    /// Signed manifold distance in Unreal units; negative values penetrate.
    pub distance: f32,
    /// Raw Bullet normal and split-penetration impulses.
    pub normal_impulse: f32,
    pub push_impulse: f32,
    /// Ball contact-point velocity relative to the car, in Unreal units/s.
    pub relative_velocity_before: glam::Vec3A,
    pub relative_velocity_after: glam::Vec3A,
}

#[derive(Debug, Clone)]
pub struct BallWorldContactPointInfo {
    pub contact_point: glam::Vec3A,
    pub contact_normal: glam::Vec3A,
    pub distance: f32,
}

#[derive(Debug, Clone)]
pub struct BallWorldConstraintInfo {
    pub points: Vec<BallWorldContactPointInfo>,
    pub effective_normal: glam::Vec3A,
    pub mean_penetration: f32,
    pub restitution_velocity: f32,
    pub normal_impulse: f32,
    pub push_impulse: f32,
    pub relative_velocity_before: glam::Vec3A,
    pub relative_velocity_after: glam::Vec3A,
}

// A struct to be accessed through the bullet contact callbacks
pub(crate) struct ArenaContactTracker {
    collision_records: Vec<ContactRecord>,
    solved_car_ball_contacts: Vec<CarBallContactInfo>,
    solved_ball_world_contacts: Vec<BallWorldConstraintInfo>,
    track_ball_world_contacts: bool,
}

impl ArenaContactTracker {
    pub fn new() -> Self {
        Self {
            collision_records: Vec::with_capacity(4), // Rarely exceeded
            solved_car_ball_contacts: Vec::with_capacity(4),
            solved_ball_world_contacts: Vec::with_capacity(1),
            track_ball_world_contacts: false,
        }
    }

    pub const fn num_records(&self) -> usize {
        self.collision_records.len()
    }

    pub fn get_record(&self, idx: usize) -> &ContactRecord {
        &self.collision_records[idx]
    }

    pub fn clear_records(&mut self) {
        self.collision_records.clear();
    }

    pub fn solved_car_ball_contacts(&self) -> &[CarBallContactInfo] {
        &self.solved_car_ball_contacts
    }

    pub fn clear_solved_contacts(&mut self) {
        self.solved_car_ball_contacts.clear();
        self.solved_ball_world_contacts.clear();
    }

    pub fn set_track_ball_world_contacts(&mut self, enabled: bool) {
        self.track_ball_world_contacts = enabled;
    }

    pub fn solved_ball_world_contacts(&self) -> &[BallWorldConstraintInfo] {
        &self.solved_ball_world_contacts
    }
}

impl ContactAddedCallback for ArenaContactTracker {
    fn callback<'a>(
        &mut self,
        manifold_point: &mut ManifoldPoint,
        mut body_a: &'a RigidBody,
        mut body_b: &'a RigidBody,
        idx: Option<usize>,
    ) {
        debug_assert!(body_a.has_contact_response() || body_b.has_contact_response());

        let should_swap =
            if body_a.user_idx != UserInfoTypes::None && body_b.user_idx != UserInfoTypes::None {
                body_a.user_idx > body_b.user_idx
            } else {
                body_b.user_idx != UserInfoTypes::None
            };

        if should_swap {
            mem::swap(&mut body_a, &mut body_b);
        }

        let user_idx_a = body_a.user_idx;
        let user_idx_b = body_b.user_idx;

        if user_idx_a == UserInfoTypes::Car {
            let hit_coefs = match user_idx_b {
                UserInfoTypes::Ball => consts::car::HIT_BALL_COEFS,
                UserInfoTypes::Car => consts::car::HIT_CAR_COEFS,
                _ => consts::car::HIT_WORLD_COEFS,
            };
            manifold_point.combined_friction = hit_coefs.friction;
            manifold_point.combined_restitution = hit_coefs.restitution;
        } else if user_idx_a == UserInfoTypes::Ball
            && user_idx_b == UserInfoTypes::None
            && body_b.is_static_obj()
        {
            manifold_point.is_special = true;
        }

        // NOTE: Push *before* the manifold is mutated by adjust_internal_edge_contacts()
        self.collision_records.push(ContactRecord {
            is_swap: should_swap,
            rb_idx_a: body_a.world_array_idx,
            rb_idx_b: body_b.world_array_idx,
            manifold_point: *manifold_point,
        });

        if let Some(idx) = idx {
            adjust_internal_edge_contacts(manifold_point, body_b, idx);
        }
    }

    fn contact_solved(
        &mut self,
        contact: ContactSolveInfo,
        body_a: &RigidBody,
        body_b: &RigidBody,
    ) {
        let (car, ball_is_body_a) = match (body_a.user_idx, body_b.user_idx) {
            (UserInfoTypes::Ball, UserInfoTypes::Car) => (body_b, true),
            (UserInfoTypes::Car, UserInfoTypes::Ball) => (body_a, false),
            _ => return,
        };
        let direction = if ball_is_body_a { 1.0 } else { -1.0 };
        let point = if ball_is_body_a {
            contact.manifold_point.pos_world_on_a
        } else {
            contact.manifold_point.pos_world_on_b
        };
        self.solved_car_ball_contacts.push(CarBallContactInfo {
            car_idx: car.user_pointer,
            contact_point: point * BT_TO_UU,
            contact_normal: contact.manifold_point.normal_world_on_b * direction,
            distance: contact.manifold_point.distance_1 * BT_TO_UU,
            normal_impulse: contact.normal_impulse,
            push_impulse: contact.push_impulse,
            relative_velocity_before: contact.relative_velocity_before * direction * BT_TO_UU,
            relative_velocity_after: contact.relative_velocity_after * direction * BT_TO_UU,
        });
    }

    fn tracks_special_contacts(&self) -> bool {
        self.track_ball_world_contacts
    }

    fn special_contact_solved(&mut self, contact: SpecialContactSolveInfo, body: &RigidBody) {
        if body.user_idx != UserInfoTypes::Ball {
            return;
        }
        self.solved_ball_world_contacts
            .push(BallWorldConstraintInfo {
                points: contact
                    .points
                    .into_iter()
                    .map(|point| BallWorldContactPointInfo {
                        contact_point: point.pos_world_on_b * BT_TO_UU,
                        contact_normal: point.normal_world_on_b,
                        distance: point.distance * BT_TO_UU,
                    })
                    .collect(),
                effective_normal: contact.effective_normal,
                mean_penetration: contact.mean_penetration * BT_TO_UU,
                restitution_velocity: contact.restitution_velocity * BT_TO_UU,
                normal_impulse: contact.normal_impulse,
                push_impulse: contact.push_impulse,
                relative_velocity_before: contact.relative_velocity_before * BT_TO_UU,
                relative_velocity_after: contact.relative_velocity_after * BT_TO_UU,
            });
    }
}
