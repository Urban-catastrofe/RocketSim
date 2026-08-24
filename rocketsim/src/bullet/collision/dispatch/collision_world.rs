use glam::Vec3A;

use super::{
    collision_dispatcher::CollisionDispatcher,
    quad_ray_callbacks::{BridgeTriQuadRayCallback, QuadRayResultCallback},
    sphere_obb_collision_alg,
};

const ENABLE_CAR_BALL_TOI: bool = false;
use crate::{
    bullet::{
        collision::{
            broadphase::GridBroadphase,
            narrowphase::persistent_manifold::{
                CONTACT_BREAKING_THRESHOLD, ContactAddedCallback, PersistentManifold,
            },
            shapes::{
                collision_shape::CollisionShapes, compound_shape::CompoundShape,
                sphere_shape::SphereShape,
            },
        },
        dynamics::rigid_body::RigidBody,
        linear_math::AffineExt,
    },
    shared::QuadRayInfo,
    sim::UserInfoTypes,
};

pub struct CollisionWorld {
    pub collision_objs: Vec<RigidBody>,
    pub dispatcher1: CollisionDispatcher,
    pub broadphase_pair_cache: GridBroadphase,
    num_skippable_statics: usize,
}

impl CollisionWorld {
    pub fn new(pair_cache: GridBroadphase) -> Self {
        Self {
            collision_objs: Vec::new(),
            dispatcher1: CollisionDispatcher::default(),
            broadphase_pair_cache: pair_cache,
            num_skippable_statics: 0,
        }
    }

    pub fn add_collision_obj(
        &mut self,
        mut obj: RigidBody,
        filter_group: u8,
        filter_mask: u8,
    ) -> usize {
        let idx = self.collision_objs.len();
        obj.world_array_idx = idx;

        let trans = obj.get_world_trans();
        let aabb = obj.get_collision_shape().get_aabb(trans);
        let proxy = self
            .broadphase_pair_cache
            .create_proxy(aabb, &obj, filter_group, filter_mask);

        obj.set_broadphase_handle(proxy);
        self.collision_objs.push(obj);

        idx
    }

    fn update_aabbs(&mut self) {
        const CBT: Vec3A = Vec3A::splat(CONTACT_BREAKING_THRESHOLD);

        let mut prev_is_static = true;
        for (i, col_obj) in self
            .collision_objs
            .iter()
            .enumerate()
            .skip(self.num_skippable_statics)
        {
            debug_assert_eq!(col_obj.world_array_idx, i);

            if prev_is_static && col_obj.is_static_obj() {
                // static objects only need their aabbs set the first time
                self.num_skippable_statics += 1;
            } else {
                prev_is_static = false;
            }

            let mut aabb = col_obj
                .get_collision_shape()
                .get_aabb(col_obj.get_world_trans());

            aabb.min -= CBT;
            aabb.max += CBT;

            if !col_obj.is_static_obj() {
                let mut aabb2 = col_obj
                    .get_collision_shape()
                    .get_aabb(&col_obj.interp_world_trans);
                aabb2.min -= CBT;
                aabb2.max += CBT;
                aabb += aabb2;
            }

            debug_assert!(
                col_obj.is_static_obj() || (aabb.max - aabb.min).length_squared() < 1e12,
                "object #{i} {:?} has invalid aabb: {:?}",
                col_obj.user_idx,
                aabb
            );
            self.broadphase_pair_cache
                .set_aabb(col_obj, col_obj.get_broadphase_handle(), aabb);
        }
    }

    pub fn perform_discrete_collision_detection<T: ContactAddedCallback>(
        &mut self,
        skipped_pairs: &[(usize, usize)],
        contact_added_callback: &mut T,
    ) {
        self.update_aabbs();

        self.broadphase_pair_cache.calculate_overlapping_pairs();
        self.dispatcher1.dispatch_all_collision_pairs(
            &self.collision_objs,
            &mut self.broadphase_pair_cache,
            skipped_pairs,
            contact_added_callback,
        );
    }

    fn can_collide_pair(&self, body_a_idx: usize, body_b_idx: usize) -> bool {
        let body_a = &self.collision_objs[body_a_idx];
        let body_b = &self.collision_objs[body_b_idx];

        (body_a.is_active() || body_b.is_active())
            && body_a.has_contact_response()
            && body_b.has_contact_response()
            && self.broadphase_pair_cache.needs_collision(
                body_a.get_broadphase_handle(),
                body_b.get_broadphase_handle(),
            )
    }

    pub fn dispatch_pair<T: ContactAddedCallback>(
        &mut self,
        body_a_idx: usize,
        body_b_idx: usize,
        contact_added_callback: &mut T,
    ) -> bool {
        if !self.can_collide_pair(body_a_idx, body_b_idx) {
            return false;
        }

        self.dispatcher1.dispatch_pair(
            &self.collision_objs,
            body_a_idx,
            body_b_idx,
            contact_added_callback,
        )
    }

    fn car_ball_pair(
        &self,
        body_a_idx: usize,
        body_b_idx: usize,
    ) -> Option<(&RigidBody, &SphereShape, &RigidBody, &CompoundShape)> {
        let body_a = &self.collision_objs[body_a_idx];
        let body_b = &self.collision_objs[body_b_idx];

        let (sphere, obb) = match (body_a.user_idx, body_b.user_idx) {
            (UserInfoTypes::Ball, UserInfoTypes::Car) => (body_a, body_b),
            (UserInfoTypes::Car, UserInfoTypes::Ball) => (body_b, body_a),
            _ => return None,
        };
        let CollisionShapes::Sphere(sphere_shape) = sphere.get_collision_shape() else {
            return None;
        };
        let CollisionShapes::Compound(obb_shape) = obb.get_collision_shape() else {
            return None;
        };

        Some((sphere, sphere_shape, obb, obb_shape))
    }

    pub fn car_ball_separation(&self, pair: (usize, usize)) -> Option<f32> {
        let (sphere, sphere_shape, obb, obb_shape) = self.car_ball_pair(pair.0, pair.1)?;
        Some(sphere_obb_collision_alg::separation(
            sphere,
            sphere_shape,
            obb,
            obb_shape,
        ))
    }

    pub fn car_ball_contact_threshold(&self, pair: (usize, usize)) -> Option<f32> {
        let (sphere, _, obb, _) = self.car_ball_pair(pair.0, pair.1)?;
        Some(PersistentManifold::new(sphere, obb).contact_breaking_threshold)
    }

    pub fn car_ball_time_of_impact(
        &self,
        pair: (usize, usize),
        time_step: f32,
        include_accumulated_velocity: bool,
    ) -> Option<f32> {
        if !ENABLE_CAR_BALL_TOI {
            return None;
        }

        let (sphere, sphere_shape, obb, obb_shape) = self.car_ball_pair(pair.0, pair.1)?;
        let sphere_lin_vel = sphere.lin_vel
            + if include_accumulated_velocity {
                sphere.accum_lin_vel
            } else {
                Vec3A::ZERO
            };
        let obb_lin_vel = obb.lin_vel
            + if include_accumulated_velocity {
                obb.accum_lin_vel
            } else {
                Vec3A::ZERO
            };
        let obb_ang_vel = obb.ang_vel
            + if include_accumulated_velocity {
                obb.accum_ang_vel
            } else {
                Vec3A::ZERO
            };

        sphere_obb_collision_alg::time_of_impact(
            sphere,
            sphere_shape,
            sphere_lin_vel,
            obb,
            obb_shape,
            obb_lin_vel,
            obb_ang_vel,
            time_step,
        )
    }

    pub fn car_ball_pairs(&self) -> Vec<(usize, usize)> {
        let ball_indices: Vec<_> = self
            .collision_objs
            .iter()
            .filter(|body| body.user_idx == UserInfoTypes::Ball)
            .map(|body| body.world_array_idx)
            .collect();
        let car_indices: Vec<_> = self
            .collision_objs
            .iter()
            .filter(|body| body.user_idx == UserInfoTypes::Car)
            .map(|body| body.world_array_idx)
            .collect();

        let mut pairs = Vec::new();
        for ball_idx in ball_indices {
            for &car_idx in &car_indices {
                let pair = (ball_idx.min(car_idx), ball_idx.max(car_idx));
                if self.car_ball_pair(pair.0, pair.1).is_some()
                    && self.can_collide_pair(pair.0, pair.1)
                {
                    pairs.push(pair);
                }
            }
        }
        pairs
    }

    pub fn car_ball_toi_pairs(&self, time_step: f32) -> Vec<(usize, usize)> {
        self.car_ball_pairs()
            .into_iter()
            .filter(|&pair| {
                self.car_ball_separation(pair)
                    .is_some_and(|separation| separation > 0.0)
                    && self
                        .car_ball_time_of_impact(pair, time_step, true)
                        .is_some()
            })
            .collect()
    }

    pub(crate) fn quad_ray_test<T: QuadRayResultCallback>(
        ray_from: &[Vec3A; 4],
        ray_to: &[Vec3A; 4],
        co: &RigidBody,
        obj_idx: usize,
        result_callback: &mut T,
    ) {
        let (ray_from_local, ray_to_local) =
            if matches!(co.get_collision_shape(), CollisionShapes::TriangleMesh(_)) {
                (*ray_from, *ray_to)
            } else {
                let world_to_co = co.get_world_trans().transpose();
                (
                    [
                        world_to_co.transform_point3a(ray_from[0]),
                        world_to_co.transform_point3a(ray_from[1]),
                        world_to_co.transform_point3a(ray_from[2]),
                        world_to_co.transform_point3a(ray_from[3]),
                    ],
                    [
                        world_to_co.transform_point3a(ray_to[0]),
                        world_to_co.transform_point3a(ray_to[1]),
                        world_to_co.transform_point3a(ray_to[2]),
                        world_to_co.transform_point3a(ray_to[3]),
                    ],
                )
            };

        let mut rcb = BridgeTriQuadRayCallback {
            from: &ray_from_local,
            to: &ray_to_local,
            hit_fraction: result_callback.get_base().closest_hit_fraction,
            collision_obj: co,
            collision_obj_idx: obj_idx,
            result_callback,
        };

        let mut ray_info = QuadRayInfo::new(&ray_from_local, &ray_to_local);
        ray_info.lambda_max = rcb.hit_fraction;

        co.get_collision_shape()
            .perform_quad_raycast(&mut rcb, &mut ray_info);
    }
}
