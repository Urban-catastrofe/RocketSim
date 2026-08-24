use glam::Vec3A;

use super::{
    constraint_solver::seq_impulse_constraint_solver::SeqImpulseConstraintSolver,
    rigid_body::{ActivationState, RigidBody},
};
use crate::bullet::{
    collision::{
        broadphase::{CollisionFilterGroups, GridBroadphase},
        dispatch::{
            collision_world::CollisionWorld,
            quad_ray_callbacks::{QuadRayCallback, QuadRayResultCallback},
        },
        narrowphase::persistent_manifold::ContactAddedCallback,
    },
    dynamics::rigid_body::Impulse,
};

pub struct DiscreteDynamicsWorld {
    collision_world: CollisionWorld,
    solver: SeqImpulseConstraintSolver,
    dynamic_body_idcs: Vec<usize>,
    gravity: Vec3A,
}

impl DiscreteDynamicsWorld {
    pub fn new(pair_cache: GridBroadphase, gravity: Vec3A) -> Self {
        Self {
            collision_world: CollisionWorld::new(pair_cache),
            solver: SeqImpulseConstraintSolver::default(),
            dynamic_body_idcs: Vec::new(),
            gravity,
        }
    }

    #[inline]
    pub fn bodies_mut(&mut self) -> &mut [RigidBody] {
        &mut self.collision_world.collision_objs
    }

    #[inline]
    pub fn bodies(&self) -> &[RigidBody] {
        &self.collision_world.collision_objs
    }

    #[inline]
    pub fn set_gravity(&mut self, gravity: Vec3A) {
        self.gravity = gravity;
    }

    pub fn ray_test<T: QuadRayResultCallback>(
        &self,
        ray_from_world: &[Vec3A; 4],
        ray_to_world: &[Vec3A; 4],
        result_callback: &mut T,
    ) {
        let mut ray_cb = QuadRayCallback::new(
            ray_from_world,
            ray_to_world,
            &self.collision_world,
            result_callback,
        );

        self.collision_world.broadphase_pair_cache.ray_test(
            ray_from_world,
            ray_to_world,
            &mut ray_cb,
        );
    }

    #[inline]
    fn add_collision_obj(&mut self, body: RigidBody, group: u8, mask: u8) -> usize {
        self.collision_world.add_collision_obj(body, group, mask)
    }

    pub fn add_rigid_body_default(&mut self, body: RigidBody) -> usize {
        let (group, mask) = if body.is_static_obj() {
            (
                CollisionFilterGroups::Static as u8,
                CollisionFilterGroups::ALL ^ CollisionFilterGroups::Static,
            )
        } else {
            (
                CollisionFilterGroups::Default as u8,
                CollisionFilterGroups::ALL,
            )
        };

        let rb_idx = self.add_collision_obj(body, group, mask);

        let rb = &mut self.collision_world.collision_objs[rb_idx];
        if rb.is_static_obj() {
            rb.set_activation_state(ActivationState::Sleeping);
        } else {
            self.dynamic_body_idcs.push(rb_idx);
        }

        rb_idx
    }

    pub fn add_rigid_body(&mut self, body: RigidBody, group: u8, mask: u8) -> usize {
        let rb_idx = self.add_collision_obj(body, group, mask);

        let rb = &mut self.collision_world.collision_objs[rb_idx];
        if rb.is_static_obj() {
            rb.set_activation_state(ActivationState::Sleeping);
        } else {
            self.dynamic_body_idcs.push(rb_idx);
        }

        rb_idx
    }

    fn apply_gravity(&mut self, time_step: f32) {
        for &body in &self.dynamic_body_idcs {
            let body = &mut self.collision_world.collision_objs[body];
            if body.is_active() {
                body.add_impulse(None, Impulse::Linear(self.gravity * time_step), false, true);
            }
        }
    }

    fn predict_unconstraint_motion(&mut self, time_step: f32) {
        for &body in &self.dynamic_body_idcs {
            let body = &mut self.collision_world.collision_objs[body];
            debug_assert!(!body.is_static_obj());

            body.apply_damping(time_step);
            let predicted_trans = body.predict_integration_trans(time_step);
            body.interp_world_trans = predicted_trans;
        }
    }

    #[inline]
    fn solve_constraints<T: ContactAddedCallback>(
        &mut self,
        time_step: f32,
        contact_callback: &mut T,
    ) {
        self.solver.solve_group(
            &mut self.collision_world.collision_objs,
            &self.dynamic_body_idcs,
            &mut self.collision_world.dispatcher1.manifolds,
            time_step,
            contact_callback,
        );
    }

    fn integrate_trans_internal(&mut self, time_step: f32) {
        for &body in &self.dynamic_body_idcs {
            let body = &mut self.collision_world.collision_objs[body];

            debug_assert!(!body.is_static_obj());
            if !body.is_active() {
                continue;
            }

            let predicted_trans = body.predict_integration_trans(time_step);
            body.set_center_of_mass_trans(predicted_trans);
        }
    }

    fn integrate_trans(&mut self, time_step: f32) {
        if !self.dynamic_body_idcs.is_empty() {
            self.integrate_trans_internal(time_step);
        }
    }

    fn consume_accumulated_velocities(&mut self) {
        for &body_idx in &self.dynamic_body_idcs {
            let body = &mut self.collision_world.collision_objs[body_idx];
            body.accum_lin_vel = Vec3A::ZERO;
            body.accum_ang_vel = Vec3A::ZERO;
        }
    }

    fn solve_car_ball_toi_pairs<T: ContactAddedCallback>(
        &mut self,
        time_step: f32,
        mut pairs: Vec<(usize, usize)>,
        contact_added_callback: &mut T,
    ) {
        let mut remaining_time = time_step;

        while !pairs.is_empty() {
            let mut earliest = None;
            for (pair_idx, &pair) in pairs.iter().enumerate() {
                let toi = if self
                    .collision_world
                    .car_ball_separation(pair)
                    .is_some_and(|separation| separation <= 0.0)
                {
                    Some(0.0)
                } else {
                    self.collision_world
                        .car_ball_time_of_impact(pair, remaining_time, false)
                };

                if let Some(toi) = toi
                    && earliest.is_none_or(|(_, earliest_toi): (usize, f32)| toi < earliest_toi)
                {
                    earliest = Some((pair_idx, toi));
                }
            }

            let Some((pair_idx, toi)) = earliest else {
                let mut resolved_pairs = Vec::new();
                for (pair_idx, &(body_a_idx, body_b_idx)) in pairs.iter().enumerate() {
                    if self.collision_world.dispatch_pair(
                        body_a_idx,
                        body_b_idx,
                        contact_added_callback,
                    ) {
                        resolved_pairs.push(pair_idx);
                    }
                }
                if resolved_pairs.is_empty() {
                    break;
                }

                for pair_idx in resolved_pairs.into_iter().rev() {
                    pairs.swap_remove(pair_idx);
                }
                self.solve_constraints(remaining_time.max(f32::EPSILON), contact_added_callback);
                continue;
            };

            if toi > 0.0 {
                self.integrate_trans(toi);
                remaining_time -= toi;
            }

            let (body_a_idx, body_b_idx) = pairs.swap_remove(pair_idx);
            if self
                .collision_world
                .dispatch_pair(body_a_idx, body_b_idx, contact_added_callback)
            {
                self.solve_constraints(remaining_time.max(f32::EPSILON), contact_added_callback);
            }
        }

        self.integrate_trans(remaining_time);
    }

    fn update_activation_state(&mut self, time_step: f32) {
        for &body in &self.dynamic_body_idcs {
            let body = &mut self.collision_world.collision_objs[body];
            body.update_activation_state(time_step);
        }
    }

    pub fn clear_accum_forces(&mut self) {
        for &body in &self.dynamic_body_idcs {
            self.collision_world.collision_objs[body].clear_accum_vels();
        }
    }

    fn internal_single_step_simulation<T: ContactAddedCallback>(
        &mut self,
        time_step: f32,
        contact_added_callback: &mut T,
    ) {
        self.predict_unconstraint_motion(time_step);

        let all_car_ball_pairs = self.collision_world.car_ball_pairs();
        let toi_pairs = self.collision_world.car_ball_toi_pairs(time_step);
        let mut pending_pairs: Vec<_> = all_car_ball_pairs
            .into_iter()
            .filter(|pair| {
                toi_pairs.contains(pair)
                    || self
                        .collision_world
                        .car_ball_separation(*pair)
                        .zip(self.collision_world.car_ball_contact_threshold(*pair))
                        .is_some_and(|(separation, threshold)| separation > threshold)
            })
            .collect();

        self.collision_world
            .perform_discrete_collision_detection(&toi_pairs, contact_added_callback);

        for manifold in &self.collision_world.dispatcher1.manifolds {
            let pair = (
                manifold.body0_idx.min(manifold.body1_idx),
                manifold.body0_idx.max(manifold.body1_idx),
            );
            if self.collision_world.car_ball_separation(pair).is_some() {
                pending_pairs.retain(|pending| *pending != pair);
            }
        }

        self.solve_constraints(time_step, contact_added_callback);
        let has_toi_after_solve = pending_pairs.iter().any(|&pair| {
            self.collision_world
                .car_ball_separation(pair)
                .is_some_and(|separation| separation <= 0.0)
                || self
                    .collision_world
                    .car_ball_time_of_impact(pair, time_step, false)
                    .is_some()
        });
        if toi_pairs.is_empty() && !has_toi_after_solve {
            self.integrate_trans(time_step);
        } else {
            // The first solve has consumed gravity and control impulses. Later TOI
            // solves must only see velocity changes produced by their contacts.
            self.consume_accumulated_velocities();
            self.solve_car_ball_toi_pairs(time_step, pending_pairs, contact_added_callback);
        }
        self.update_activation_state(time_step);
    }

    pub fn step_simulation<T: ContactAddedCallback>(
        &mut self,
        time_step: f32,
        contact_added_callback: &mut T,
    ) {
        self.apply_gravity(time_step);
        self.internal_single_step_simulation(time_step, contact_added_callback);
    }
}
