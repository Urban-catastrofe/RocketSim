use glam::Vec3A;

use super::{contact_solver_info, solver_body::SolverBody, solver_constraint::SolverConstraint};
use crate::bullet::{
    collision::narrowphase::{
        manifold_point::ManifoldPoint,
        persistent_manifold::{
            CONTACT_BREAKING_THRESHOLD, ContactAddedCallback, ContactSolveInfo, PersistentManifold,
            SpecialContactSolveInfo, SpecialContactSolvePoint,
        },
    },
    dynamics::rigid_body::{CollisionFlags, RigidBody},
    linear_math::{integrate_trans, integrate_trans_no_rot, plane_space_1},
};

/// Coincident-point dedup for the ball's aggregated world contact, ported from
/// upstream PR #74 (VirxEC, "Collision fixes"). The arena mesh puts several
/// triangles under one contact, so the same physical touch can enter the
/// average a dozen times and drag the mean normal toward whichever surface
/// happens to own the most triangles.
const SPECIAL_CONTACT_DEDUP_NORMAL_DOT: f32 = 0.999;
const SPECIAL_CONTACT_DEDUP_SCALE: f32 = 0.1;

#[derive(Clone, Copy)]
struct SpecialContact {
    obj_idx: usize,
    pos_world_on_a: Vec3A,
    pos_world_on_b: Vec3A,
    normal_world_on_b: Vec3A,
    distance: f32,
    dedup_distance: f32,
}

impl SpecialContact {
    fn is_duplicate_of(self, other: Self) -> bool {
        let tolerance = self.dedup_distance.max(other.dedup_distance);
        let tolerance_sq = tolerance * tolerance;

        self.obj_idx == other.obj_idx
            && (self.pos_world_on_a - other.pos_world_on_a).length_squared() <= tolerance_sq
            && (self.pos_world_on_b - other.pos_world_on_b).length_squared() <= tolerance_sq
            && (self.distance - other.distance).abs() <= tolerance
            && self.normal_world_on_b.dot(other.normal_world_on_b)
                >= SPECIAL_CONTACT_DEDUP_NORMAL_DOT
    }
}

struct SpecialResolveInfo {
    pub obj_idx: usize,
    pub num_special_collisions: u16,
    pub total_normal: Vec3A,
    /// Sum of `rel_pos.length()` over the points: the distance from the body's
    /// centre to each contact point, i.e. about one ball radius. This is the
    /// lever arm, *not* a penetration depth -- see [`Self::total_penetration`].
    pub total_dist: f32,
    /// Sum of the points' own `distance_1`: the signed gap to the surface,
    /// negative when overlapping.
    ///
    /// Kept separate from `total_dist` because the two are different quantities
    /// that the aggregated constraint needs for different jobs. C++'s
    /// `convertContactSpecial` uses `total_dist` for both, which makes the
    /// constraint's `m_distance1` a positive radius, so `positional_error` takes
    /// the `penetration > 0` branch every single time and `m_rhsPenetration` is
    /// identically zero -- the ball is the one body in the arena that never gets
    /// a split-impulse push-out. C++ still gets one from the per-point
    /// constraints it also builds (the split-impulse loop has no `m_isSpecial`
    /// check); our port `continue`s before creating those, so without this the
    /// channel does not exist at all on our side.
    pub total_penetration: f32,
    /// The deepest point's own `distance_1`, kept alongside the mean so the two
    /// can be compared. Upstream PR #74 uses this in place of the mean.
    pub min_penetration: f32,
    pub restitution: f32,
    pub friction: f32,
    /// Points already folded into the average, for coincident-point dedup.
    special_contacts: Vec<SpecialContact>,
}

impl SpecialResolveInfo {
    pub const DEFAULT: Self = Self {
        obj_idx: 0,
        num_special_collisions: 0,
        total_normal: Vec3A::ZERO,
        total_dist: 0.0,
        total_penetration: 0.0,
        min_penetration: f32::MAX,
        restitution: 0.0,
        friction: 0.0,
        special_contacts: Vec::new(),
    };

    fn reset(&mut self) {
        self.obj_idx = 0;
        self.num_special_collisions = 0;
        self.total_normal = Vec3A::ZERO;
        self.total_dist = 0.0;
        self.total_penetration = 0.0;
        self.min_penetration = f32::MAX;
        self.restitution = 0.0;
        self.friction = 0.0;
        self.special_contacts.clear();
    }

    fn add_special_collision(
        &mut self,
        body0: &RigidBody,
        body1: &RigidBody,
        cp: &ManifoldPoint,
        rel_pos1: Vec3A,
        rel_pos2: Vec3A,
        contact_breaking_threshold: f32,
    ) {
        let (obj_idx, rel_pos) = if !body0.is_static_obj() {
            (body0.world_array_idx, rel_pos1)
        } else if !body1.is_static_obj() {
            (body1.world_array_idx, rel_pos2)
        } else {
            return;
        };

        let contact = SpecialContact {
            obj_idx,
            pos_world_on_a: cp.pos_world_on_a,
            pos_world_on_b: cp.pos_world_on_b,
            normal_world_on_b: cp.normal_world_on_b,
            distance: cp.distance_1,
            dedup_distance: (rel_pos.length() * SPECIAL_CONTACT_DEDUP_SCALE)
                .max(2.0 * contact_breaking_threshold),
        };

        if self
            .special_contacts
            .iter()
            .copied()
            .any(|existing| contact.is_duplicate_of(existing))
        {
            return;
        }

        self.special_contacts.push(contact);
        self.obj_idx = obj_idx;
        self.num_special_collisions += 1;
        self.friction = cp.combined_friction;
        self.restitution = cp.combined_restitution;
        self.total_normal += cp.normal_world_on_b;
        self.total_dist += rel_pos.length();
        self.total_penetration += cp.distance_1;
        self.min_penetration = self.min_penetration.min(cp.distance_1);
    }
}

pub struct SeqImpulseConstraintSolver {
    tmp_solver_body_pool: Vec<SolverBody>,
    tmp_solver_contact_constraint_pool: Vec<SolverConstraint>,
    tmp_solver_contact_friction_constraint_pool: Vec<SolverConstraint>,
    fixed_body_id: Option<usize>,
    least_squares_residual: f32,
    special_resolve_info: SpecialResolveInfo,
}

impl Default for SeqImpulseConstraintSolver {
    fn default() -> Self {
        Self {
            tmp_solver_body_pool: Vec::new(),
            tmp_solver_contact_constraint_pool: Vec::new(),
            tmp_solver_contact_friction_constraint_pool: Vec::new(),
            fixed_body_id: None,
            least_squares_residual: 0.0,
            special_resolve_info: SpecialResolveInfo::DEFAULT,
        }
    }
}

impl SeqImpulseConstraintSolver {
    fn get_or_init_solver_body(&mut self, rb: &mut RigidBody) -> usize {
        if let Some(companion_id) = rb.companion_id {
            return companion_id;
        }

        if !rb.is_static_obj() && rb.inv_mass != 0.0 {
            let solver_body_id = self.tmp_solver_body_pool.len();
            rb.companion_id = Some(solver_body_id);

            self.tmp_solver_body_pool.push(SolverBody::new(rb));
            return solver_body_id;
        }

        if let Some(fixed_body_id) = self.fixed_body_id {
            fixed_body_id
        } else {
            let solver_body_id = self.tmp_solver_body_pool.len();
            rb.companion_id = Some(solver_body_id);
            self.fixed_body_id = Some(solver_body_id);

            self.tmp_solver_body_pool.push(SolverBody::DEFAULT);
            solver_body_id
        }
    }

    pub fn solve_group<T: ContactAddedCallback>(
        &mut self,
        collision_objs: &mut [RigidBody],
        non_static_bodies: &[usize],
        manifolds: &mut Vec<PersistentManifold>,
        time_step: f32,
        contact_callback: &mut T,
    ) {
        self.solve_group_setup(
            collision_objs,
            non_static_bodies,
            manifolds,
            time_step,
            contact_callback.tracks_special_contacts(),
        );
        self.solve_group_iterations();
        self.solve_group_finish(collision_objs, time_step, contact_callback);
    }

    fn solve_group_setup(
        &mut self,
        collision_objs: &mut [RigidBody],
        non_static_bodies: &[usize],
        manifolds: &mut Vec<PersistentManifold>,
        time_step: f32,
        track_special_contacts: bool,
    ) {
        self.setup_solver_bodies(collision_objs, non_static_bodies);

        for manifold in manifolds.iter_mut() {
            debug_assert!(manifold.body0_idx < collision_objs.len());
            debug_assert!(manifold.body1_idx < collision_objs.len());
            debug_assert_ne!(manifold.body0_idx, manifold.body1_idx);
            let [body0, body1] = unsafe {
                collision_objs.get_disjoint_unchecked_mut([manifold.body0_idx, manifold.body1_idx])
            };

            let solver_body_id_a = self.get_or_init_solver_body(body0);
            let solver_body_id_b = self.get_or_init_solver_body(body1);

            debug_assert!(solver_body_id_a < self.tmp_solver_body_pool.len());
            debug_assert!(solver_body_id_b < self.tmp_solver_body_pool.len());
            debug_assert_ne!(solver_body_id_a, solver_body_id_b);
            let [solver_body_a, solver_body_b] = unsafe {
                self.tmp_solver_body_pool
                    .get_disjoint_unchecked_mut([solver_body_id_a, solver_body_id_b])
            };

            body0.companion_id = Some(solver_body_id_a);
            body1.companion_id = Some(solver_body_id_b);

            for cp in &mut manifold.point_cache {
                assert!(cp.distance_1 <= manifold.contact_processing_threshold);

                let rel_pos1 = cp.pos_world_on_a - body0.get_world_trans().translation;
                let rel_pos2 = cp.pos_world_on_b - body1.get_world_trans().translation;

                if cp.is_special {
                    self.special_resolve_info.add_special_collision(
                        body0,
                        body1,
                        cp,
                        rel_pos1,
                        rel_pos2,
                        manifold.contact_breaking_threshold,
                    );

                    // Skip normal contact processing for special contacts
                    continue;
                }

                let rb0 = solver_body_a.original_body.map(|_| &*body0);
                let rb1 = solver_body_b.original_body.map(|_| &*body1);
                let friction_idx = self.tmp_solver_contact_friction_constraint_pool.len();

                let mut contact_constraint = SolverConstraint::get_contact_constraint(
                    (solver_body_id_a, solver_body_id_b),
                    (solver_body_a, solver_body_b),
                    (rb0, rb1),
                    (rel_pos1, rel_pos2),
                    cp,
                    friction_idx,
                    time_step,
                );
                contact_constraint.body_idx_a = manifold.body0_idx;
                contact_constraint.body_idx_b = manifold.body1_idx;
                contact_constraint.rel_pos_a = rel_pos1;
                contact_constraint.rel_pos_b = rel_pos2;
                contact_constraint.manifold_point = Some(*cp);
                contact_constraint.relative_velocity_before = solver_body_a
                    .get_vel_in_local_point_no_delta(rel_pos1)
                    - solver_body_b.get_vel_in_local_point_no_delta(rel_pos2);
                self.tmp_solver_contact_constraint_pool
                    .push(contact_constraint);

                cp.calc_lat_friction_dir(solver_body_a, solver_body_b, rel_pos1, rel_pos2);

                self.tmp_solver_contact_friction_constraint_pool.push(
                    SolverConstraint::get_friction_constraint(
                        (solver_body_id_a, solver_body_id_b),
                        (solver_body_a, solver_body_b),
                        (rb0, rb1),
                        (rel_pos1, rel_pos2),
                        cp,
                        friction_idx,
                    ),
                );
            }
        }

        manifolds.clear();

        if self.special_resolve_info.num_special_collisions > 0 {
            let body = &mut collision_objs[self.special_resolve_info.obj_idx];
            self.convert_contact_special(body, time_step, track_special_contacts);
            self.special_resolve_info.reset();
        }
    }

    fn setup_solver_bodies(
        &mut self,
        collision_objs: &mut [RigidBody],
        non_static_bodies: &[usize],
    ) {
        self.fixed_body_id = None;

        self.tmp_solver_body_pool
            .reserve(non_static_bodies.len() + 1);
        self.tmp_solver_contact_constraint_pool
            .reserve(non_static_bodies.len() * 2);
        self.tmp_solver_contact_friction_constraint_pool
            .reserve(non_static_bodies.len() * 2);

        for rb in &mut *collision_objs {
            rb.companion_id = None;
        }

        for &rb_idx in non_static_bodies {
            let rb = &mut collision_objs[rb_idx];
            debug_assert_ne!(rb.inv_mass, 0.0);

            if !rb.is_active() {
                continue;
            }

            let solver_body_id = self.tmp_solver_body_pool.len();
            rb.companion_id = Some(solver_body_id);

            self.tmp_solver_body_pool.push(SolverBody::new(rb));
        }
    }

    fn convert_contact_special(
        &mut self,
        body: &RigidBody,
        time_step: f32,
        track_special_contacts: bool,
    ) {
        let sri = &self.special_resolve_info;
        let num_collisions = f32::from(sri.num_special_collisions);
        let distance = sri.total_dist / num_collisions;
        // Upstream PR #74 normalizes here. The mean of N unit normals is shorter
        // than one whenever the points disagree, and an un-normalized constraint
        // normal scales the whole impulse down by that shortfall.
        let normal_world_on_b = (sri.total_normal / num_collisions).normalize_or_zero();
        // The mean signed gap to the surface. `distance` above is the mean lever
        // arm and cannot serve as this: it is a length, so it is never negative.
        let mean_penetration = sri.total_penetration / num_collisions;

        let friction_idx = self.tmp_solver_contact_constraint_pool.len();

        let solver_body_id_a = body.companion_id.unwrap();
        let solver_body_id_b = if let Some(fixed_body_id) = self.fixed_body_id {
            fixed_body_id
        } else {
            let solver_body_id = self.tmp_solver_body_pool.len();
            self.fixed_body_id = Some(solver_body_id);

            self.tmp_solver_body_pool.push(SolverBody::DEFAULT);
            solver_body_id
        };

        let solver_body_a = &mut self.tmp_solver_body_pool[solver_body_id_a];

        let rel_pos1 = normal_world_on_b * -distance;
        let relaxation = contact_solver_info::SOR;

        let inv_time_step = 1.0 / time_step;
        let erp = contact_solver_info::ERP_2;

        let torque_axis_0 = rel_pos1.cross(normal_world_on_b);
        let angular_component_a = body
            .inv_inertia_tensor_world
            .mul_transpose_vec3a(torque_axis_0);

        let denom = {
            let vec = angular_component_a.cross(rel_pos1);
            body.inv_mass + normal_world_on_b.dot(vec)
        };
        let jac_diag_ab_inv = relaxation / denom;

        let (contact_normal_1, rel_pos1_cross_normal) = (normal_world_on_b, torque_axis_0);

        // Penetration recovery may only undo what this step could have created.
        // The narrowphase only generates points within `CONTACT_BREAKING_THRESHOLD`
        // of the surface, so a depth far beyond one step of travel was not
        // produced by the step: the body reached the solver already embedded, and
        // pushing it all the way out in one tick teleports it. That is not a
        // hypothetical -- `ball_corner_exit_low` and `ball_corner_exit_high` start
        // the ball ~75 UU inside the corner mesh with Rocket League applying no
        // impulse at all, and an uncapped recovery moves it 60 UU in a tick. It is
        // also what makes C++ blow up on exactly those two recordings, at 45.8 and
        // 45.2 UU per step against our 0.96 and 1.36.
        //
        // Anything deeper than the cap is still recovered, just over several ticks
        // rather than one, which is the same thing `ERP_2 < 1` already does.
        let max_recover = body.lin_vel.length() * time_step + CONTACT_BREAKING_THRESHOLD;
        let penetration = mean_penetration.max(-max_recover);

        let vel = body.get_vel_in_local_point(rel_pos1);
        let rel_vel = normal_world_on_b.dot(vel);

        let restitution = SolverConstraint::restitution_curve(rel_vel, sri.restitution).max(0.0);

        let (external_force_impulse_a, external_torque_impulse_a) = (
            solver_body_a.external_force_impulse,
            solver_body_a.external_torque_impulse,
        );

        let rel_vel = contact_normal_1.dot(solver_body_a.lin_vel + external_force_impulse_a)
            + rel_pos1_cross_normal.dot(solver_body_a.ang_vel + external_torque_impulse_a);

        let positional_error = if penetration > 0.0 {
            0.0
        } else {
            -penetration * erp * inv_time_step
        };

        let vel_error = restitution - rel_vel;

        let penetration_impulse = positional_error * jac_diag_ab_inv;
        let vel_impulse = vel_error * jac_diag_ab_inv;

        let (rhs, rhs_penetration) =
            if penetration > contact_solver_info::SPLIT_IMPULSE_PENETRATION_THRESHOLD {
                (penetration_impulse + vel_impulse, 0.0)
            } else {
                (vel_impulse, penetration_impulse)
            };

        let special_contact_info = track_special_contacts.then(|| SpecialContactSolveInfo {
            points: sri
                .special_contacts
                .iter()
                .map(|contact| SpecialContactSolvePoint {
                    pos_world_on_b: contact.pos_world_on_b,
                    normal_world_on_b: contact.normal_world_on_b,
                    distance: contact.distance,
                })
                .collect(),
            effective_normal: normal_world_on_b,
            mean_penetration,
            restitution_velocity: restitution,
            normal_impulse: 0.0,
            push_impulse: 0.0,
            relative_velocity_before: vel,
            relative_velocity_after: Vec3A::ZERO,
        });

        self.tmp_solver_contact_constraint_pool
            .push(SolverConstraint {
                solver_body_id_a,
                solver_body_id_b,
                angular_component_a,
                jac_diag_ab_inv,
                contact_normal_1,
                rel_pos1_cross_normal,
                rhs,
                rhs_penetration,
                friction: sri.friction,
                body_idx_a: sri.obj_idx,
                rel_pos_a: rel_pos1,
                special_contact_info,
                lower_limit: 0.0,
                upper_limit: 1e10,
                ..Default::default()
            });

        let vel = solver_body_a.get_vel_in_local_point_no_delta(rel_pos1);
        let rel_vel = normal_world_on_b.dot(vel);

        let mut lateral_friction_dir_1 = vel - normal_world_on_b * rel_vel;
        let lat_rel_vel = lateral_friction_dir_1.length_squared();

        if lat_rel_vel > f32::EPSILON {
            lateral_friction_dir_1 *= 1.0 / lat_rel_vel.sqrt();
        } else {
            lateral_friction_dir_1 = plane_space_1(normal_world_on_b);
        }

        // addFrictionConstraint
        let (contact_normal_1, rel_pos1_cross_normal, angular_component_a) = {
            let torque_axis = rel_pos1.cross(lateral_friction_dir_1);

            (
                lateral_friction_dir_1,
                torque_axis,
                body.inv_inertia_tensor_world
                    .mul_transpose_vec3a(torque_axis),
            )
        };

        let denom = {
            let vec = angular_component_a.cross(rel_pos1);
            body.inv_mass + lateral_friction_dir_1.dot(vec)
        };
        let jac_diag_ab_inv = relaxation / denom;

        let rel_vel = contact_normal_1.dot(solver_body_a.lin_vel + external_force_impulse_a)
            + rel_pos1_cross_normal.dot(solver_body_a.ang_vel + external_torque_impulse_a);

        let vel_error = -rel_vel;
        let vel_impulse = vel_error * jac_diag_ab_inv;

        self.tmp_solver_contact_friction_constraint_pool
            .push(SolverConstraint {
                friction_idx,
                solver_body_id_a,
                solver_body_id_b,
                contact_normal_1,
                rel_pos1_cross_normal,
                angular_component_a,
                jac_diag_ab_inv,
                rhs: vel_impulse,
                lower_limit: -sri.friction,
                upper_limit: sri.friction,
                friction: sri.friction,
                ..Default::default()
            });
    }

    fn solve_group_split_impulse_iterations(&mut self) {
        let n = self.tmp_solver_contact_constraint_pool.len();
        // One convergence bit per contact, saturating at 128 bits (the old u64
        // mask panicked at >=64 contacts, e.g. 3v3 pile-ups). A group with more
        // than 128 contacts — unrealistic in Rocketsim — disables the mask and
        // simply runs every contact on every iteration (correct, no early-exit).
        let masked = n <= 128;
        let mut should_run = if n >= 128 {
            u128::MAX
        } else {
            (1u128 << n) - 1
        };

        for _ in 0..contact_solver_info::NUM_ITERATIONS {
            for (i, contact) in self
                .tmp_solver_contact_constraint_pool
                .iter_mut()
                .enumerate()
            {
                // `masked` short-circuits so the shift never reaches 128.
                if masked && should_run & (1u128 << i) == 0 {
                    continue;
                }

                debug_assert_ne!(contact.solver_body_id_a, contact.solver_body_id_b);
                let [body_a, body_b] = unsafe {
                    self.tmp_solver_body_pool.get_disjoint_unchecked_mut([
                        contact.solver_body_id_a,
                        contact.solver_body_id_b,
                    ])
                };

                let residual = contact.resolve_split_penetration_impulse(body_a, body_b);
                if masked && residual * residual == 0.0 {
                    should_run ^= 1u128 << i;
                }
            }

            if masked && should_run == 0 {
                break;
            }
        }
    }

    fn solve_single_iteration(&mut self) -> f32 {
        let mut least_squares_residual = 0.0;

        for contact in &mut self.tmp_solver_contact_constraint_pool {
            if contact.is_special {
                continue;
            }

            debug_assert_ne!(contact.solver_body_id_a, contact.solver_body_id_b);
            let [body_a, body_b] = unsafe {
                self.tmp_solver_body_pool.get_disjoint_unchecked_mut([
                    contact.solver_body_id_a,
                    contact.solver_body_id_b,
                ])
            };

            let residual = contact.resolve_single_constraint_row_lower_limit(body_a, body_b);
            least_squares_residual = (residual * residual).max(least_squares_residual);
        }

        for contact in &mut self.tmp_solver_contact_friction_constraint_pool {
            let total_impulse =
                self.tmp_solver_contact_constraint_pool[contact.friction_idx].applied_impulse;
            if total_impulse <= 0.0 {
                continue;
            }

            let limit = contact.friction * total_impulse;
            contact.lower_limit = -limit;
            contact.upper_limit = limit;

            debug_assert_ne!(contact.solver_body_id_a, contact.solver_body_id_b);
            let [body_a, body_b] = unsafe {
                self.tmp_solver_body_pool.get_disjoint_unchecked_mut([
                    contact.solver_body_id_a,
                    contact.solver_body_id_b,
                ])
            };

            let residual = contact.resolve_single_constraint_row_generic(body_a, body_b);
            least_squares_residual = (residual * residual).max(least_squares_residual);
        }

        least_squares_residual
    }

    fn solve_group_iterations(&mut self) {
        self.solve_group_split_impulse_iterations();

        for _ in 0..contact_solver_info::NUM_ITERATIONS {
            self.least_squares_residual = self.solve_single_iteration();
            if self.least_squares_residual == 0.0 {
                break;
            }
        }
    }

    fn solve_group_finish<T: ContactAddedCallback>(
        &mut self,
        collision_objs: &mut [RigidBody],
        time_step: f32,
        contact_callback: &mut T,
    ) {
        for contact in &self.tmp_solver_contact_constraint_pool {
            if let Some(mut info) = contact.special_contact_info.clone() {
                let solver_body = &self.tmp_solver_body_pool[contact.solver_body_id_a];
                info.normal_impulse = contact.applied_impulse;
                info.push_impulse = contact.applied_push_impulse;
                info.relative_velocity_after =
                    solver_body.get_vel_in_local_point_with_delta(contact.rel_pos_a);
                contact_callback.special_contact_solved(info, &collision_objs[contact.body_idx_a]);
                continue;
            }
            let Some(manifold_point) = contact.manifold_point else {
                continue;
            };
            let solver_body_a = &self.tmp_solver_body_pool[contact.solver_body_id_a];
            let solver_body_b = &self.tmp_solver_body_pool[contact.solver_body_id_b];
            let relative_velocity_after = solver_body_a
                .get_vel_in_local_point_with_delta(contact.rel_pos_a)
                - solver_body_b.get_vel_in_local_point_with_delta(contact.rel_pos_b);
            contact_callback.contact_solved(
                ContactSolveInfo {
                    manifold_point,
                    normal_impulse: contact.applied_impulse,
                    push_impulse: contact.applied_push_impulse,
                    relative_velocity_before: contact.relative_velocity_before,
                    relative_velocity_after,
                },
                &collision_objs[contact.body_idx_a],
                &collision_objs[contact.body_idx_b],
            );
        }

        // writeBackBodies
        for solver in &mut self.tmp_solver_body_pool {
            let Some(body) = solver.original_body.map(|idx| &mut collision_objs[idx]) else {
                continue;
            };

            solver.lin_vel += solver.delta_lin_vel;
            solver.ang_vel += solver.delta_ang_vel;

            if solver.push_vel.length_squared() != 0.0 || solver.turn_vel.length_squared() != 0.0 {
                if body.collision_flags & CollisionFlags::NoAngularMotion != 0 {
                    integrate_trans_no_rot(
                        &mut solver.world_trans.translation,
                        solver.push_vel,
                        time_step,
                    );
                } else {
                    integrate_trans(
                        &mut solver.world_trans,
                        &mut solver.world_rot,
                        solver.push_vel,
                        solver.turn_vel * contact_solver_info::SPLIT_IMPULSE_TURN_ERP,
                        time_step,
                    );
                }
            }

            body.set_lin_vel(solver.lin_vel + solver.external_force_impulse);
            body.set_ang_vel(solver.ang_vel + solver.external_torque_impulse);

            body.set_world_trans(solver.world_trans);
        }

        self.tmp_solver_body_pool.clear();
        self.tmp_solver_contact_constraint_pool.clear();
        self.tmp_solver_contact_friction_constraint_pool.clear();
    }
}
