use glam::Vec3A;

use crate::bullet::{
    collision::dispatch::quad_ray_callbacks::{
        ClosestQuadRayResultCallback, QuadRayResultCallback,
    },
    dynamics::{discrete_dynamics_world::DiscreteDynamicsWorld, rigid_body::RigidBody},
};

#[derive(Clone, Copy)]
pub struct VehicleRaycasterResult<'a> {
    pub hit_point_in_world: Vec3A,
    pub hit_normal_in_world: Vec3A,
    pub rigid_body: &'a RigidBody,
    /// Index of the body that was hit. The reference above borrows the world,
    /// so a wheel cannot hold on to it; the friction impulse is computed in a
    /// later pass and needs to look the body up again.
    pub ground_body_idx: usize,
}

pub struct VehicleRaycaster {
    added_filter_mask: u8,
}

impl VehicleRaycaster {
    pub const fn new(added_filter_mask: u8) -> Self {
        Self { added_filter_mask }
    }

    pub fn cast_rays<'a>(
        &self,
        collision_world: &'a DiscreteDynamicsWorld,
        from: &[Vec3A; 4],
        to: &[Vec3A; 4],
        ignore_obj: &RigidBody,
    ) -> [Option<VehicleRaycasterResult<'a>>; 4] {
        let mut ray_callback = ClosestQuadRayResultCallback::new(from, to, Some(ignore_obj));
        ray_callback.base.collision_filter_group |= self.added_filter_mask;
        collision_world.ray_test(from, to, &mut ray_callback);

        let mut results = [None; 4];

        for (i, result) in results.iter_mut().enumerate() {
            if ray_callback.has_hit(i)
                && let Some(co_idx) = ray_callback.base.collision_obj_idx[i]
            {
                let rb = &collision_world.bodies()[co_idx];
                if rb.has_contact_response() {
                    *result = Some(VehicleRaycasterResult {
                        rigid_body: rb,
                        ground_body_idx: co_idx,
                        hit_point_in_world: ray_callback.hit_point_world[i],
                        hit_normal_in_world: ray_callback.hit_normal_world[i].normalize_or_zero(),
                    });
                }
            }
        }

        results
    }
}
