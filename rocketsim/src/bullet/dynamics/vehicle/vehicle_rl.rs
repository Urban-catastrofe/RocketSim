use glam::Vec3A;

use super::{
    NUM_WHEELS,
    raycaster::VehicleRaycaster,
    wheel_info::{RaycastInfo, WheelInfo},
};
use crate::bullet::{
    collision::broadphase::CollisionFilterGroups,
    dynamics::{discrete_dynamics_world::DiscreteDynamicsWorld, rigid_body::RigidBody},
};

pub struct VehicleRL {
    raycaster: VehicleRaycaster,
    chassis_body_idx: usize,
    pub wheels: [WheelInfo; NUM_WHEELS],
}

impl VehicleRL {
    pub const fn new(chassis_body_idx: usize, wheels: [WheelInfo; NUM_WHEELS]) -> Self {
        Self {
            raycaster: VehicleRaycaster::new(CollisionFilterGroups::DropshotFloor as u8),
            chassis_body_idx,
            wheels,
        }
    }

    pub fn get_upwards_dir_from_wheel_contacts(&self, cb: &RigidBody) -> Vec3A {
        let mut sum_contact_dir = Vec3A::ZERO;
        for wheel in &self.wheels {
            if let Some(raycast_info) = wheel.raycast_info.as_ref() {
                sum_contact_dir += raycast_info.contact_normal;
            }
        }

        sum_contact_dir
            .try_normalize()
            .unwrap_or_else(|| cb.get_up_vector())
    }

    pub const fn get_num_wheels(&self) -> usize {
        self.wheels.len()
    }

    pub fn update_vehicle_first(
        &mut self,
        collision_world: &DiscreteDynamicsWorld,
        time_step: f32,
    ) {
        let chassis = &collision_world.bodies()[self.chassis_body_idx];
        let chassis_trans = chassis.get_world_trans();

        let mut sources = [Vec3A::ZERO; 4];
        let mut targets = [Vec3A::ZERO; 4];

        for (i, wheel) in self.wheels.iter_mut().enumerate() {
            (sources[i], targets[i]) = wheel.prepare_for_raycast(chassis_trans);
        }

        let ray_results = self
            .raycaster
            .cast_rays(collision_world, &sources, &targets, chassis);

        for (i, wheel) in self.wheels.iter_mut().enumerate() {
            if let Some(ray_result) = ray_results[i] {
                wheel.apply_ray_cast(chassis, ray_result, time_step, i < 2);
            } else {
                wheel.reset_wheel_suspension();
            }
        }
    }

    /// Compute each wheel's friction impulse, once the coefficients for this
    /// tick are known.
    ///
    /// This is deliberately a separate pass from
    /// [`Self::update_vehicle_first`]. The impulse is a product of the raycast
    /// geometry (which `update_vehicle_first` produces) and four per-wheel
    /// coefficients -- `lat_friction`, `long_friction`, `engine_force`,
    /// `brake` -- which `Car::update_wheels` produces afterwards. C++
    /// RocketSim computes the impulse inside the raycast pass, so it multiplies
    /// this tick's geometry by last tick's coefficients. Ground truth says RL
    /// does not: fitting the measured lateral impulse against both alignments
    /// puts the current tick well ahead (median |residual| 0.037 vs 0.064).
    ///
    /// It must still run before any non-accumulated impulse of this tick, so
    /// that the contact velocity it reads is the one at the start of the tick.
    /// The sticky force is accumulated (`accum = true`) and so does not count.
    pub fn update_vehicle_friction(
        &mut self,
        collision_world: &DiscreteDynamicsWorld,
        time_step: f32,
    ) {
        let bodies = collision_world.bodies();
        let chassis = &bodies[self.chassis_body_idx];

        for wheel in &mut self.wheels {
            let Some(RaycastInfo {
                contact_normal,
                contact_point,
                ground_body_idx,
                ..
            }) = wheel.raycast_info.as_ref()
            else {
                continue;
            };
            let (contact_normal, contact_point, ground_body_idx) =
                (*contact_normal, *contact_point, *ground_body_idx);

            let impulse = wheel.calc_friction_impulses(
                chassis,
                &bodies[ground_body_idx],
                contact_normal,
                contact_point,
                time_step,
            );

            if let Some(raycast_info) = wheel.raycast_info.as_mut() {
                raycast_info.impulse = impulse;
            }
        }
    }

    pub fn update_vehicle_second(
        &mut self,
        collision_world: &mut DiscreteDynamicsWorld,
        step: f32,
    ) {
        let chassis = &mut collision_world.bodies_mut()[self.chassis_body_idx];
        for wheel in &mut self.wheels {
            wheel.update_suspension(chassis, step);
        }

        // note: all suspension MUST be updated before impulses are applied
        for wheel in &mut self.wheels {
            wheel.apply_friction_impulses(chassis, step);
        }
    }
}
