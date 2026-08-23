use glam::{Affine3A, Quat, Vec3A};

use crate::bullet::{
    collision::{
        narrowphase::persistent_manifold::{ContactAddedCallback, PersistentManifold},
        shapes::{compound_shape::CompoundShape, sphere_shape::SphereShape},
    },
    dynamics::rigid_body::RigidBody,
    linear_math::{AffineExt, integrate_trans},
};

const TOI_MAX_ITERATIONS: usize = 128;
const TOI_DISTANCE_EPSILON: f32 = 1e-6;
const TOI_TIME_EPSILON: f32 = 1e-6;

fn integrate_obb_trans(
    start_trans: Affine3A,
    start_rot: Quat,
    lin_vel: Vec3A,
    ang_vel: Vec3A,
    time: f32,
) -> Affine3A {
    let mut trans = start_trans;
    let mut rot = start_rot;
    integrate_trans(&mut trans, &mut rot, lin_vel, ang_vel, time);
    trans
}

fn separation_at_transforms(
    sphere_pos: Vec3A,
    radius: f32,
    obb_trans: Affine3A,
    obb_shape: &CompoundShape,
) -> f32 {
    let child_world_trans = obb_trans * obb_shape.child_trans;
    let sphere_from_local = child_world_trans.inv_x_form(sphere_pos);
    let box_extents = obb_shape.child_shape.get_half_extents();
    let closest = sphere_from_local.clamp(-box_extents, box_extents);
    let center_distance = (sphere_from_local - closest).length();

    center_distance - radius - obb_shape.child_shape.get_margin()
}

pub(crate) fn separation(
    sphere_obj: &RigidBody,
    sphere_shape: &SphereShape,
    obb_obj: &RigidBody,
    obb_shape: &CompoundShape,
) -> f32 {
    separation_at_transforms(
        sphere_obj.get_world_trans().translation,
        sphere_shape.get_radius(),
        *obb_obj.get_world_trans(),
        obb_shape,
    )
}

pub(crate) fn time_of_impact(
    sphere_obj: &RigidBody,
    sphere_shape: &SphereShape,
    sphere_lin_vel: Vec3A,
    obb_obj: &RigidBody,
    obb_shape: &CompoundShape,
    obb_lin_vel: Vec3A,
    obb_ang_vel: Vec3A,
    time_step: f32,
) -> Option<f32> {
    let sphere_start = sphere_obj.get_world_trans().translation;
    let obb_start = *obb_obj.get_world_trans();
    let obb_start_rot = obb_obj.get_world_rot();
    let radius = sphere_shape.get_radius();

    let separation_at = |time: f32| {
        let sphere_pos = sphere_start + sphere_lin_vel * time;
        let obb_trans =
            integrate_obb_trans(obb_start, obb_start_rot, obb_lin_vel, obb_ang_vel, time);
        separation_at_transforms(sphere_pos, radius, obb_trans, obb_shape)
    };

    let mut separation = separation_at(0.0);
    if separation <= TOI_DISTANCE_EPSILON || time_step <= 0.0 {
        return None;
    }

    let box_radius = obb_shape.child_trans.translation.length()
        + (obb_shape.child_shape.get_half_extents()
            + Vec3A::splat(obb_shape.child_shape.get_margin()))
        .length();
    let speed_bound = (sphere_lin_vel - obb_lin_vel).length() + obb_ang_vel.length() * box_radius;
    if speed_bound <= f32::EPSILON {
        return None;
    }

    let mut lower = 0.0;
    let mut upper = None;
    for _ in 0..TOI_MAX_ITERATIONS {
        let advance = (separation / speed_bound).max(TOI_TIME_EPSILON);
        let next = (lower + advance).min(time_step);
        let next_separation = separation_at(next);

        if next_separation <= TOI_DISTANCE_EPSILON {
            upper = Some(next);
            break;
        }
        if next >= time_step {
            return None;
        }

        lower = next;
        separation = next_separation;
    }

    let mut upper = upper?;
    for _ in 0..16 {
        let middle = 0.5 * (lower + upper);
        if separation_at(middle) > TOI_DISTANCE_EPSILON {
            lower = middle;
        } else {
            upper = middle;
        }
    }

    Some(upper)
}

fn get_sphere_penetration(box_extents: Vec3A, sphere_from_local: Vec3A) -> (f32, Vec3A, Vec3A) {
    let mut min_dist = box_extents.x - sphere_from_local.x;
    let mut closest = sphere_from_local;
    closest.x = box_extents.x;
    let mut normal = Vec3A::X;

    let mut test_face = |face_dist: f32, axis: usize, sign: f32| {
        if face_dist < min_dist {
            min_dist = face_dist;
            closest = sphere_from_local;
            closest[axis] = box_extents[axis] * sign;
            normal = Vec3A::ZERO;
            normal[axis] = sign;
        }
    };

    test_face(box_extents.x + sphere_from_local.x, 0, -1.0);
    test_face(box_extents.y - sphere_from_local.y, 1, 1.0);
    test_face(box_extents.y + sphere_from_local.y, 1, -1.0);
    test_face(box_extents.z - sphere_from_local.z, 2, 1.0);
    test_face(box_extents.z + sphere_from_local.z, 2, -1.0);

    (min_dist, closest, normal)
}

pub fn process_collision<T: ContactAddedCallback>(
    sphere_obj: &RigidBody,
    sphere_shape: &SphereShape,
    obb_obj: &RigidBody,
    obb_shape: &CompoundShape,
    contact_added_callback: &mut T,
) -> Option<PersistentManifold> {
    let sphere_trans = sphere_obj.get_world_trans();
    let aabb_1 = sphere_shape.get_aabb(sphere_trans);

    let org_trans = obb_obj.get_world_trans();
    let aabb_2 = obb_shape.get_aabb(org_trans);

    if !aabb_1.intersects(&aabb_2) {
        return None;
    }

    let child_trans = &obb_shape.child_trans;
    let new_child_world_trans = org_trans * child_trans;

    let box_shape = &obb_shape.child_shape;
    let box_extents = box_shape.get_half_extents();

    let sphere_from_local = new_child_world_trans.inv_x_form(sphere_trans.translation);

    let mut closest = sphere_from_local.clamp(-box_extents, box_extents);
    let mut delta = sphere_from_local - closest;
    let dist_sq = delta.length_squared();

    let radius = sphere_shape.get_radius();
    let box_margin = box_shape.get_margin();

    let mut manifold = PersistentManifold::new(sphere_obj, obb_obj);
    let intersection_dist = radius + box_margin;
    let contact_dist = intersection_dist + manifold.contact_breaking_threshold;
    if dist_sq > contact_dist * contact_dist {
        return None;
    }

    let mut dist;
    if dist_sq > f32::EPSILON {
        dist = dist_sq.sqrt();
        delta /= dist;
    } else {
        (dist, closest, delta) = get_sphere_penetration(box_extents, sphere_from_local);
        dist *= -1.0;
    };

    let normal_on_box = new_child_world_trans.transform_vector3a(delta);
    let point_on_box = new_child_world_trans.transform_point3a(closest);
    let depth = dist - intersection_dist;

    // This is the official contact point on the box
    let point_on_box_plus_margin = point_on_box + (normal_on_box * box_margin);
    manifold.add_contact_point(
        sphere_obj,
        obb_obj,
        normal_on_box,
        point_on_box_plus_margin,
        depth,
        None,
        contact_added_callback,
    );
    manifold.refresh_contact_points(sphere_obj, obb_obj);

    if manifold.point_cache.is_empty() {
        None
    } else {
        Some(manifold)
    }
}

#[cfg(test)]
mod tests {
    use glam::{Affine3A, Vec3A};

    use super::time_of_impact;
    use crate::bullet::{
        collision::shapes::{
            box_shape::BoxShape, collision_shape::CollisionShapes, compound_shape::CompoundShape,
            sphere_shape::SphereShape,
        },
        dynamics::rigid_body::{RigidBody, RigidBodyConstructionInfo},
    };

    fn make_pair(sphere_x: f32) -> (RigidBody, RigidBody) {
        let sphere = SphereShape::new(0.5);
        let mut sphere_info = RigidBodyConstructionInfo::new(1.0, CollisionShapes::Sphere(sphere));
        sphere_info.start_world_trans.translation = Vec3A::new(sphere_x, 0.0, 0.0);

        let compound = CompoundShape::new(BoxShape::new(Vec3A::ONE), Affine3A::IDENTITY);
        let obb_info = RigidBodyConstructionInfo::new(1.0, CollisionShapes::Compound(compound));

        (RigidBody::new(sphere_info), RigidBody::new(obb_info))
    }

    fn pair_toi(sphere: &RigidBody, obb: &RigidBody, sphere_vel: Vec3A) -> Option<f32> {
        let CollisionShapes::Sphere(sphere_shape) = sphere.get_collision_shape() else {
            unreachable!();
        };
        let CollisionShapes::Compound(obb_shape) = obb.get_collision_shape() else {
            unreachable!();
        };

        time_of_impact(
            sphere,
            sphere_shape,
            sphere_vel,
            obb,
            obb_shape,
            Vec3A::ZERO,
            Vec3A::ZERO,
            1.0,
        )
    }

    #[test]
    fn finds_first_crossing_even_when_sphere_exits_during_tick() {
        let (sphere, obb) = make_pair(3.0);
        let toi = pair_toi(&sphere, &obb, Vec3A::new(-6.0, 0.0, 0.0)).unwrap();

        assert!((toi - 0.25).abs() < 1e-4, "unexpected TOI: {toi}");
    }

    #[test]
    fn ignores_pairs_already_touching() {
        let (sphere, obb) = make_pair(1.5);
        assert_eq!(pair_toi(&sphere, &obb, Vec3A::NEG_X), None);
    }

    #[test]
    fn ignores_separating_pairs() {
        let (sphere, obb) = make_pair(3.0);
        assert_eq!(pair_toi(&sphere, &obb, Vec3A::X), None);
    }

    #[test]
    fn converges_with_large_tangential_velocity() {
        let sphere = SphereShape::new(0.5);
        let mut sphere_info =
            RigidBodyConstructionInfo::new(1.0, CollisionShapes::Sphere(sphere));
        sphere_info.start_world_trans.translation = Vec3A::new(3.0, 0.0, 0.0);
        let sphere = RigidBody::new(sphere_info);

        let compound = CompoundShape::new(
            BoxShape::new(Vec3A::new(1.0, 10.0, 1.0)),
            Affine3A::IDENTITY,
        );
        let obb = RigidBody::new(RigidBodyConstructionInfo::new(
            1.0,
            CollisionShapes::Compound(compound),
        ));
        let CollisionShapes::Sphere(sphere_shape) = sphere.get_collision_shape() else {
            unreachable!();
        };
        let CollisionShapes::Compound(obb_shape) = obb.get_collision_shape() else {
            unreachable!();
        };

        let toi = time_of_impact(
            &sphere,
            sphere_shape,
            Vec3A::new(-200.0, 1000.0, 0.0),
            &obb,
            obb_shape,
            Vec3A::ZERO,
            Vec3A::ZERO,
            0.01,
        )
        .unwrap();

        assert!((toi - 0.0075).abs() < 1e-4, "unexpected TOI: {toi}");
    }
}
