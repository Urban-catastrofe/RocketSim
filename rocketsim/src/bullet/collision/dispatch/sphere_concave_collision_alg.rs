use glam::{Affine3A, Vec3A};

use crate::bullet::{
    collision::{
        narrowphase::persistent_manifold::{ContactAddedCallback, PersistentManifold},
        shapes::{
            bvh_triangle_mesh_shape::BvhTriangleMeshShape, sphere_shape::SphereShape,
            triangle_callback::ProcessTriangle, triangle_shape::TriangleShape,
        },
    },
    dynamics::rigid_body::RigidBody,
    linear_math::AffineExt,
};

struct SphereTriangleCallback<'a, T: ContactAddedCallback> {
    pub manifold: PersistentManifold,
    pub convex_obj: &'a RigidBody,
    pub tri_obj: &'a RigidBody,
    contact_added_callback: &'a mut T,
    sphere_center: Vec3A,
    sphere_radius: f32,
}

#[derive(Clone, Copy)]
struct SweptContact {
    toi: f32,
    normal: Vec3A,
    point: Vec3A,
    triangle_idx: usize,
}

struct SweptSphereTriangleCallback {
    start: Vec3A,
    delta: Vec3A,
    radius: f32,
    earliest: Option<SweptContact>,
}

impl ProcessTriangle for SweptSphereTriangleCallback {
    fn process_triangle(&mut self, triangle: &TriangleShape, triangle_idx: usize) {
        let mut toi = 0.0;

        // Conservative advancement on the exact point-to-triangle distance.
        // The closest feature may change from a face to an edge while moving,
        // so recompute it after every advance rather than casting against the
        // triangle plane alone.
        for _ in 0..32 {
            let center = self.start + self.delta * toi;
            let obj_to_points = [
                center - triangle.points[0],
                center - triangle.points[1],
                center - triangle.points[2],
            ];
            let point = triangle.closest_point(&obj_to_points);
            let offset = center - point;
            let distance = offset.length();
            let separation = distance - self.radius;
            let normal = if distance > f32::EPSILON {
                offset / distance
            } else {
                triangle.normal
            };

            if separation <= 1e-5 {
                let contact = SweptContact {
                    toi,
                    normal,
                    point,
                    triangle_idx,
                };
                if self.earliest.is_none_or(|earliest| toi < earliest.toi) {
                    self.earliest = Some(contact);
                }
                return;
            }

            let closing_distance = -self.delta.dot(normal);
            if closing_distance <= 1e-6 {
                return;
            }

            toi += separation / closing_distance;
            if toi > 1.0 {
                return;
            }
        }
    }
}

fn is_entering_soccar_goal_rim_contact(
    point_world: Vec3A,
    center_world: Vec3A,
    movement_world: Vec3A,
) -> bool {
    let point = point_world * crate::consts::BT_TO_UU;
    let center = center_world * crate::consts::BT_TO_UU;
    let movement = movement_world * crate::consts::BT_TO_UU;
    let near_goal_face =
        (point.y.abs() - crate::consts::goal::SOCCAR_GOAL_SCORE_BASE_THRESHOLD_Y).abs() < 32.0;
    let near_post = (point.x.abs() - crate::consts::goal::SOCCAR_GOAL_HALF_WIDTH).abs() < 160.0
        && point.z < crate::consts::goal::SOCCAR_GOAL_HEIGHT + 120.0;
    let near_crossbar = (point.z - crate::consts::goal::SOCCAR_GOAL_HEIGHT).abs() < 160.0
        && point.x.abs() < crate::consts::goal::SOCCAR_GOAL_HALF_WIDTH + 160.0;

    near_goal_face && (near_post || near_crossbar) && center.y * movement.y > 0.0
}

fn is_soccar_crossbar_contact(point_world: Vec3A) -> bool {
    let point = point_world * crate::consts::BT_TO_UU;
    (point.z - crate::consts::goal::SOCCAR_GOAL_HEIGHT).abs() < 160.0
        && point.x.abs() < crate::consts::goal::SOCCAR_GOAL_HALF_WIDTH + 160.0
}

fn is_soccar_post_contact(point_world: Vec3A) -> bool {
    let point = point_world * crate::consts::BT_TO_UU;
    (point.x.abs() - crate::consts::goal::SOCCAR_GOAL_HALF_WIDTH).abs() < 160.0
        && point.z < crate::consts::goal::SOCCAR_GOAL_HEIGHT + 120.0
}

fn can_resolve_goal_sweep(
    point_world: Vec3A,
    normal_world: Vec3A,
    movement_world: Vec3A,
    radius: f32,
) -> bool {
    let point = point_world * crate::consts::BT_TO_UU;
    let within_outer_post = point.x.abs() <= crate::consts::goal::SOCCAR_GOAL_HALF_WIDTH + 32.0;
    let crossbar_only =
        is_soccar_crossbar_contact(point_world) && !is_soccar_post_contact(point_world);
    let crosses_crossbar = !crossbar_only || movement_world.z.abs() > radius * 0.005;
    let above_post_base =
        !is_soccar_post_contact(point_world) || point.z > radius * crate::consts::BT_TO_UU;

    within_outer_post
        && crosses_crossbar
        && above_post_base
        && normal_world.y.abs() >= 0.7
        && movement_world.y.abs() <= radius * 0.25
}

fn may_reach_soccar_goal_rim(start_world: Vec3A, end_world: Vec3A, radius: f32) -> bool {
    let scale = crate::consts::BT_TO_UU;
    let start = start_world * scale;
    let end = end_world * scale;
    let radius = radius * scale;
    let movement = end - start;
    if start.y * movement.y <= 0.0 || movement.y.abs() > radius * 0.25 {
        return false;
    }

    let near_goal_face = (start.y.abs() - crate::consts::goal::SOCCAR_GOAL_SCORE_BASE_THRESHOLD_Y)
        .abs()
        .min((end.y.abs() - crate::consts::goal::SOCCAR_GOAL_SCORE_BASE_THRESHOLD_Y).abs())
        <= radius + 32.0;
    let near_post = (start.x.abs() - crate::consts::goal::SOCCAR_GOAL_HALF_WIDTH)
        .abs()
        .min((end.x.abs() - crate::consts::goal::SOCCAR_GOAL_HALF_WIDTH).abs())
        <= radius + 160.0
        && start.z.min(end.z) <= crate::consts::goal::SOCCAR_GOAL_HEIGHT + radius + 120.0;
    let near_crossbar = (start.z - crate::consts::goal::SOCCAR_GOAL_HEIGHT)
        .abs()
        .min((end.z - crate::consts::goal::SOCCAR_GOAL_HEIGHT).abs())
        <= radius + 160.0
        && start.x.abs().min(end.x.abs())
            <= crate::consts::goal::SOCCAR_GOAL_HALF_WIDTH + radius + 160.0;

    near_goal_face && (near_post || near_crossbar)
}

impl<'a, T: ContactAddedCallback> SphereTriangleCallback<'a, T> {
    pub fn new(
        convex_obj: &'a RigidBody,
        tri_obj: &'a RigidBody,
        sphere_center: Vec3A,
        sphere_radius: f32,
        contact_added_callback: &'a mut T,
    ) -> Self {
        Self {
            manifold: PersistentManifold::new(convex_obj, tri_obj),
            convex_obj,
            tri_obj,
            sphere_center,
            sphere_radius,
            contact_added_callback,
        }
    }
}

impl<T: ContactAddedCallback> ProcessTriangle for SphereTriangleCallback<'_, T> {
    fn process_triangle(&mut self, triangle: &TriangleShape, triangle_idx: usize) {
        let Some(contact_info) = triangle.intersect_sphere(
            self.sphere_center,
            self.sphere_radius,
            self.manifold.contact_breaking_threshold,
        ) else {
            return;
        };

        let normal_on_b = self
            .tri_obj
            .get_world_trans()
            .transform_vector3a(contact_info.result_normal);
        let point_in_world = self
            .tri_obj
            .get_world_trans()
            .transform_point3a(contact_info.contact_point);

        self.manifold.add_contact_point(
            self.convex_obj,
            self.tri_obj,
            normal_on_b,
            point_in_world,
            contact_info.depth,
            Some(triangle_idx),
            self.contact_added_callback,
        );
    }
}

pub fn process_collision<T: ContactAddedCallback>(
    convex_obj: &RigidBody,
    sphere_shape: &SphereShape,
    concave_obj: &RigidBody,
    tri_mesh: &BvhTriangleMeshShape,
    contact_added_callback: &mut T,
) -> Option<PersistentManifold> {
    let xform1 = convex_obj.get_world_trans();
    let xform2 = concave_obj.get_world_trans().transpose();
    let convex_in_triangle_space = Affine3A {
        matrix3: xform2.matrix3 * xform1.matrix3,
        translation: xform2.transform_point3a(xform1.translation),
    };

    let mut convex_triangle_callback = SphereTriangleCallback::new(
        convex_obj,
        concave_obj,
        convex_in_triangle_space.translation,
        sphere_shape.get_radius(),
        contact_added_callback,
    );

    let aabb = sphere_shape.get_aabb(&convex_in_triangle_space);
    tri_mesh.process_all_triangles(&mut convex_triangle_callback, &aabb);

    if !convex_triangle_callback.manifold.point_cache.is_empty() {
        convex_triangle_callback
            .manifold
            .refresh_contact_points(convex_obj, concave_obj);
        let movement_world = convex_obj.interp_world_trans.translation - xform1.translation;
        let separated_crossbar_contact = convex_triangle_callback
            .manifold
            .point_cache
            .iter()
            .all(|point| point.distance_1 > 0.0)
            && convex_triangle_callback
                .manifold
                .point_cache
                .iter()
                .any(|point| {
                    is_entering_soccar_goal_rim_contact(
                        point.pos_world_on_b,
                        xform1.translation,
                        movement_world,
                    ) && is_soccar_crossbar_contact(point.pos_world_on_b)
                });
        if separated_crossbar_contact {
            return None;
        }
        return Some(convex_triangle_callback.manifold);
    }

    // The broadphase covers both the current and predicted transforms. Complete
    // that swept test in the narrowphase so a fast sphere cannot cross a thin or
    // curved mesh feature for an entire tick.
    let predicted_center = xform2.transform_point3a(convex_obj.interp_world_trans.translation);
    if predicted_center == convex_in_triangle_space.translation {
        return None;
    }
    if !may_reach_soccar_goal_rim(
        xform1.translation,
        convex_obj.interp_world_trans.translation,
        sphere_shape.get_radius(),
    ) {
        return None;
    }

    let mut swept_callback = SweptSphereTriangleCallback {
        start: convex_in_triangle_space.translation,
        delta: predicted_center - convex_in_triangle_space.translation,
        radius: sphere_shape.get_radius(),
        earliest: None,
    };
    let predicted_xform = Affine3A {
        matrix3: convex_in_triangle_space.matrix3,
        translation: predicted_center,
    };
    let mut swept_aabb = aabb;
    swept_aabb += sphere_shape.get_aabb(&predicted_xform);
    tri_mesh.process_all_triangles(&mut swept_callback, &swept_aabb);

    let contact = swept_callback.earliest?;
    let mut manifold = PersistentManifold::new(convex_obj, concave_obj);
    let normal_world = concave_obj
        .get_world_trans()
        .transform_vector3a(contact.normal);
    let point_world = concave_obj
        .get_world_trans()
        .transform_point3a(contact.point);
    let movement_world = convex_obj.interp_world_trans.translation - xform1.translation;
    if !is_entering_soccar_goal_rim_contact(point_world, xform1.translation, movement_world) {
        return None;
    }
    // This path resolves at the tick boundary rather than substepping to TOI.
    // Keep it to front-facing, bounded advances where that approximation is
    // validated; tangential base contacts and very fast crossings stay on the
    // established discrete manifold path.
    if !can_resolve_goal_sweep(
        point_world,
        normal_world,
        movement_world,
        sphere_shape.get_radius(),
    ) {
        return None;
    }
    manifold.add_contact_point(
        convex_obj,
        concave_obj,
        normal_world,
        point_world,
        0.0,
        Some(contact.triangle_idx),
        contact_added_callback,
    );
    let current_center = xform1.translation;
    let tri_inv = concave_obj.get_world_trans().transpose();
    for point in &mut manifold.point_cache {
        let point_on_sphere = current_center - point.normal_world_on_b * sphere_shape.get_radius();
        point.pos_world_on_a = point_on_sphere;
        point.pos_world_on_b = point_on_sphere;
        point.local_point_a = xform1.inv_x_form(point_on_sphere);
        point.local_point_b = tri_inv.transform_point3a(point_on_sphere);
        point.distance_1 = 0.0;
    }
    manifold.refresh_contact_points(convex_obj, concave_obj);
    Some(manifold)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn swept_sphere_finds_triangle_crossing() {
        let triangle = TriangleShape::new([
            Vec3A::new(0.0, -2.0, -2.0),
            Vec3A::new(0.0, 2.0, -2.0),
            Vec3A::new(0.0, 0.0, 2.0),
        ]);
        let mut callback = SweptSphereTriangleCallback {
            start: Vec3A::new(2.0, 0.0, 0.0),
            delta: Vec3A::new(-3.0, 0.0, 0.0),
            radius: 0.5,
            earliest: None,
        };

        callback.process_triangle(&triangle, 7);

        let contact = callback.earliest.unwrap();
        assert!((contact.toi - 0.5).abs() < 1e-5);
        assert_eq!(contact.triangle_idx, 7);
        assert!((contact.normal - Vec3A::X).length() < 1e-5);
    }

    #[test]
    fn goal_rim_sweep_only_accepts_entering_front_contacts() {
        let uu_to_bt = crate::consts::UU_TO_BT;
        let post = Vec3A::new(892.0, 5124.0, 300.0) * uu_to_bt;
        let center = Vec3A::new(880.0, 5025.0, 300.0) * uu_to_bt;

        assert!(is_entering_soccar_goal_rim_contact(post, center, Vec3A::Y));
        assert!(!is_entering_soccar_goal_rim_contact(
            post,
            center,
            -Vec3A::Y
        ));
        assert!(!is_entering_soccar_goal_rim_contact(
            Vec3A::new(1400.0, 5124.0, 200.0) * uu_to_bt,
            center,
            Vec3A::Y
        ));
    }

    #[test]
    fn goal_sweep_rejects_tangential_and_large_advances() {
        let radius = 2.0;

        assert!(can_resolve_goal_sweep(
            Vec3A::new(0.0, 0.0, 0.0),
            Vec3A::new(0.0, -1.0, 0.0),
            Vec3A::new(0.0, 0.5, 0.0),
            radius,
        ));
        assert!(!can_resolve_goal_sweep(
            Vec3A::new(0.0, 0.0, 0.0),
            Vec3A::new(0.8, -0.2, 0.0),
            Vec3A::new(0.0, 0.5, 0.0),
            radius,
        ));
        assert!(!can_resolve_goal_sweep(
            Vec3A::new(0.0, 0.0, 0.0),
            Vec3A::new(0.0, -1.0, 0.0),
            Vec3A::new(0.0, 0.6, 0.0),
            radius,
        ));

        let crossbar = Vec3A::new(0.0, 5124.0, 643.0) * crate::consts::UU_TO_BT;
        assert!(!can_resolve_goal_sweep(
            crossbar,
            -Vec3A::Y,
            Vec3A::new(0.0, 0.4, 0.005),
            radius,
        ));
        assert!(can_resolve_goal_sweep(
            crossbar,
            -Vec3A::Y,
            Vec3A::new(0.0, 0.4, 0.02),
            radius,
        ));

        let outside_post = Vec3A::new(995.0, 5124.0, 300.0) * crate::consts::UU_TO_BT;
        assert!(!can_resolve_goal_sweep(
            outside_post,
            -Vec3A::Y,
            Vec3A::new(0.0, 0.4, 0.0),
            radius,
        ));

        let post_base = Vec3A::new(895.0, 5100.0, 35.0) * crate::consts::UU_TO_BT;
        assert!(!can_resolve_goal_sweep(
            post_base,
            Vec3A::new(0.2, -0.75, 0.6),
            Vec3A::new(0.0, 0.4, 0.0),
            radius,
        ));
    }

    #[test]
    fn goal_sweep_skips_free_flight_away_from_goal() {
        let radius = 91.25 * crate::consts::UU_TO_BT;
        let near_post = Vec3A::new(900.0, 5010.0, 300.0) * crate::consts::UU_TO_BT;

        assert!(may_reach_soccar_goal_rim(
            near_post,
            near_post + Vec3A::new(0.0, 20.0, 0.0) * crate::consts::UU_TO_BT,
            radius,
        ));
        assert!(!may_reach_soccar_goal_rim(
            Vec3A::ZERO,
            Vec3A::new(0.0, 20.0, 0.0) * crate::consts::UU_TO_BT,
            radius,
        ));
        assert!(!may_reach_soccar_goal_rim(
            near_post,
            near_post + Vec3A::new(0.0, 30.0, 0.0) * crate::consts::UU_TO_BT,
            radius,
        ));
    }
}
