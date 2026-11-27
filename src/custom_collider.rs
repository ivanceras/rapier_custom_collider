/// The implementation here is copied from trait implementation of Ball
/// The only difference made is in `shape_type, `as_typed_shape` and `scaled` which is not called
/// in the demo
///
///
use bevy_rapier3d::prelude::*;
use parry3d::math::{Pose, Vector, ComplexField, RealField};
use parry3d::bounding_volume::{Aabb, BoundingSphere};
use parry3d::mass_properties::MassProperties;
use parry3d::query::point::{PointProjection, PointQuery};
use parry3d::query::{Ray, RayCast, RayIntersection};
use parry3d::shape::{Ball, FeatureId, Shape, ShapeType, SubShapeId, TypedShape, SharedShape, SupportMap};
use parry3d::shape::ConvexPolyhedron;
use either::Either;
use num::Zero;



#[derive(Clone, Copy, PartialEq)]
pub struct CustomSphere{
    radius: f32,
}

impl CustomSphere{
    pub fn new(radius: f32) -> Self {
        Self { radius }
    }

    pub fn create_collider(&self) -> Collider {
        Collider::from(SharedShape::new(self.clone()))
    }

    pub fn scaled(
        self,
        scale: Vector,
        nsubdivs: u32,
    ) -> Option<Either<Self, ConvexPolyhedron>> {
        todo!();
        //if scale.x != scale.y || scale.x != scale.z || scale.y != scale.z {
        //    // The scaled shape isn't a ball.
        //    let (mut vtx, idx) = self.to_trimesh(nsubdivs, nsubdivs);
        //    vtx.iter_mut().for_each(|pt| *pt *= scale);
        //    Some(either::Either::Right(
        //        super::ConvexPolyhedron::from_convex_mesh(vtx, &idx)?,
        //    ))
        //} else {
        //    let uniform_scale = scale.x;
        //    Some(either::Either::Left(Self::new(
        //        self.radius * uniform_scale.abs(),
        //    )))
        //}
    }
    /// Computes the world-space [`Aabb`] of this ball transformed by `pos`.
    #[inline]
    pub fn aabb(&self, pos: &Pose) -> Aabb {
        ball_aabb(pos.translation, self.radius)
    }

    /// Computes the local-space [`Aabb`] of this ball.
    #[inline]
    pub fn local_aabb(&self) -> Aabb {
        local_ball_aabb(self.radius)
    }
    /// Computes the world-space bounding sphere of this ball, transformed by `pos`.
    #[inline]
    pub fn bounding_sphere(&self, pos: &Pose) -> BoundingSphere {
        let bv: BoundingSphere = self.local_bounding_sphere();
        bv.transform_by(pos)
    }

    /// Computes the local-space Aabb of this ball.
    #[inline]
    pub fn local_bounding_sphere(&self) -> BoundingSphere {
        BoundingSphere::new(Vector::ZERO, self.radius)
    }
}

/// Computes the Axis-Aligned Bounding Box of a ball transformed by `center`.
#[inline]
pub fn ball_aabb(center: Vector, radius: Real) -> Aabb {
    Aabb::new(
        center + Vector::splat(-radius),
        center + Vector::splat(radius),
    )
}

/// Computes the Axis-Aligned Bounding Box of a ball.
#[inline]
pub fn local_ball_aabb(radius: Real) -> Aabb {
    let half_extents = Vector::splat(radius);

    Aabb::new(-half_extents, half_extents)
}

impl RayCast for CustomSphere {
    #[inline]
    fn cast_local_ray(&self, ray: &Ray, max_time_of_impact: Real, solid: bool) -> Option<Real> {
        ray_toi_with_ball(Vector::ZERO, self.radius, ray, solid)
            .1
            .filter(|time_of_impact| *time_of_impact <= max_time_of_impact)
    }

    #[inline]
    fn cast_local_ray_and_get_normal(
        &self,
        ray: &Ray,
        max_time_of_impact: Real,
        solid: bool,
    ) -> Option<RayIntersection> {
        ray_toi_and_normal_with_ball(Vector::ZERO, self.radius, ray, solid)
            .1
            .filter(|int| int.time_of_impact <= max_time_of_impact)
    }
}

/// Computes the time of impact of a ray on a ball.
///
/// The first result element is `true` if the ray started inside of the ball.
#[inline]
pub fn ray_toi_with_ball(
    center: Vector,
    radius: Real,
    ray: &Ray,
    solid: bool,
) -> (bool, Option<Real>) {
    let dcenter = ray.origin - center;

    let a = ray.dir.length_squared();
    let b = dcenter.dot(ray.dir);
    let c = dcenter.length_squared() - radius * radius;

    // Special case for when the dir is zero.
    if a.is_zero() {
        if c > 0.0 {
            return (false, None);
        } else {
            return (true, Some(0.0));
        }
    }

    if c > 0.0 && b > 0.0 {
        (false, None)
    } else {
        let delta = b * b - a * c;

        if delta < 0.0 {
            // no solution
            (false, None)
        } else {
            let t = (-b - <Real as ComplexField>::sqrt(delta)) / a;

            if t <= 0.0 {
                // origin inside of the ball
                if solid {
                    (true, Some(0.0))
                } else {
                    (true, Some((-b + delta.sqrt()) / a))
                }
            } else {
                (false, Some(t))
            }
        }
    }
}

/// Computes the time of impact and contact normal of a ray on a ball.
#[inline]
pub fn ray_toi_and_normal_with_ball(
    center: Vector,
    radius: Real,
    ray: &Ray,
    solid: bool,
) -> (bool, Option<RayIntersection>) {
    let (inside, inter) = ray_toi_with_ball(center, radius, ray, solid);

    (
        inside,
        inter.map(|n| {
            let pos = ray.origin + ray.dir * n - center;
            let normal = pos.normalize();

            RayIntersection::new(n, if inside { -normal } else { normal }, FeatureId::Face(0))
        }),
    )
}

impl PointQuery for CustomSphere {
    #[inline]
    fn project_local_point(&self, pt: Vector, solid: bool) -> PointProjection {
        let distance_squared = pt.length_squared();

        let inside = distance_squared <= self.radius * self.radius;

        if inside && solid {
            PointProjection::new(true, pt)
        } else {
            let proj = pt * (self.radius / <Real as ComplexField>::sqrt(distance_squared));
            PointProjection::new(inside, proj)
        }
    }

    #[inline]
    fn project_local_point_and_get_feature(&self, pt: Vector) -> (PointProjection, FeatureId) {
        (self.project_local_point(pt, false), FeatureId::Face(0))
    }

    #[inline]
    fn distance_to_local_point(&self, pt: Vector, solid: bool) -> Real {
        let dist = pt.length() - self.radius;

        if solid && dist < 0.0 {
            0.0
        } else {
            dist
        }
    }

    #[inline]
    fn contains_local_point(&self, pt: Vector) -> bool {
        pt.length_squared() <= self.radius * self.radius
    }
}

impl Shape for CustomSphere {
    fn clone_dyn(&self) -> Box<dyn Shape> {
        Box::new(*self)
    }

    fn scale_dyn(&self, scale: Vector, num_subdivisions: u32) -> Option<Box<dyn Shape>> {
        let scaled = self.scaled(scale, num_subdivisions)?;
        Some(scaled.either::<_, _, Box<dyn Shape>>(|x| Box::new(x), |x| Box::new(x)))
    }

    fn compute_local_aabb(&self) -> Aabb {
        self.local_aabb()
    }

    fn compute_local_bounding_sphere(&self) -> BoundingSphere {
        self.local_bounding_sphere()
    }

    fn compute_aabb(&self, position: &Pose) -> Aabb {
        self.aabb(position)
    }

    fn mass_properties(&self, density: Real) -> MassProperties {
        MassProperties::from_ball(density, self.radius)
    }

    fn ccd_thickness(&self) -> Real {
        self.radius
    }

    fn ccd_angular_thickness(&self) -> Real {
        Real::pi()
    }

    fn is_convex(&self) -> bool {
        true
    }

    fn shape_type(&self) -> ShapeType {
        ShapeType::Custom
    }

    fn as_typed_shape(&self) -> TypedShape<'_> {
        TypedShape::Custom(self)
    }

    fn as_support_map(&self) -> Option<&dyn SupportMap> {
        Some(self as &dyn SupportMap)
    }

    /// The shape's normal at the given point located on a specific feature.
    #[inline]
    fn feature_normal_at_point(
        &self,
        _: SubShapeId,
        _: FeatureId,
        point: Vector,
    ) -> Option<Vector> {
        (point).try_normalize()
    }
}

impl SupportMap for CustomSphere {
    #[inline]
    fn support_point(&self, m: &Pose, dir: Vector) -> Vector {
        self.support_point_toward(m, dir.normalize())
    }

    #[inline]
    fn support_point_toward(&self, m: &Pose, dir: Vector) -> Vector {
        m.translation + dir * self.radius
    }

    #[inline]
    fn local_support_point(&self, dir: Vector) -> Vector {
        self.local_support_point_toward(dir.normalize())
    }

    #[inline]
    fn local_support_point_toward(&self, dir: Vector) -> Vector {
        dir * self.radius
    }
}
