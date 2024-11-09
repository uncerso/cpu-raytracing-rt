use crate::{aabb::AABB, intersections::Intersections, primitives::r#box::intersect_box_coef, ray::Ray, types::{Float, Vec3}};

#[derive(Debug)]
pub struct IntersectableAABB {
    sizes: Vec3,
    position: Vec3,
}

impl IntersectableAABB {
    pub fn new(aabb: &AABB) -> Self {
        Self {
            sizes: (aabb.max - aabb.min) / 2.0,
            position: (aabb.max + aabb.min) / 2.0,
        }
    }

    pub fn intersects(&self, ray: &Ray) -> Option<Float> {
        match intersect_box_coef(&self.sizes, &Ray { origin: ray.origin - self.position, dir: ray.dir }) {
            Intersections::None => None,
            Intersections::One(t) => Some(t.t),
            Intersections::Two(t, _) => Some(t.t),
        }
    }
}
