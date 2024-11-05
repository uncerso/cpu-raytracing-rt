use cgmath::{InnerSpace as _, SquareMatrix as _};

use crate::{aabb::{HasAABB, AABB}, intersections::{Intersectable, Intersection, Intersections}, ray::Ray, types::{Float, Mat3, Vec3}};

#[derive(Debug, Clone)]
pub struct Triangle {
    pub a: Vec3,
    pub ba: Vec3,
    pub ca: Vec3,
    pub normal: Vec3,
    pub inverted_area: Float,
    pub aabb: AABB,
}

impl Triangle {
    pub fn new(a: Vec3, b: Vec3, c: Vec3) -> Self {
        let ba = b - a;
        let ca = c - a;
        let sized_normal = ba.cross(ca);
        let area = sized_normal.dot(sized_normal).sqrt();
        let mut aabb = AABB::empty();
        aabb.extend(&a);
        aabb.extend(&b);
        aabb.extend(&c);
        Self { a, ba, ca, normal: sized_normal.normalize(), inverted_area: 1.0 / area, aabb }
    }
}

impl Intersectable for Triangle {
    fn intersection(self: &Self, ray: &Ray) -> Option<Intersection> {
        let Some(matrix) = Mat3::from_cols(self.ba, self.ca, -ray.dir).invert() else { return None; };
        let Vec3 { x: u, y: v, z: t } = matrix * (ray.origin - self.a);
        if u < 0.0 || v < 0.0 || 1.0 < u + v || t < 0.0 {
            return None;
        }
        let normal = &self.normal;
        let inside = ray.dir.dot(*normal) > 0.0;
        return Some(Intersection {
            t,
            normal: if inside { -*normal } else { *normal },
            inside,
        });
    }

    fn all_intersections(self: &Self, ray: &Ray) -> Intersections {
        return match self.intersection(ray) {
            None => Intersections::None,
            Some(intersection) => Intersections::One(intersection),
        };
    }
}

impl HasAABB for Triangle {
    fn aabb(self: &Self) -> &AABB {
        &self.aabb
    }
}
