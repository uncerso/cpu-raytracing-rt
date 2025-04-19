use cgmath::InnerSpace as _;

use crate::{intersections::{Intersectable, Intersection, Intersections}, ray::Ray, types::Vec3};

#[derive(Debug, Clone)]
pub struct Plane {
    pub normal: Vec3,
}

impl Intersectable for Plane {
    fn intersection(self: &Self, ray: &Ray) -> Option<Intersection> {
        let nd = self.normal.dot(ray.dir);
        let t = -self.normal.dot(ray.origin) / nd;
        if t < 0.0 { None } else {
            Some(Intersection::with_geometry_normals(
                t,
                if nd <= 0.0 { 1.0 } else { -1.0 } * self.normal,
                false,
            ))
        }
    }

    fn all_intersections(self: &Self, ray: &Ray) -> Intersections {
        return match self.intersection(ray) {
            None => Intersections::None,
            Some(intersection) => Intersections::One(intersection),
        };
    }
}
