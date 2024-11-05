use cgmath::{ElementWise as _, InnerSpace as _};

use crate::{aabb::{HasAABB, AABB}, intersections::{Intersectable, Intersection, Intersections}, ray::Ray, types::{Float, Vec3}};

#[derive(Debug, Clone)]
pub struct Ellipsoid {
    pub radiuses: Vec3,
    pub aabb: AABB,
}

impl Ellipsoid {
    pub fn new(radiuses: Vec3) -> Self {
        let mut aabb = AABB::empty();
        aabb.extend(&radiuses);
        aabb.extend(&-radiuses);
        Self { radiuses, aabb }
    }
}

impl Intersectable for Ellipsoid {
    fn intersection(self: &Self, ray: &Ray) -> Option<Intersection> {
        let r = &self.radiuses;
        return match intersect_ellipsoid_coef(r, ray) {
            Intersections::None => None,
            Intersections::One(t2) => {
                Some(Intersection { t: t2, normal: -ray.position_at(t2).div_element_wise(*r).div_element_wise(*r).normalize(), inside: true })
            },
            Intersections::Two(t1, _) => {
                Some(Intersection { t: t1, normal: ray.position_at(t1).div_element_wise(*r).div_element_wise(*r).normalize(), inside: false })
            },
        };
    }

    fn all_intersections(self: &Self, ray: &Ray) -> Intersections {
        let r = &self.radiuses;
        return match intersect_ellipsoid_coef(r, ray) {
            Intersections::None => Intersections::None,
            Intersections::One(t2) => {
                Intersections::One(Intersection { t: t2, normal: -ray.position_at(t2).div_element_wise(*r).div_element_wise(*r).normalize(), inside: true })
            },
            Intersections::Two(t1, t2) => Intersections::Two(
                Intersection { t: t1, normal: ray.position_at(t1).div_element_wise(*r).div_element_wise(*r).normalize(), inside: false },
                Intersection { t: t2, normal: -ray.position_at(t2).div_element_wise(*r).div_element_wise(*r).normalize(), inside: true },
            ),
        };
    }
}

fn intersect_ellipsoid_coef(r: &Vec3, ray: &Ray) -> Intersections<Float> {
    let origin = ray.origin.div_element_wise(*r);
    let dir = ray.dir.div_element_wise(*r);

    let c = origin.dot(origin);
    let b = origin.dot(dir);
    let a = dir.dot(dir);

    // a * t^2 + 2 * b * t + c = 1
    let d = b * b - a * (c - 1.0);
    if d < 0.0 {
        return Intersections::None;
    }
    let ds = d.sqrt();
    let mut t1 = (-b + ds) / a;
    let mut t2 = (-b - ds) / a;
    if t2 < t1 {
        std::mem::swap(&mut t1, &mut t2);
    };

    if 0.0 <= t1 {
        return Intersections::Two(t1, t2);
    }
    if 0.0 <= t2 {
        return Intersections::One(t2);
    }
    Intersections::None
}

impl HasAABB for Ellipsoid {
    fn aabb(self: &Self) -> &AABB {
        &self.aabb
    }
}