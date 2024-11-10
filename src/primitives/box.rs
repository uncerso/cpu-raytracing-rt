use cgmath::vec3;

use crate::{aabb::{HasAABB, AABB}, intersections::{Intersectable, Intersection, Intersections}, ray::Ray, types::{Float, Vec3}};

#[derive(Debug, Clone)]
pub struct Box {
    pub sizes: Vec3,
    pub aabb: AABB,
}

impl Box {
    pub fn new(sizes: Vec3) -> Self {
        let mut aabb = AABB::empty();
        aabb.extend(&sizes);
        aabb.extend(&-sizes);
        Self { sizes, aabb }
    }
}

impl Intersectable for Box {
    fn intersection(self: &Self, ray: &Ray) -> Option<Intersection> {
        return match intersect_box_coef(&self.sizes, ray) {
            Intersections::None => None,
            Intersections::One(t2) => {
                Some(Intersection { t: t2.t, normal: t2.real_normal(), inside: true })
            },
            Intersections::Two(t1, _) => {
                Some(Intersection { t: t1.t, normal: t1.real_normal(), inside: false })
            },
        };
    }

    fn all_intersections(self: &Self, ray: &Ray) -> Intersections {
        return match intersect_box_coef(&self.sizes, ray) {
            Intersections::None => Intersections::None,
            Intersections::One(t2) => {
                Intersections::One(Intersection { t: t2.t, normal: t2.real_normal(), inside: true })
            },
            Intersections::Two(t1, t2) => Intersections::Two(
                Intersection { t: t1.t, normal: t1.real_normal(), inside: false },
                Intersection { t: t2.t, normal: t2.real_normal(), inside: true },
            ),
        };
    }
}

#[derive(Debug)]
pub struct BoxPlaneIntersection {
    pub t: Float,
    normal: Float,
    dim_index: usize,
}

impl BoxPlaneIntersection {
    fn max(self, other: BoxPlaneIntersection) -> Self {
        if self.t < other.t { other } else { self }
    }
    fn min(self, other: BoxPlaneIntersection) -> Self {
        if self.t < other.t { self } else { other }
    }

    fn real_normal(&self) -> Vec3 {
        if self.dim_index == 0 {
            return vec3(self.normal, 0.0, 0.0);
        }
        if self.dim_index == 1 {
            return vec3(0.0, self.normal, 0.0);
        }
        vec3(0.0, 0.0, self.normal)
    }
}

pub fn intersect_box_coef(s: &Vec3, ray: &Ray) -> Intersections<BoxPlaneIntersection> {
    let mut t: Option<(BoxPlaneIntersection, BoxPlaneIntersection)> = None;
    for i in 0..3 {
        if ray.dir[i] == 0.0 && s[i] < ray.origin[i].abs() {
            return Intersections::None;
        }
        let Some((t1, t2, normal)) = box_planes_intersect(s, &ray, i) else { continue; };
        t = Some(match t {
            Some((max_t1, min_t2)) => (
                BoxPlaneIntersection{ t: t1, normal, dim_index: i }.max(max_t1),
                BoxPlaneIntersection{ t: t2, normal, dim_index: i }.min(min_t2),
            ),
            None => (
                BoxPlaneIntersection{ t: t1, normal, dim_index: i },
                BoxPlaneIntersection{ t: t2, normal, dim_index: i },
            ),
        })
    }

    let Some((min_intersection, max_intersection)) = t else { return Intersections::None; };
    if max_intersection.t < min_intersection.t {
        return Intersections::None;
    }

    if 0.0 <= min_intersection.t {
        return Intersections::Two(min_intersection, max_intersection);
    }
    if 0.0 <= max_intersection.t {
        return Intersections::One(max_intersection);
    }
    Intersections::None
}

fn box_planes_intersect(s: &Vec3, ray: &Ray, index: usize) -> Option<(Float, Float, Float)> {
    if ray.dir[index] == 0.0 {
        return None;
    }
    let t1 = (s[index] - ray.origin[index]) / ray.dir[index];
    let t2 = (-s[index] - ray.origin[index]) / ray.dir[index];
    Some(if t1 < t2 { (t1, t2, 1.0) } else { (t2, t1, -1.0) } )
}

impl HasAABB for Box {
    fn aabb(self: &Self) -> &AABB {
        &self.aabb
    }
}

#[cfg(test)]
mod test {
    use cgmath::{vec3, InnerSpace};

    use crate::{intersections::{Intersectable, Intersection}, primitives::Box, ray::Ray, types::Float};

    #[test]
    fn a() {
        let aabb = Box::new(vec3(1.0, 2.0, 1.0));
        let ray = Ray { origin: vec3(0.0, 0.0, 2.0), dir: vec3(0.0, 0.0, 1.0).normalize() };
        assert!(aabb.intersection(&ray).is_none());
    }

    #[test]
    fn b() {
        let aabb = Box::new(vec3(1.0, 2.0, 1.0));
        let ray = Ray { origin: vec3(0.0, 0.0, -2.0), dir: vec3(0.0, 0.0, 1.0).normalize() };
        let expected = Intersection{ t: 1.0, normal: vec3(0.0, 0.0, -1.0), inside: false };
        equal_intersections(aabb.intersection(&ray).unwrap(), expected);
    }

    #[test]
    fn c() {
        let aabb = Box::new(vec3(1.0, 2.0, 1.0));
        let ray = Ray { origin: vec3(2.0, 0.0, -2.0), dir: vec3(0.0, 0.0, 1.0).normalize() };
        assert!(aabb.intersection(&ray).is_none());
    }

    #[test]
    fn d() {
        let aabb = Box::new(vec3(1.0, 2.0, 1.0));
        let ray = Ray { origin: vec3(-2.0, 0.0, -2.0), dir: vec3(1.0, 0.0, 1.0).normalize() };
        let expected = Intersection{ t: (2.0 as Float).sqrt(), normal: vec3(0.0, 0.0, -1.0), inside: false };
        equal_intersections(aabb.intersection(&ray).unwrap(), expected);
    }

    #[test]
    fn e() {
        let aabb = Box::new(vec3(1.0, 2.0, 1.0));
        let ray = Ray { origin: vec3(-1.0, 0.0, -2.0), dir: vec3(0.0, 0.0, 1.0).normalize() };
        let expected = Intersection{ t: 1.0, normal: vec3(0.0, 0.0, -1.0), inside: false };
        equal_intersections(aabb.intersection(&ray).unwrap(), expected);
    }

    fn equal_intersections(a: Intersection, b: Intersection) {
        assert_eq!(a.t, b.t);
        assert_eq!(a.normal, b.normal);
        assert_eq!(a.inside, b.inside);
    }
}
