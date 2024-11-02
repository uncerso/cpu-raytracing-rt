use cgmath::{vec3, ElementWise as _, InnerSpace as _, Rotation as _, SquareMatrix, Vector3};

use crate::{ray::Ray, scene::{LightPrimitive, Primitive, Scene, Triangle}, types::{Float, Mat3, Quat, Vec3}};

#[derive(Debug)]
pub struct Intersection {
    pub t: Float,
    pub normal: Vec3,
    pub inside: bool,
}

#[derive(Debug)]
pub enum Intersections {
    None,
    One(Intersection),
    Two(Intersection /* nearest */, Intersection /* farthest */),
}

impl Intersection {
    pub fn with_rotated_normal(self, q: Quat) -> Self {
        Self { t: self.t, normal: q.rotate_vector(self.normal).normalize(), inside: self.inside }
    }
}

enum IntersectionCoef<T> {
    None,
    One(T),
    Two(T /* nearest */, T /* farthest */),
}

pub fn intersect<'a>(ray: &Ray, scene: &'a Scene, max_dist: Float) -> Option<(Intersection, &'a Primitive)> {
    let mut res: Option<(Intersection, &Primitive)> = None;
    for primitive in &scene.primitives {
        let model_space_ray = model_space_ray(&primitive.position, &primitive.rotation, ray);
        let intersection = match &primitive.prim_type {
            crate::scene::PrimitiveType::Box(sizes) => intersect_box_nearest(sizes, &model_space_ray),
            crate::scene::PrimitiveType::Ellipsoid(radiuses) => intersect_ellipsoid_nearest(radiuses, &model_space_ray),
            crate::scene::PrimitiveType::Plane(normal) => intersect_plane(normal, &model_space_ray),
            crate::scene::PrimitiveType::Triangle(triangle) => intersect_triangle(triangle, &model_space_ray),
        };
        let Some(intersection) = intersection else { continue; };
        match &res {
            Some((nearest_intersection, _)) => {
                if intersection.t < nearest_intersection.t {
                    res = Some((intersection.with_rotated_normal(primitive.rotation), primitive));
                }
            },
            None => res = Some((intersection.with_rotated_normal(primitive.rotation), primitive)),
        }
    }
    res.and_then(|(intersection, primitive)| {
        if intersection.t * ray.dir.magnitude() <= max_dist {
            Some((intersection, primitive))
        } else {
            None
        }
    })
}

pub fn intersect_light(ray: &Ray, primitive: &LightPrimitive) -> Intersections {
    let model_space_ray = model_space_ray(&primitive.position, &primitive.rotation, ray);
    let intersections = match &primitive.prim_type {
        crate::scene::LightPrimitiveType::Box(sizes) => intersect_box_all(sizes, &model_space_ray),
        crate::scene::LightPrimitiveType::Ellipsoid(radiuses) => intersect_ellipsoid_all(radiuses, &model_space_ray),
        crate::scene::LightPrimitiveType::Triangle(triangle) => intersect_triangle_all(triangle, &model_space_ray),
    };

    match intersections {
        Intersections::None => Intersections::None ,
        Intersections::One(intersection) => {
            Intersections::One(intersection.with_rotated_normal(primitive.rotation))
        },
        Intersections::Two(intersection1, intersection2) => {
            Intersections::Two(
                intersection1.with_rotated_normal(primitive.rotation),
                intersection2.with_rotated_normal(primitive.rotation),
            )
        },
    }
}

struct BoxPlaneIntersection {
    t: Float,
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

pub fn intersect_box_nearest(s: &Vec3, ray: &Ray) -> Option<Intersection> {
    return match intersect_box_coef(s, ray) {
        IntersectionCoef::None => None,
        IntersectionCoef::One(t2) => {
            Some(Intersection { t: t2.t, normal: t2.real_normal(), inside: true })
        },
        IntersectionCoef::Two(t1, _) => {
            Some(Intersection { t: t1.t, normal: t1.real_normal(), inside: false })
        },
    };
}

pub fn intersect_box_all(s: &Vec3, ray: &Ray) -> Intersections {
    return match intersect_box_coef(s, ray) {
        IntersectionCoef::None => Intersections::None,
        IntersectionCoef::One(t2) => {
            Intersections::One(Intersection { t: t2.t, normal: t2.real_normal(), inside: true })
        },
        IntersectionCoef::Two(t1, t2) => Intersections::Two(
            Intersection { t: t1.t, normal: t1.real_normal(), inside: false },
            Intersection { t: t2.t, normal: t2.real_normal(), inside: true },
        ),
    };
}

fn intersect_box_coef(s: &Vec3, ray: &Ray) -> IntersectionCoef<BoxPlaneIntersection> {
    let mut t: Option<(BoxPlaneIntersection, BoxPlaneIntersection)> = None;
    for i in 0..3 {
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

    let Some((min_intersection, max_intersection)) = t else { return IntersectionCoef::None; };
    if max_intersection.t < min_intersection.t {
        return IntersectionCoef::None;
    }

    if 0.0 <= min_intersection.t {
        return IntersectionCoef::Two(min_intersection, max_intersection);
    }
    if 0.0 <= max_intersection.t {
        return IntersectionCoef::One(max_intersection);
    }
    IntersectionCoef::None
}

fn box_planes_intersect(s: &Vec3, ray: &Ray, index: usize) -> Option<(Float, Float, Float)> {
    if ray.dir[index] == 0.0 {
        return None;
    }
    let t1 = (s[index] - ray.origin[index]) / ray.dir[index];
    let t2 = (-s[index] - ray.origin[index]) / ray.dir[index];
    Some(if t1 < t2 { (t1, t2, 1.0) } else { (t2, t1, -1.0) } )
}

pub fn intersect_ellipsoid_nearest(r: &Vec3, ray: &Ray) -> Option<Intersection> {
    return match intersect_ellipsoid_coef(r, ray) {
        IntersectionCoef::None => None,
        IntersectionCoef::One(t2) => {
            Some(Intersection { t: t2, normal: -ray.position_at(t2).div_element_wise(*r).div_element_wise(*r).normalize(), inside: true })
        },
        IntersectionCoef::Two(t1, _) => {
            Some(Intersection { t: t1, normal: ray.position_at(t1).div_element_wise(*r).div_element_wise(*r).normalize(), inside: false })
        },
    };
}

fn intersect_ellipsoid_all(r: &Vec3, ray: &Ray) -> Intersections {
    return match intersect_ellipsoid_coef(r, ray) {
        IntersectionCoef::None => Intersections::None,
        IntersectionCoef::One(t2) => {
            Intersections::One(Intersection { t: t2, normal: -ray.position_at(t2).div_element_wise(*r).div_element_wise(*r).normalize(), inside: true })
        },
        IntersectionCoef::Two(t1, t2) => Intersections::Two(
            Intersection { t: t1, normal: ray.position_at(t1).div_element_wise(*r).div_element_wise(*r).normalize(), inside: false },
            Intersection { t: t2, normal: -ray.position_at(t2).div_element_wise(*r).div_element_wise(*r).normalize(), inside: true },
        ),
    };
}

fn intersect_ellipsoid_coef(r: &Vec3, ray: &Ray) -> IntersectionCoef<Float> {
    let origin = ray.origin.div_element_wise(*r);
    let dir = ray.dir.div_element_wise(*r);

    let c = origin.dot(origin);
    let b = origin.dot(dir);
    let a = dir.dot(dir);

    // a * t^2 + 2 * b * t + c = 1
    let d = b * b - a * (c - 1.0);
    if d < 0.0 {
        return IntersectionCoef::None;
    }
    let ds = d.sqrt();
    let mut t1 = (-b + ds) / a;
    let mut t2 = (-b - ds) / a;
    if t2 < t1 {
        std::mem::swap(&mut t1, &mut t2);
    };

    if 0.0 <= t1 {
        return IntersectionCoef::Two(t1, t2);
    }
    if 0.0 <= t2 {
        return IntersectionCoef::One(t2);
    }
    IntersectionCoef::None
}

pub fn intersect_plane(normal: &Vec3, ray: &Ray) -> Option<Intersection> {
    let nd = normal.dot(ray.dir);
    let t = -normal.dot(ray.origin) / nd;
    if t < 0.0 { None } else {
        Some(Intersection {
            t,
            normal: if nd <= 0.0 { 1.0 } else { -1.0 } * normal,
            inside: false
        })
    }
}

pub fn intersect_triangle(triangle: &Triangle, ray: &Ray) -> Option<Intersection> {
    let Some(matrix) = Mat3::from_cols(triangle.ba, triangle.ca, -ray.dir).invert() else { return None; };
    let Vector3 { x: u, y: v, z: t } = matrix * (ray.origin - triangle.a);
    if u < 0.0 || v < 0.0 || 1.0 < u + v || t < 0.0 {
        return None;
    }
    let normal = &triangle.normal;
    let inside = ray.dir.dot(*normal) > 0.0;
    return Some(Intersection {
        t,
        normal: if inside { -*normal } else { *normal },
        inside,
    });
}

fn intersect_triangle_all(triangle: &Triangle, ray: &Ray) -> Intersections {
    return match intersect_triangle(triangle, ray) {
        None => Intersections::None,
        Some(intersection) => Intersections::One(intersection),
    };
}

fn model_space_ray(position: &Vec3, rotation: &Quat, ray: &Ray) -> Ray {
    let rot = rotation.conjugate();
    Ray {
        origin: rot.rotate_vector(ray.origin - position),
        dir: rot.rotate_vector(ray.dir)
    }
}
