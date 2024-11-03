use cgmath::{InnerSpace as _, Rotation as _};

use crate::{ray::Ray, scene::{LightPrimitive, LightPrimitiveType, Primitive, PrimitiveType, Scene}, types::{Float, Quat, Vec3}};

pub trait Intersectable {
    fn intersection(self: &Self, ray: &Ray) -> Option<Intersection>;
    fn all_intersections(self: &Self, ray: &Ray) -> Intersections;
}

#[derive(Debug)]
pub struct Intersection {
    pub t: Float,
    pub normal: Vec3,
    pub inside: bool,
}

#[derive(Debug)]
pub enum Intersections<T = Intersection> {
    None,
    One(T),
    Two(T /* nearest */, T /* farthest */),
}

impl Intersection {
    pub fn with_rotated_normal(self, q: Quat) -> Self {
        Self { t: self.t, normal: q.rotate_vector(self.normal).normalize(), inside: self.inside }
    }
}

pub fn intersect<'a>(ray: &Ray, scene: &'a Scene, max_dist: Float) -> Option<(Intersection, &'a Primitive)> {
    let mut res: Option<(Intersection, &Primitive)> = None;
    for primitive in &scene.primitives {
        let model_space_ray = model_space_ray(&primitive.position, &primitive.rotation, ray);
        let intersection = match &primitive.prim_type {
            PrimitiveType::Box(r#box) => r#box.intersection(&model_space_ray),
            PrimitiveType::Ellipsoid(ellipsoid) => ellipsoid.intersection(&model_space_ray),
            PrimitiveType::Plane(plane) => plane.intersection(&model_space_ray),
            PrimitiveType::Triangle(triangle) => triangle.intersection(&model_space_ray),
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
        LightPrimitiveType::Box(r#box) => r#box.all_intersections(&model_space_ray),
        LightPrimitiveType::Ellipsoid(ellipsoid) => ellipsoid.all_intersections(&model_space_ray),
        LightPrimitiveType::Triangle(triangle) => triangle.all_intersections(&model_space_ray),
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

fn model_space_ray(position: &Vec3, rotation: &Quat, ray: &Ray) -> Ray {
    let rot = rotation.conjugate();
    Ray {
        origin: rot.rotate_vector(ray.origin - position),
        dir: rot.rotate_vector(ray.dir)
    }
}
