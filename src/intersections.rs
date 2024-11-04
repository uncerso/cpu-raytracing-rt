use cgmath::{InnerSpace as _, Rotation as _};

use crate::{ray::Ray, scene::{Metadata, Primitive, ScenePrimitives}, types::{Float, Quat, Vec3}};

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

pub fn intersect<'a>(ray: &Ray, scene: &'a ScenePrimitives, max_dist: Float) -> Option<(Intersection, &'a Metadata)> {
    let mut res: Option<(Intersection, &Metadata, &Quat)> = None;

    fn intersect<'a, T: Intersectable>(primitive: &'a Primitive<T>, ray: &Ray, res: &mut Option<(Intersection, &'a Metadata, &'a Quat)>) {
        let model_space_ray = model_space_ray(&primitive.position, &primitive.rotation, ray);
        let intersection = primitive.primitive.intersection(&model_space_ray);
        let Some(intersection) = intersection else { return; };
        match res {
            Some((nearest_intersection, _, _)) => {
                if intersection.t < nearest_intersection.t {
                    *res = Some((intersection, &primitive.metadata, &primitive.rotation));
                }
            },
            None => *res = Some((intersection, &primitive.metadata, &primitive.rotation)),
        };
    }

    for primitive in scene.boxes.iter() {
        intersect(primitive, ray, &mut res);
    }

    for primitive in scene.ellipsoids.iter() {
        intersect(primitive, ray, &mut res);
    }

    for primitive in scene.triangles.iter() {
        intersect(primitive, ray, &mut res);
    }

    for primitive in scene.planes.iter() {
        intersect(primitive, ray, &mut res);
    }

    res.and_then(|(intersection, metadata, rotation)| {
        if intersection.t * ray.dir.magnitude() <= max_dist {
            Some((intersection.with_rotated_normal(*rotation), metadata))
        } else {
            None
        }
    })
}

pub fn intersect_light<T: Intersectable>(ray: &Ray, lights: &Primitive<T>) -> Intersections {
    let model_space_ray = model_space_ray(&lights.position, &lights.rotation, ray);
    let intersections = lights.primitive.all_intersections(&model_space_ray);

    match intersections {
        Intersections::None => Intersections::None ,
        Intersections::One(intersection) => {
            Intersections::One(intersection.with_rotated_normal(lights.rotation))
        },
        Intersections::Two(intersection1, intersection2) => {
            Intersections::Two(
                intersection1.with_rotated_normal(lights.rotation),
                intersection2.with_rotated_normal(lights.rotation),
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
