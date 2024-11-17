use cgmath::{InnerSpace as _, Rotation as _};

use crate::{intersection_probability::IntersectionProbability, ray::Ray, scene::{LightPrimitives, Metadata, Primitive, ScenePrimitives, TrianglePrimitive}, types::{Float, Quat, Vec3}};

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

    for primitive in scene.planes.iter() {
        let intersection = primitive.intersection(&ray);
        let Some(intersection) = intersection else { continue; };
        update_best_intersection_with(intersection, &primitive.metadata, &primitive.rotation, &mut res);
    }

    update_best_intersection(scene.boxes.intersection(ray), &mut res);
    update_best_intersection(scene.ellipsoids.intersection(ray), &mut res);
    update_best_intersection_by_triangle(scene.triangles.intersection(ray), &mut res);

    res.and_then(|(intersection, metadata, rotation)| {
        if intersection.t * ray.dir.magnitude() <= max_dist {
            Some((intersection.with_rotated_normal(*rotation), metadata))
        } else {
            None
        }
    })
}

const DONT_ROTATE: Quat = Quat::new(1.0, 0.0, 0.0, 0.0);

fn update_best_intersection_by_triangle<'a>(intersection: Option<(Intersection, &'a TrianglePrimitive)>, best_result: &mut Option<(Intersection, &'a Metadata, &'a Quat)>) {
    let Some((intersection, primitive)) = intersection else { return; };
    update_best_intersection_with(intersection, &primitive.metadata, &DONT_ROTATE, best_result);
}

fn update_best_intersection<'a, T: Intersectable>(intersection: Option<(Intersection, &'a Primitive<T>)>, best_result: &mut Option<(Intersection, &'a Metadata, &'a Quat)>) {
    let Some((intersection, primitive)) = intersection else { return; };
    update_best_intersection_with(intersection, &primitive.metadata, &primitive.rotation, best_result);
}

fn update_best_intersection_with<'a>(intersection: Intersection, metadata: &'a Metadata, rotation: &'a Quat, best_result: &mut Option<(Intersection, &'a Metadata, &'a Quat)>) {
    match best_result {
        Some((nearest_intersection, _, _)) => {
            if intersection.t < nearest_intersection.t {
                *best_result = Some((intersection, &metadata, &rotation))
            }
        },
        None => *best_result = Some((intersection, &metadata, &rotation)),
    }
}

pub fn intersect_lights(ray: &Ray, lights: &LightPrimitives, callback: &mut impl FnMut(Intersection, &dyn IntersectionProbability)) {
    lights.boxes.intersections(ray, &mut |intersection, primitive| callback(intersection.with_rotated_normal(primitive.rotation), &primitive.primitive));
    lights.ellipsoids.intersections(ray, &mut |intersection, primitive| callback(intersection.with_rotated_normal(primitive.rotation), &primitive.primitive));
    lights.triangles.intersections(ray, &mut |intersection, primitive| callback(intersection, &primitive.primitive));
}

fn model_space_ray(position: &Vec3, rotation: &Quat, ray: &Ray) -> Ray {
    let rot = rotation.conjugate();
    Ray {
        origin: rot.rotate_vector(ray.origin - position),
        dir: rot.rotate_vector(ray.dir)
    }
}

impl<T: Intersectable> Intersectable for Primitive<T> {
    fn intersection(self: &Self, ray: &Ray) -> Option<Intersection> {
        self.primitive.intersection(&model_space_ray(&self.position, &self.rotation, ray))
    }

    fn all_intersections(self: &Self, ray: &Ray) -> Intersections {
        self.primitive.all_intersections(&model_space_ray(&self.position, &self.rotation, ray))
    }
}

impl Intersectable for TrianglePrimitive {
    fn intersection(self: &Self, ray: &Ray) -> Option<Intersection> {
        self.primitive.intersection(ray)
    }

    fn all_intersections(self: &Self, ray: &Ray) -> Intersections {
        self.primitive.all_intersections(ray)
    }
}
