use cgmath::{num_traits::zero, AbsDiffEq, ElementWise, InnerSpace, Rotation};
use rand::{rngs::ThreadRng, Rng};

use crate::{intersection_probability::IntersectionProbability, intersections::{intersect_light, Intersectable, Intersection, Intersections}, ray::Ray, scene::{LightPrimitives, Primitive}, types::{Float, Vec3, EPSILON, PI}};

pub trait RaySampler {
    fn sample(self: &Self, rng: &mut ThreadRng) -> Vec3;
    fn pdf(self: &Self, dir: Vec3) -> Float;
}

pub struct Uniform {
    normal: Vec3,
}

pub struct Cosine {
    normal: Vec3,
}

pub struct Mix<T : RaySampler, U : RaySampler> {
    lhs: T,
    rhs: U,
}

pub struct Light<'a> {
    pos: Vec3,
    lights: &'a LightPrimitives,
}

impl Uniform {
    #[allow(dead_code)]
    pub fn new(normal: Vec3) -> Self {
        Self { normal }
    }
}

impl Cosine {
    pub fn new(normal: Vec3) -> Self {
        Self { normal }
    }
}

impl<T : RaySampler, U : RaySampler> Mix<T, U> {
    pub fn new(lhs: T, rhs: U) -> Self {
        Self { lhs, rhs }
    }
}

impl<'a> Light<'a> {
    pub fn new(pos: Vec3, lights: &'a LightPrimitives) -> Self {
        Self { pos, lights }
    }
}

impl RaySampler for Uniform {
    fn sample(self: &Self, rng: &mut ThreadRng) -> Vec3 {
        let v = uniform_on_sphere(rng);
        if self.normal.dot(v) < 0.0 { -v } else { v }
    }

    fn pdf(self: &Self, dir: Vec3) -> Float {
        if self.normal.dot(dir) <= 0.0 {
            return 0.0;
        }
        return 1.0 / (2.0 * PI);
    }
}

impl RaySampler for Cosine {
    fn sample(self: &Self, rng: &mut ThreadRng) -> Vec3 {
        let v = uniform_on_sphere(rng);
        let dir = v + self.normal;
        if dir.abs_diff_eq(&zero(), EPSILON * 16.0) {
            return self.normal;
        }
        return dir.normalize();
    }

    fn pdf(self: &Self, dir: Vec3) -> Float {
        if self.normal.dot(dir) <= 0.0 {
            return 0.0;
        }
        return self.normal.dot(dir) / PI
    }
}

impl<T : RaySampler, U : RaySampler> RaySampler for Mix<T, U> {
    fn sample(self: &Self, rng: &mut ThreadRng) -> Vec3 {
        if rng.gen_bool(0.5) {
            self.lhs.sample(rng)
        } else {
            self.rhs.sample(rng)
        }
    }

    fn pdf(self: &Self, dir: Vec3) -> Float {
        (self.lhs.pdf(dir) + self.rhs.pdf(dir)) / 2.0
    }
}

impl<'a> RaySampler for Light<'a> {
    fn sample(self: &Self, rng: &mut ThreadRng) -> Vec3 {
        let index = rng.gen_range(0..self.lights.len());
        let world_pos: Vec3;

        if index < self.lights.boxes.len() {
            let light = &self.lights.boxes[index];
            let local_pos = uniform_on_box(&light.primitive.sizes, rng);
            world_pos = light.rotation.rotate_vector(local_pos) + light.position;
        } else if index < self.lights.boxes.len() + self.lights.ellipsoids.len() {
            let index = index - self.lights.boxes.len();
            let light = &self.lights.ellipsoids[index];
            let local_pos = uniform_on_sphere(rng).mul_element_wise(light.primitive.radiuses);
            world_pos = light.rotation.rotate_vector(local_pos) + light.position;
        } else {
            let index = index - self.lights.boxes.len() - self.lights.ellipsoids.len();
            let light = &self.lights.triangles[index];
            let triangle = &light.primitive;
            let mut u = rng.gen_range(0.0..=1.0);
            let mut v = rng.gen_range(0.0..=1.0);
            if u + v > 1.0 {
                u = 1.0 - u;
                v = 1.0 - v;
            }
            let local_pos = triangle.ba * u + triangle.ca * v;
            world_pos = light.rotation.rotate_vector(local_pos) + light.position;
        }
        (world_pos - self.pos).normalize()
    }

    fn pdf(self: &Self, dir: Vec3) -> Float {
        let ray = Ray { origin: self.pos + EPSILON * dir, dir };
        let mut impact: Float = 0.0;
        for light in &self.lights.boxes {
            impact += light_impact(&ray, light);
        }
        for light in &self.lights.ellipsoids {
            impact += light_impact(&ray, light);
        }
        for light in &self.lights.triangles {
            impact += light_impact(&ray, light);
        }
        impact / self.lights.len() as Float
    }
}

fn light_impact<T: IntersectionProbability + Intersectable>(ray: &Ray, light: &Primitive<T>) -> Float {
    match intersect_light(ray, light) {
        Intersections::None => 0.0,
        Intersections::One(intersection) => {
            light.primitive.intersection_probability(&intersection) * to_direction_probability(&intersection, &ray)
        }
        Intersections::Two(intersection1, intersection2) => {
            light.primitive.intersection_probability(&intersection1) * to_direction_probability(&intersection1, &ray) +
            light.primitive.intersection_probability(&intersection2) * to_direction_probability(&intersection2, &ray)
        },
    }
}

fn uniform_on_box(sizes: &Vec3, rng: &mut ThreadRng) -> Vec3 {
    let w4x = sizes.y * sizes.z;
    let w4y = sizes.x * sizes.z;
    let w4z = sizes.x * sizes.y;
    let choice = rng.gen_range(0.0..(w4x + w4y + w4z));
    let sign = (rng.gen_range(0..=1) * 2 - 1) as Float;
    let u1 = rng.gen_range(-1.0..=1.0);
    let u2 = rng.gen_range(-1.0..=1.0);
    if choice < w4x {
        Vec3 {x: sign, y: u1, z: u2 }
    } else if choice < w4x + w4y {
        Vec3 {x: u1, y: sign, z: u2 }
    } else {
        Vec3 {x: u1, y: u2, z: sign }
    }.mul_element_wise(*sizes)
}

fn uniform_on_sphere(rng: &mut ThreadRng) -> Vec3 {
    vec3_standard_rng(rng).normalize()
}

fn vec3_standard_rng(rng: &mut ThreadRng) -> Vec3 {
    let arr: [Float; 3] = rng.gen();
    return Vec3 {
        x: arr[0] * 2.0 - 1.0,
        y: arr[1] * 2.0 - 1.0,
        z: arr[2] * 2.0 - 1.0,
    };
}

fn to_direction_probability(intersection: &Intersection, ray: &Ray) -> Float {
    intersection.t * intersection.t / ray.dir.dot(intersection.normal).abs()
}
