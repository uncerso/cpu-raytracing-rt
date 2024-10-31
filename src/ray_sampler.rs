use cgmath::{num_traits::zero, AbsDiffEq, Array, ElementWise, InnerSpace, Rotation};
use rand::{rngs::ThreadRng, Rng};

use crate::{intersections::{intersect_light, Intersection}, ray::Ray, scene::{LightPrimitive, LightPrimitiveType::{Box, Ellipsoid}}, types::{Float, Vec3, EPSILON, PI}};

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
    lights: &'a [LightPrimitive],
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
    pub fn new(pos: Vec3, lights: &'a [LightPrimitive]) -> Self {
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
        let light = &self.lights[index];
        let local_pos = match light.prim_type {
            Box(sizes) => {
                uniform_on_box(&sizes, rng)
            },
            Ellipsoid(radiuses) => {
                uniform_on_sphere(rng).mul_element_wise(radiuses)
            },
        };
        let world_pos = light.rotation.rotate_vector(local_pos) + light.position;
        (world_pos - self.pos).normalize()
    }

    fn pdf(self: &Self, dir: Vec3) -> Float {
        let ray = Ray { origin: self.pos + EPSILON * dir, dir };
        let mut impact: Float = 0.0;
        for light in self.lights {
            match intersect_light(&ray, &light) {
                crate::intersections::Intersections::None => continue,
                crate::intersections::Intersections::One(intersection) => {
                    impact += match light.prim_type {
                        Box(sizes) => impact_box(&sizes, &intersection, &ray),
                        Ellipsoid(radiuses) => impact_ellipsoid(&radiuses, &intersection, &ray),
                    };
                },
                crate::intersections::Intersections::Two(intersection1, intersection2) => {
                    impact += match light.prim_type {
                        Box(sizes) => {
                            impact_box(&sizes, &intersection1, &ray) + impact_box(&sizes, &intersection2, &ray)
                        },
                        Ellipsoid(radiuses) => {
                            impact_ellipsoid(&radiuses, &intersection1, &ray) + impact_ellipsoid(&radiuses, &intersection2, &ray)
                        },
                    };
                },
            }
        }
        impact / self.lights.len() as Float
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

fn impact_ellipsoid(radiuses: &Vec3, intersection: &Intersection, ray: &Ray) -> Float {
    let coef = Vec3 {
        x: radiuses.y * radiuses.z,
        y: radiuses.x * radiuses.z,
        z: radiuses.x * radiuses.y,
    }.mul_element_wise(intersection.normal);

    let p = 1.0 / (4.0 * PI * coef.dot(coef).sqrt());
    p * to_direction_probability(intersection, ray)
}

fn impact_box(sizes: &Vec3, intersection: &Intersection, ray: &Ray) -> Float {
    let p = 1.0 / Vec3 {
        x: sizes.y * sizes.z,
        y: sizes.x * sizes.z,
        z: sizes.x * sizes.y,
    }.sum() / 8.0;

    p * to_direction_probability(intersection, ray)
}

fn to_direction_probability(intersection: &Intersection, ray: &Ray) -> Float {
    intersection.t * intersection.t / ray.dir.dot(intersection.normal).abs()
}