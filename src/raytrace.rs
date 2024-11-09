use cgmath::{num_traits::{zero, Pow}, ElementWise, InnerSpace};
use rand::{rngs::ThreadRng, Rng};

use crate::{image::RGB, intersections::{intersect, Intersection}, ray::Ray, ray_sampler::{Cosine, Light, Mix, RaySampler}, scene::{Material, Scene}, types::{Float, EPSILON, PI}};

const AIR_IOR: Float = 1.0;

pub fn raytrace(ray: &Ray, scene: &Scene, rng: &mut ThreadRng) -> RGB {
    raytrace_impl(&Ray { origin: ray.origin, dir: ray.dir.normalize() }, scene, rng, scene.ray_depth)
}

fn raytrace_impl(ray: &Ray, scene: &Scene, rng: &mut ThreadRng, left_ray_depth: u8) -> RGB {
    if left_ray_depth == 0 { return zero(); }
    let Some((intersection, metadata)) = intersect(ray, &scene.primitives, Float::INFINITY) else { return scene.bg_color; };
    return metadata.emission + match metadata.material {
        Material::Diffuse => {
            let intersection_pos = ray.position_at(intersection.t);
            let cosine_sampler = Cosine::new(intersection.normal);
            let mix_sampler = Mix::new(Cosine::new(intersection.normal), Light::new(intersection_pos, &scene.lights));
            let mut ray_sampler: &dyn RaySampler = if scene.lights.is_empty() { &cosine_sampler } else { &mix_sampler };
            let mut rng_dir = ray_sampler.sample(rng);
            if rng_dir.dot(intersection.normal) <= 0.0 {
                ray_sampler = &cosine_sampler;
                rng_dir = ray_sampler.sample(rng);
            }
            let pdf = ray_sampler.pdf(rng_dir);
            if pdf == 0.0 {
                zero()
            } else {
                let light_from_dir = raytrace_impl(&Ray { origin: intersection_pos + EPSILON * rng_dir, dir: rng_dir }, scene, rng, left_ray_depth - 1);
                rng_dir.dot(intersection.normal) * metadata.color.mul_element_wise(light_from_dir) / PI / pdf
            }
        },

        Material::Dielectric(ior) => {
            let mut n1 = AIR_IOR;
            let mut n2 = ior;
            if intersection.inside { std::mem::swap(&mut n1, &mut n2) }
            let reflected_ray = &reflected_ray(ray, &intersection);

            match refracted_ray(ray, &intersection, n1 / n2) {
                None => raytrace_impl(reflected_ray, scene, rng, left_ray_depth - 1),
                Some(refracted_ray) => {
                    let reflection_power = reflection_power(n1, n2, ray, &intersection);
                    if rng.gen_bool(reflection_power.clamp(0.0, 1.0) as f64) {
                        raytrace_impl(reflected_ray, scene, rng, left_ray_depth - 1)
                    } else {
                        let refracted_color = raytrace_impl(&refracted_ray, scene, rng, left_ray_depth - 1);
                        if intersection.inside { refracted_color } else { refracted_color.mul_element_wise(metadata.color) }
                    }
                }
            }
        },

        Material::Metallic => {
            raytrace_impl(&reflected_ray(ray, &intersection), scene, rng, left_ray_depth - 1).mul_element_wise(metadata.color)
        }
    };
}

fn reflection_power(n1: Float, n2: Float, ray: &Ray, intersection: &Intersection) -> Float {
    let r0: Float = ((n1 - n2) / (n1 + n2)).pow(2);
    r0 + (1.0 - r0) * (1.0 + ray.dir.dot(intersection.normal)).pow(5)
}

fn reflected_ray(ray: &Ray, intersection: &Intersection) -> Ray {
    let dir = ray.dir - 2.0 * intersection.normal * intersection.normal.dot(ray.dir);
    Ray {
        origin: ray.position_at(intersection.t) + EPSILON * dir,
        dir,
    }
}

fn refracted_ray(ray: &Ray, intersection: &Intersection, reflection_index: Float) -> Option<Ray> {
    let cos_theta1 = -intersection.normal.dot(ray.dir);
    let sin_theta2 = reflection_index * (1.0 - cos_theta1 * cos_theta1).sqrt();
    if sin_theta2 > 1.0 {
        return None; // Total Internal Reflection
    }

    let cos_theta2 = (1.0 - sin_theta2 * sin_theta2).sqrt();
    let dir = reflection_index * ray.dir + (reflection_index * cos_theta1 - cos_theta2) * intersection.normal;
    Some(Ray {
        origin: ray.position_at(intersection.t) + EPSILON * dir,
        dir,
    })
}
