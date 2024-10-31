use cgmath::{num_traits::{zero, Pow}, vec3, ElementWise, InnerSpace};
use rand::{rngs::ThreadRng, Rng};

use crate::{image::RGB, intersections::{intersect, Intersection}, ray::Ray, ray_sampler::{Cosine, Light, Mix, RaySampler}, scene::Scene, types::{Float, EPSILON, PI}};

const AIR_IOR: Float = 1.0;

pub fn raytrace(ray: &Ray, scene: &Scene, rng: &mut ThreadRng) -> RGB {
    raytrace_impl(&Ray { origin: ray.origin, dir: ray.dir.normalize() }, scene, rng, scene.ray_depth)
}

fn raytrace_impl(ray: &Ray, scene: &Scene, rng: &mut ThreadRng, left_ray_depth: u8) -> RGB {
    if left_ray_depth == 0 { return zero(); }
    let Some((intersection, primitive)) = intersect(ray, scene, Float::INFINITY) else { return scene.bg_color; };
    return primitive.emission + match primitive.material {
        crate::scene::Material::Diffuse => {
            let intersection_pos = ray.position_at(intersection.t);
            let cosine_sampler = Cosine::new(intersection.normal);
            let mix_sampler = Mix::new(Cosine::new(intersection.normal), Light::new(intersection_pos, scene.lights.as_slice()));
            let ray_sampler: &dyn RaySampler = if scene.lights.is_empty() { &cosine_sampler } else { &mix_sampler };
            let rng_dir = ray_sampler.sample(rng);
            if rng_dir.dot(intersection.normal) <= 0.0 {
                zero()
            } else {
                let light_from_dir = raytrace_impl(&Ray { origin: intersection_pos + EPSILON * rng_dir, dir: rng_dir }, scene, rng, left_ray_depth - 1);
                rng_dir.dot(intersection.normal) * primitive.color.mul_element_wise(light_from_dir) / PI / ray_sampler.pdf(rng_dir)
            }
        },

        crate::scene::Material::Dielectric(ior) => {
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
                        let primitive_color = if intersection.inside { vec3(1.0, 1.0, 1.0) } else { primitive.color };
                        refracted_color.mul_element_wise(primitive_color)
                    }
                }
            }
        },

        crate::scene::Material::Metallic => {
            raytrace_impl(&reflected_ray(ray, &intersection), scene, rng, left_ray_depth - 1).mul_element_wise(primitive.color)
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

#[cfg(test)]
mod tests {
    use cgmath::{assert_abs_diff_eq, Deg, Rotation, Rotation3};
    use crate::{intersections::{intersect_box_nearest, intersect_ellipsoid_nearest, Intersection}, ray::Ray, types::{Quat, Vec3, EPSILON}};

    use super::*;

    #[test]
    fn a() {
        let ray = Ray {
            origin: zero(),
            dir: Vec3::unit_z(),
        };
        let intersection = Intersection { t: 10.0, normal: -Vec3::unit_z(), inside: false };

        let refracted = refracted_ray(&ray, &intersection, 1.0 / 1.04);
        assert!(refracted.is_some());
        let refracted = refracted.unwrap();
        assert_eq!(refracted.dir, Vec3::unit_z());
        assert_eq!(refracted.origin, Vec3::unit_z() * 10.0 + EPSILON * refracted.dir);
    }

    #[test]
    fn b() {
        let ray = Ray {
            origin: zero(),
            dir: Vec3::unit_z(),
        };
        let intersection = Intersection { t: 10.0, normal: -Vec3::unit_z(), inside: true };

        let refracted = refracted_ray(&ray, &intersection, 1.04 / 1.00);
        assert!(refracted.is_some());
        let refracted = refracted.unwrap();
        assert_eq!(refracted.dir, Vec3::unit_z());
        assert_eq!(refracted.origin, Vec3::unit_z() * 10.0 + EPSILON * refracted.dir);
    }

    #[test]
    fn c() {
        let ray = Ray {
            origin: zero(),
            dir: Vec3::unit_z(),
        };

        let intersection = Intersection { t: 10.0, normal: vec3(-1.0, 0.0, -1.0).normalize(), inside: false };

        let refracted = refracted_ray(&ray, &intersection, 1.0);
        assert!(refracted.is_some());
        let refracted = refracted.unwrap();
        assert_abs_diff_eq!(refracted.dir, ray.dir);
        assert_eq!(refracted.origin, ray.dir * 10.0 + EPSILON * refracted.dir);
    }

    #[test]
    fn d() {
        let ray = Ray {
            origin: zero(),
            dir: Vec3::unit_z(),
        };

        let intersection = intersect_box_nearest(&vec3(0.5, 0.5, 0.5), &ray);
        assert!(intersection.is_some());
        let intersection = intersection.unwrap();
        assert_eq!(intersection.t, 0.5);
        assert_eq!(intersection.normal, vec3(0.0, 0.0, -1.0));
        assert_eq!(intersection.inside, true);
    }

    #[test]
    fn d2() {
        let mut ray = Ray {
            origin: zero(),
            dir: Vec3::unit_z(),
        };

        let q = Quat::from_axis_angle(Vec3::unit_y(), Deg(1.0) );

        for _ in 0..360 {
            let intersection = intersect_box_nearest(&vec3(0.5, 0.5, 0.5), &ray);
            assert!(intersection.is_some());
            let intersection = intersection.unwrap();
            assert_eq!(intersection.normal.dot(vec3(1.0, 1.0, 1.0)), -ray.dir.dot(vec3(1.0, 1.0, 1.0)).signum());
            assert_eq!(intersection.inside, true);
            ray.dir = q.rotate_vector(ray.dir);
        }
    }

    #[test]
    fn d3() {
        let mut ray = Ray {
            origin: zero(),
            dir: vec3(1.0, 1.0, 1.0).normalize(),
        };

        let q = Quat::from_axis_angle(Vec3::unit_y(), Deg(1.0) );

        for _ in 0..360 {
            let intersection = intersect_box_nearest(&vec3(0.5, 0.5, 0.5), &ray);
            assert!(intersection.is_some());
            let intersection = intersection.unwrap();
            assert_eq!(intersection.normal.dot(vec3(1.0, 1.0, 1.0)), -ray.dir.dot(vec3(1.0, 0.0, 1.0)).signum());
            assert_eq!(intersection.inside, true);
            ray.dir = q.rotate_vector(ray.dir);
        }
    }

    #[test]
    fn d4() {
        let mut ray = Ray {
            origin: zero(),
            dir: vec3(1.0, 1.1, 1.0).normalize(),
        };

        let q = Quat::from_axis_angle(Vec3::unit_y(), Deg(1.0) );

        for _ in 0..360 {
            let intersection = intersect_box_nearest(&vec3(0.5, 0.5, 0.5), &ray);
            assert!(intersection.is_some());
            let intersection = intersection.unwrap();
            assert!(intersection.normal == vec3(0.0, -1.0, 0.0) || intersection.normal.dot(vec3(1.0, 1.0, 1.0)) == -ray.dir.dot(vec3(1.0, 0.0, 1.0)).signum());
            assert_eq!(intersection.inside, true);
            ray.dir = q.rotate_vector(ray.dir);
        }
    }

    #[test]
    fn ellipsoid1() {
        let ray = Ray {
            origin: vec3(0.0, 0.0, -8.0),
            dir: Vec3::unit_z(),
        };

        let intersection = intersect_ellipsoid_nearest(&vec3(4.0, 2.0, 0.5), &ray);
        assert!(intersection.is_some());
        let intersection = intersection.unwrap();
        assert_eq!(intersection.t, 7.5);
        assert_eq!(intersection.normal, vec3(0.0, 0.0, -1.0));
        assert_eq!(intersection.inside, false);
    }

    #[test]
    fn ellipsoid4() {
        let ray = Ray {
            origin: vec3(0.0, 0.0, -8.0),
            dir: vec3(0.5, 0.0, 8.0).normalize(),
        };

        let r = vec3(0.5, 2.0, 4.0);
        let intersection = intersect_ellipsoid_nearest(&r, &ray);
        assert!(intersection.is_some());
        let intersection = intersection.unwrap();

        let expected_pos = vec3( 3.0/10.0, 0.0, -16.0/5.0);
        assert_abs_diff_eq!(intersection.t, (expected_pos-ray.origin).magnitude());
        assert_abs_diff_eq!(intersection.normal, expected_pos.div_element_wise(r).div_element_wise(r).normalize());
        assert_eq!(intersection.inside, false);
    }

    #[test]
    fn e() {
        let ray = Ray {
            origin: vec3(0.0, 0.0, -1.0),
            dir: Vec3::unit_z(),
        };

        let intersection = intersect_box_nearest(&vec3(0.5, 0.5, 0.5), &ray);
        assert!(intersection.is_some());
        let intersection = intersection.unwrap();
        assert_eq!(intersection.t, 0.5);
        assert_eq!(intersection.normal, vec3(0.0, 0.0, -1.0));
        assert_eq!(intersection.inside, false);
    }
}
