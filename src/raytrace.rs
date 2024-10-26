use cgmath::{num_traits::{zero, Pow}, vec3, ElementWise, InnerSpace, Rotation};
use rand::{rngs::ThreadRng, Rng};

use crate::{image::RGB, rng::on_half_sphere, scene::{Primitive, Scene}, types::{Float, Quat, Vec3}};

static EPSILON: Float = 1e-8;

#[derive(Debug)]
pub struct Ray {
    pub origin: Vec3,
    pub dir: Vec3,
}

impl Ray {
    fn position_at(&self, t: Float) -> Vec3 {
        self.origin + self.dir * t
    }
}

#[derive(Debug)]
struct Intersection {
    t: Float,
    normal: Vec3,
    inside: bool,
}

impl Intersection {
    fn with_rotated_normal(self, q: Quat) -> Self {
        Self { t: self.t, normal: q.rotate_vector(self.normal), inside: self.inside }
    }
}

fn intersect<'a>(ray: &Ray, scene: &'a Scene, max_dist: Float) -> Option<(Intersection, &'a Primitive)> {
    let mut res: Option<(Intersection, &Primitive)> = None;
    for primitive in &scene.primitives {
        let model_space_ray = model_space_ray(primitive, ray);
        let intersection = match primitive.prim_type {
            crate::scene::PrimitiveType::Box(sizes) => intersect_box(&sizes, &model_space_ray),
            crate::scene::PrimitiveType::Ellipsoid(radiuses) => intersect_ellipsoid(&radiuses, &model_space_ray),
            crate::scene::PrimitiveType::Plane(normal) => intersect_plane(&normal, &model_space_ray),
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

fn model_space_ray(primitive: &Primitive, ray: &Ray) -> Ray {
    let rot = primitive.rotation.conjugate();
    Ray {
        origin: rot.rotate_vector(ray.origin - primitive.position),
        dir: rot.rotate_vector(ray.dir)
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

fn intersect_box(s: &Vec3, ray: &Ray) -> Option<Intersection> {
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

    let Some((min_intersection, max_intersection)) = &t else { return None; };
    if max_intersection.t < min_intersection.t {
        return None;
    }

    if 0.0 <= min_intersection.t {
        return Some(Intersection { t: min_intersection.t, normal: min_intersection.real_normal(), inside: false });
    }
    if 0.0 <= max_intersection.t {
        return Some(Intersection { t: max_intersection.t, normal: max_intersection.real_normal(), inside: true });
    }
    None
}

fn box_planes_intersect(s: &Vec3, ray: &Ray, index: usize) -> Option<(Float, Float, Float)> {
    if ray.dir[index] == 0.0 {
        return None;
    }
    let t1 = (s[index] - ray.origin[index]) / ray.dir[index];
    let t2 = (-s[index] - ray.origin[index]) / ray.dir[index];
    Some(if t1 < t2 { (t1, t2, 1.0) } else { (t2, t1, -1.0) } )
}

fn intersect_ellipsoid(r: &Vec3, ray: &Ray) -> Option<Intersection> {
    let origin = ray.origin.div_element_wise(*r);
    let dir = ray.dir.div_element_wise(*r);

    let c = origin.dot(origin);
    let b = origin.dot(dir);
    let a = dir.dot(dir);

    // a * t^2 + 2 * b * t + c = 1
    let d = b * b - a * (c - 1.0);
    if d < 0.0 {
        return None;
    }
    let ds = d.sqrt();
    let mut t1 = (-b + ds) / a;
    let mut t2 = (-b - ds) / a;
    if t2 < t1 {
        std::mem::swap(&mut t1, &mut t2);
    };

    if 0.0 <= t1 {
        return Some(Intersection { t: t1, normal: ray.position_at(t1).div_element_wise(*r).div_element_wise(*r).normalize(), inside: false });
    }
    if 0.0 <= t2 {
        return Some(Intersection { t: t2, normal: -ray.position_at(t2).div_element_wise(*r).div_element_wise(*r).normalize(), inside: true });
    }
    None
}

fn intersect_plane(normal: &Vec3, ray: &Ray) -> Option<Intersection> {
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

static AIR_IOR: Float = 1.0;

pub fn raytrace(ray: &Ray, scene: &Scene, rng: &mut ThreadRng) -> RGB {
    raytrace_impl(&Ray { origin: ray.origin, dir: ray.dir.normalize() }, scene, rng, scene.ray_depth)
}

fn raytrace_impl(ray: &Ray, scene: &Scene, rng: &mut ThreadRng, left_ray_depth: u8) -> RGB {
    if left_ray_depth == 0 { return zero(); }
    let Some((intersection, primitive)) = intersect(ray, scene, Float::INFINITY) else { return scene.bg_color; };
    return primitive.emission + match primitive.material {
        crate::scene::Material::Diffuse => {
            let rng_dir = on_half_sphere(&intersection.normal, rng);
            let light_from_dir = raytrace_impl(&Ray { origin: ray.position_at(intersection.t) + EPSILON * rng_dir, dir: rng_dir }, scene, rng, left_ray_depth - 1);
            2.0 * rng_dir.dot(intersection.normal) * primitive.color.mul_element_wise(light_from_dir)
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
                    if rng.gen_bool(reflection_power.clamp(0.0, 1.0)) {
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
    use cgmath::{assert_abs_diff_eq, Deg, Rotation3};

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

        let intersection = intersect_box(&vec3(0.5, 0.5, 0.5), &ray);
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
            let intersection = intersect_box(&vec3(0.5, 0.5, 0.5), &ray);
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
            let intersection = intersect_box(&vec3(0.5, 0.5, 0.5), &ray);
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
            let intersection = intersect_box(&vec3(0.5, 0.5, 0.5), &ray);
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

        let intersection = intersect_ellipsoid(&vec3(4.0, 2.0, 0.5), &ray);
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
        let intersection = intersect_ellipsoid(&r, &ray);
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

        let intersection = intersect_box(&vec3(0.5, 0.5, 0.5), &ray);
        assert!(intersection.is_some());
        let intersection = intersection.unwrap();
        assert_eq!(intersection.t, 0.5);
        assert_eq!(intersection.normal, vec3(0.0, 0.0, -1.0));
        assert_eq!(intersection.inside, false);
    }
}
