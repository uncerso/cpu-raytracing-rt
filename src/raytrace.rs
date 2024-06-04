use cgmath::{ElementWise, InnerSpace, Rotation};

use crate::{image::RGB, scene::{Primitive, Scene}, types::Vec3};

#[derive(Debug)]
pub struct Ray {
    pub origin: Vec3,
    pub dir: Vec3,
}

fn intersect(ray: &Ray, scene: &Scene) -> Option<(f32, RGB)> {
    let mut res: Option<(f32, RGB)> = None;
    for primitive in &scene.primitives {
        let model_space_ray = model_space_ray(primitive, ray);
        let intersection = match primitive.prim_type {
            crate::scene::PrimitiveType::Box(sizes) => intersect_box(&sizes, &model_space_ray),
            crate::scene::PrimitiveType::Ellipsoid(radiuses) => intersect_ellipsoid(&radiuses, &model_space_ray),
            crate::scene::PrimitiveType::Plane(normal) => intersect_plane(&normal, &model_space_ray),
        };
        let Some(dist) = intersection else { continue; };
        match res {
            Some((min_dist, _)) => {
                if dist < min_dist {
                    res = Some((dist, primitive.color));
                }
            },
            None => res = Some((dist, primitive.color)),
        }
    }
    res
}

fn model_space_ray(primitive: &Primitive, ray: &Ray) -> Ray {
    let rot = primitive.rotation.conjugate();
    Ray {
        origin: rot.rotate_vector(ray.origin - primitive.position),
        dir: rot.rotate_vector(ray.dir)
    }
}

fn intersect_box(s: &Vec3, ray: &Ray) -> Option<f32> {
    let mut t: Option<(f32, f32)> = None;
    for i in 0..3 {
        let Some((t1, t2)) = box_planes_intersect(s, &ray, i) else { continue; };
        t = Some(match t {
            Some((max_t1, min_t2)) => (t1.max(max_t1), t2.min(min_t2)),
            None => (t1, t2),
        })
    }
    t.and_then(|(min_t, max_t)| {
        if max_t < min_t { None } else { min_ordered_nonneg(min_t, max_t) }
    })
}

fn box_planes_intersect(s: &Vec3, ray: &Ray, index: usize) -> Option<(f32, f32)> {
    if ray.dir[index] == 0.0 {
        return None;
    }
    let t1 = (s[index] - ray.origin[index]) / ray.dir[index];
    let t2 = (-s[index] - ray.origin[index]) / ray.dir[index];
    Some(if t1 < t2 { (t1, t2) } else { (t2, t1) } )
}

fn intersect_ellipsoid(r: &Vec3, ray: &Ray) -> Option<f32> {
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
    min_ordered_nonneg(t1, t2)
}

fn min_ordered_nonneg(min_t: f32, max_t: f32) -> Option<f32> {
    if 0.0 <= min_t {
        return Some(min_t);
    }
    if 0.0 <= max_t {
        return Some(max_t);
    }
    None
}

fn intersect_plane(n: &Vec3, ray: &Ray) -> Option<f32> {
    let t = -n.dot(ray.origin) / n.dot(ray.dir);
    if t < 0.0 { None } else { Some(t) }
}

pub fn raytrace(ray: &Ray, scene: &Scene) -> RGB {
    match intersect(ray, scene) {
        Some((_, color)) => color,
        None => scene.bg_color,
    }
}
