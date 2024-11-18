use cgmath::{vec3, ElementWise as _};

use crate::{ray::Ray, types::{Float, Vec3}};

#[derive(Debug, Clone)]
pub struct AABB {
    pub min: Vec3,
    pub max: Vec3,
}

pub trait HasAABB {
    fn aabb(self: &Self) -> &AABB;
}

impl AABB {
    pub fn empty() -> Self {
        Self {
            min: vec3(Float::INFINITY, Float::INFINITY, Float::INFINITY),
            max: vec3(-Float::INFINITY, -Float::INFINITY, -Float::INFINITY),
        }
    }

    pub fn extend(self: &mut Self, v: &Vec3) {
        self.min = apply(&self.min, v, min);
        self.max = apply(&self.max, v, max);
    }

    pub fn extend_aabb(self: &mut Self, v: &AABB) {
        self.min = apply(&self.min, &v.min, min);
        self.max = apply(&self.max, &v.max, max);
    }
}

fn min(a: Float, b: Float) -> Float {
    if a < b { a } else { b }
}

fn max(a: Float, b: Float) -> Float {
    if b < a { a } else { b }
}

fn apply(a: &Vec3, b: &Vec3, f: fn(Float, Float) -> Float) -> Vec3 {
    Vec3 {
        x: f(a.x, b.x),
        y: f(a.y, b.y),
        z: f(a.z, b.z),
    }
}

impl AABB {
    pub fn intersects(&self, ray: &Ray) -> Option<Float> {
        for i in 0..3 {
            if ray.dir[i] == 0.0 && (ray.origin[i] < self.min[i] || self.max[i] < ray.origin[i]) {
                return None;
            }
        }

        if self.inside(&ray.origin) {
            return Some(0.0);
        }

        let t_min = (self.min - ray.origin).div_element_wise(ray.dir);
        let t_max = (self.max - ray.origin).div_element_wise(ray.dir);
        let t1 = apply(&t_min, &t_max, safe_min);
        let t2 = apply(&t_min, &t_max, safe_max);
        let t_near = safe_max(safe_max(t1.x, t1.y), t1.z);
        let t_far = safe_min(safe_min(t2.x, t2.y), t2.z);
        if t_near > t_far {
            return None;
        }
        if 0.0 <= t_near {
            return Some(t_near);
        }
        if 0.0 <= t_far {
            return Some(t_far);
        }
        None
    }

    fn inside(&self, origin: &Vec3) -> bool {
        for i in 0..3 {
            if origin[i] < self.min[i] || self.max[i] < origin[i] {
                return false;
            }
        }
        return true;
    }
}

fn safe_min(a: Float, b: Float) -> Float {
    if !a.is_finite() {
        return b;
    }
    if !b.is_finite() {
        return a;
    }
    min(a, b)
}

fn safe_max(a: Float, b: Float) -> Float {
    if !a.is_finite() {
        return b;
    }
    if !b.is_finite() {
        return a;
    }
    max(a, b)
}

#[cfg(test)]
mod test {
    use cgmath::{vec3, InnerSpace};

    use crate::{ray::Ray, types::Float};

    use super::AABB;

    #[test]
    fn a() {
        let aabb = AABB { min: vec3(-1.0, -2.0, -1.0), max: vec3(1.0, 2.0, 1.0) };
        let ray = Ray { origin: vec3(0.0, 0.0, 2.0), dir: vec3(0.0, 0.0, 1.0).normalize() };
        assert!(aabb.intersects(&ray).is_none());
    }

    #[test]
    fn b() {
        let aabb = AABB { min: vec3(-1.0, -2.0, -1.0), max: vec3(1.0, 2.0, 1.0) };
        let ray = Ray { origin: vec3(0.0, 0.0, -2.0), dir: vec3(0.0, 0.0, 1.0).normalize() };
        assert_eq!(aabb.intersects(&ray), Some(1.0));
    }

    #[test]
    fn c() {
        let aabb = AABB { min: vec3(-1.0, -2.0, -1.0), max: vec3(1.0, 2.0, 1.0) };
        let ray = Ray { origin: vec3(2.0, 0.0, -2.0), dir: vec3(0.0, 0.0, 1.0).normalize() };
        assert!(aabb.intersects(&ray).is_none());
    }

    #[test]
    fn d() {
        let aabb = AABB { min: vec3(-1.0, -2.0, -1.0), max: vec3(1.0, 2.0, 1.0) };
        let ray = Ray { origin: vec3(-2.0, 0.0, -2.0), dir: vec3(1.0, 0.0, 1.0).normalize() };
        assert!(aabb.intersects(&ray) == Some((2.0 as Float).sqrt()));
    }

    #[test]
    fn e() {
        let aabb = AABB { min: vec3(-1.0, -2.0, -1.0), max: vec3(1.0, 2.0, 1.0) };
        let ray = Ray { origin: vec3(-1.0, 0.0, -2.0), dir: vec3(0.0, 0.0, 1.0).normalize() };
        assert_eq!(aabb.intersects(&ray), Some(1.0));
    }
}
