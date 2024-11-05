use cgmath::vec3;

use crate::types::{Float, Vec3};

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

fn apply<F: Fn(Float, Float) -> Float>(a: &Vec3, b: &Vec3, f: F) -> Vec3 {
    Vec3 {
        x: f(a.x, b.x),
        y: f(a.y, b.y),
        z: f(a.z, b.z),
    }
}
