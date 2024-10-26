use cgmath::InnerSpace;
use rand::{rngs::ThreadRng, Rng};

use crate::types::{Float, Vec3};

pub fn on_half_sphere(normal: &Vec3, rng: &mut ThreadRng) -> Vec3 {
    let v = on_sphere(rng);
    if normal.dot(v) < 0.0 { -v } else { v }
}

pub fn on_sphere(rng: &mut ThreadRng) -> Vec3 {
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
