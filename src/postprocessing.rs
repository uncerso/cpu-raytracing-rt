use cgmath::{num_traits::clamp, vec3, ElementWise};

use crate::types::{Float, Vec3};

pub fn correct_gamma(x: Vec3) -> Vec3 {
    x.map(|e| e.powf(1.0 / 2.2))
}

pub fn aces_tonemap(x: Vec3) -> Vec3 {
    let a: Float = 2.51;
    let b: Float = 0.03;
    let c: Float = 2.43;
    let d: Float = 0.59;
    let e: Float = 0.14;

    // saturate((x * (a * x + b)) / (x * (c * x + d) + e))
    saturate(
        mul_add(x, a, b).mul_element_wise(x).div_element_wise(mul_add_cwise(x, mul_add(x, c, d), e))
    )
}

fn saturate(color: Vec3) -> Vec3 {
    vec3(
        clamp(color.x, 0.0, 1.0),
        clamp(color.y, 0.0, 1.0),
        clamp(color.z, 0.0, 1.0),
    )
}

// a * x + b
fn mul_add(x: Vec3, a: Float, b: Float) -> Vec3 {
    (a * x).add_element_wise(b)
}

// a * x + b
fn mul_add_cwise(x: Vec3, a: Vec3, b: Float) -> Vec3 {
    a.mul_element_wise(x).add_element_wise(b)
}