use std::f64::consts::PI as WIDE_PI;

use cgmath::{Vector3, Quaternion};

pub type Float = f64;
pub type Quat = Quaternion<Float>;
pub type Vec3 = Vector3<Float>;

pub const PI: Float = WIDE_PI as Float;
pub const EPSILON: Float = Float::EPSILON * 512.0;
