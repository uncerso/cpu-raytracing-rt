use std::f64::consts::PI as WIDE_PI;

use cgmath::{Matrix3, Quaternion, Vector3};

pub type Float = f64;
pub type Quat = Quaternion<Float>;
pub type Vec3 = Vector3<Float>;
pub type Mat3 = Matrix3<Float>;

pub const PI: Float = WIDE_PI as Float;
pub const EPSILON: Float = Float::EPSILON * 512.0;
