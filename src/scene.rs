use std::f32::consts::PI;

use cgmath::{num_traits::zero, Vector2};
use crate::types::{Vec3, Quat};

#[derive(Debug)]
pub enum PrimitiveType {
    Box(Vec3),
    Ellipsoid(Vec3),
    Plane(Vec3),
}

#[derive(Debug)]
pub struct Primitive {
    pub prim_type: PrimitiveType,
    pub position: Vec3,
    pub rotation: Quat,
    pub color: Vec3,
}

#[derive(Debug)]
pub struct CameraParams {
    pub position: Vec3,
    pub right: Vec3,
    pub up: Vec3,
    pub forward: Vec3,
    pub fov_x: f32,
}

#[derive(Debug)]
pub struct Scene {
    pub primitives: Vec<Primitive>,
    pub bg_color: Vec3,
    pub camera: CameraParams,
    pub dimensions: Vector2<u32>,
}

impl Primitive {
    pub fn new(prim_type: PrimitiveType) -> Self {
        Self { prim_type, position: zero(), rotation: Quat::from_sv(1.0, zero()), color: zero() }
    }
}

impl CameraParams {
    pub fn new() -> Self {
        Self { position: zero(), right: Vec3::unit_x(), up: Vec3::unit_y(), forward: Vec3::unit_z(), fov_x: PI / 2.0 }
    }
}

impl Scene {
    pub fn new() -> Self {
        Self { primitives: vec![], bg_color: zero(), camera: CameraParams::new(), dimensions: zero() }
    }
}