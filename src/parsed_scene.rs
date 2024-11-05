use cgmath::Vector2;
use crate::{primitives, types::{Float, Quat, Vec3}};

#[derive(Debug)]
pub enum Material {
    Dielectric,
    Metallic,
}

#[derive(Debug)]
pub enum PrimitiveType {
    Box(primitives::Box),
    Ellipsoid(primitives::Ellipsoid),
    Triangle(primitives::Triangle),
    Plane(primitives::Plane),
}

#[derive(Debug)]
pub struct PrimitiveProperties {
    pub material: Option<Material>,
    pub ior: Option<Float>,
    pub position: Option<Vec3>,
    pub rotation: Option<Quat>,
    pub color: Option<Vec3>,
    pub emission: Option<Vec3>,
}

#[derive(Debug)]
pub struct Primitive {
    pub prim_type: Option<PrimitiveType>,
    pub properties: PrimitiveProperties,
}

#[derive(Debug)]
pub struct CameraParams {
    pub position: Option<Vec3>,
    pub right: Option<Vec3>,
    pub up: Option<Vec3>,
    pub forward: Option<Vec3>,
    pub fov_x: Option<Float>,
}

#[derive(Debug)]
pub struct Scene {
    pub primitives: Vec<Primitive>,
    pub ray_depth: Option<u8>,
    pub bg_color: Option<Vec3>,
    pub samples: Option<usize>,
    pub camera: CameraParams,
    pub dimensions: Option<Vector2<usize>>,
}

impl PrimitiveProperties {
    pub fn new() -> Self {
        Self { material: None, ior: None, position: None, rotation:None, color: None, emission: None }
    }
}

impl Primitive {
    pub fn new() -> Self {
        Self { prim_type: None, properties: PrimitiveProperties::new() }
    }
}

impl CameraParams {
    pub fn new() -> Self {
        Self { position: None, right: None, up: None, forward: None, fov_x: None}
    }
}

impl Scene {
    pub fn new() -> Self {
        Self { primitives: vec![], ray_depth: None, bg_color: None, samples: None, camera: CameraParams::new(), dimensions: None }
    }
}
