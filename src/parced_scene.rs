use cgmath::Vector2;
use crate::types::{Float, Quat, Vec3};

#[derive(Debug)]
pub enum Material {
    Dielectric,
    Metallic,
}

pub type PrimitiveType = crate::scene::PrimitiveType;

#[derive(Debug)]
pub struct Primitive {
    pub prim_type: Option<PrimitiveType>,
    pub material: Option<Material>,
    pub ior: Option<Float>,
    pub position: Option<Vec3>,
    pub rotation: Option<Quat>,
    pub color: Option<Vec3>,
    pub emission: Option<Vec3>,
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

impl Primitive {
    pub fn new() -> Self {
        Self { prim_type: None, material: None, ior: None, position: None, rotation:None, color: None, emission: None }
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
