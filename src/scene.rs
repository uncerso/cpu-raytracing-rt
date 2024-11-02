use std::f64::consts::PI;

use cgmath::{num_traits::zero, InnerSpace, Vector2};
use crate::{parced_scene, types::{Float, Quat, Vec3}};

#[derive(Debug)]
pub enum Material {
    Dielectric(Float /* ior */),
    Diffuse,
    Metallic,
}

#[derive(Debug, Clone)]
pub struct Triangle {
    pub a: Vec3,
    pub ba: Vec3,
    pub ca: Vec3,
    pub normal: Vec3,
    pub inverted_area: Float,
}

#[derive(Debug)]
pub enum PrimitiveType {
    Box(Vec3 /* sizes */),
    Ellipsoid(Vec3 /* radiuses */),
    Triangle(Triangle),
    Plane(Vec3 /* normal */),
}

#[derive(Debug)]
pub enum LightPrimitiveType {
    Box(Vec3 /* sizes */),
    Ellipsoid(Vec3 /* radiuses */),
    Triangle(Triangle),
}

#[derive(Debug)]
pub struct Primitive {
    pub prim_type: PrimitiveType,
    pub material: Material,
    pub position: Vec3,
    pub rotation: Quat,
    pub color: Vec3,
    pub emission: Vec3,
}

#[derive(Debug)]
pub struct LightPrimitive {
    pub prim_type: LightPrimitiveType,
    pub position: Vec3,
    pub rotation: Quat,
}

#[derive(Debug)]
pub struct CameraParams {
    pub position: Vec3,
    pub right: Vec3,
    pub up: Vec3,
    pub forward: Vec3,
    pub fov_x: Float,
}

#[derive(Debug)]
pub struct Scene {
    pub primitives: Vec<Primitive>,
    pub lights: Vec<LightPrimitive>, // just subset of primitives
    pub ray_depth: u8,
    pub bg_color: Vec3,
    pub samples: usize,
    pub camera: CameraParams,
    pub dimensions: Vector2<usize>,
}

impl Primitive {
    pub fn new(primitive: parced_scene::Primitive) -> Self {
        Self {
            prim_type: primitive.prim_type.unwrap(),
            material: match primitive.material {
                Some(material) => match material {
                    parced_scene::Material::Dielectric => Material::Dielectric(primitive.ior.unwrap()),
                    parced_scene::Material::Metallic   => Material::Metallic,
                },
                None => Material::Diffuse,
            },
            position: primitive.position.unwrap_or(zero()),
            rotation: primitive.rotation.unwrap_or(Quat::from_sv(1.0, zero())),
            color: primitive.color.unwrap_or(zero()),
            emission: primitive.emission.unwrap_or(zero()),
        }
    }
}

impl CameraParams {
    pub fn new(camera: parced_scene::CameraParams) -> Self {
        Self {
            position: camera.position.unwrap_or(zero()),
            right: camera.right.unwrap_or(Vec3::unit_x()).normalize(),
            up: camera.up.unwrap_or(Vec3::unit_y()).normalize(),
            forward: camera.forward.unwrap_or(Vec3::unit_z()).normalize(),
            fov_x: camera.fov_x.unwrap_or(PI as Float / 2.0),
        }
    }
}

impl Scene {
    pub fn new(scene: parced_scene::Scene) -> Self {
        let primitives = scene.primitives.into_iter().map(Primitive::new).collect();
        let lights = extract_lights(&primitives);
        Self {
            primitives,
            ray_depth:  scene.ray_depth.unwrap_or(16),
            bg_color:   scene.bg_color.unwrap_or(zero()),
            samples:    scene.samples.unwrap_or(64),
            dimensions: scene.dimensions.unwrap(),
            camera: CameraParams::new(scene.camera),
            lights,
        }
    }
}

fn extract_lights(primitives: &Vec<Primitive>) -> Vec<LightPrimitive> {
    primitives.iter().filter_map(|p| {
        if p.emission == zero() {
            return None;
        }
        match &p.prim_type {
            PrimitiveType::Box(sizes) => Some(LightPrimitive {
                prim_type: LightPrimitiveType::Box(*sizes), position: p.position, rotation: p.rotation
            }),
            PrimitiveType::Ellipsoid(radiuses) => Some(LightPrimitive {
                prim_type: LightPrimitiveType::Ellipsoid(*radiuses), position: p.position, rotation: p.rotation
            }),
            PrimitiveType::Triangle(triangle) => Some(LightPrimitive {
                prim_type: LightPrimitiveType::Triangle(triangle.clone()), position: p.position, rotation: p.rotation
            }),
            PrimitiveType::Plane(_) => None,
        }
    }).collect()
}
