use std::f32::consts::PI;

use cgmath::{num_traits::zero, InnerSpace, Vector2};
use crate::{parced_scene, types::{Quat, Vec3}};

#[derive(Debug)]
pub enum Material {
    Dielectric(f32 /* ior */),
    Diffuse,
    Metallic,
}

#[derive(Debug)]
pub enum PrimitiveType {
    Box(Vec3 /* sizes */),
    Ellipsoid(Vec3 /* radiuses */),
    Plane(Vec3 /* normal */),
}

#[derive(Debug)]
pub struct PointLight {
    pub pos: Vec3,
    pub attenuation: Vec3,
}

#[derive(Debug)]
pub enum LightType {
    Dir(Vec3),
    Point(PointLight),
}

#[derive(Debug)]
pub struct Light {
    pub intensity: Vec3,
    pub light: LightType,
}

#[derive(Debug)]
pub struct Primitive {
    pub prim_type: PrimitiveType,
    pub material: Material,
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
    pub lights: Vec<Light>,
    pub ray_depth: u8,
    pub bg_color: Vec3,
    pub ambient_light: Vec3,
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
            color: primitive.color.unwrap(),
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
            fov_x: camera.fov_x.unwrap_or(PI / 2.0),
        }
    }
}

impl Light {
    pub fn new(light: parced_scene::Light) -> Self {
        Self {
            intensity: light.intensity.unwrap(),
            light: if light.direction.is_some() {
                LightType::Dir(light.direction.unwrap().normalize())
            } else {
                LightType::Point(PointLight{
                    pos:         light.position.unwrap(),
                    attenuation: light.attenuation.unwrap()
                })
            },
        }
    }
}

impl Scene {
    pub fn new(scene: parced_scene::Scene) -> Self {
        Self {
            primitives: scene.primitives.into_iter().map(Primitive::new).collect(),
            lights:     scene.lights.into_iter().map(Light::new).collect(),
            ray_depth:  scene.ray_depth.unwrap_or(16),
            bg_color:   scene.bg_color.unwrap_or(zero()),
            dimensions: scene.dimensions.unwrap(),
            ambient_light: scene.ambient_light.unwrap_or(zero()),
            camera: CameraParams::new(scene.camera),
        }
    }
}
