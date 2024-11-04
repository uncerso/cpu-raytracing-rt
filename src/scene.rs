use std::f64::consts::PI;

use cgmath::{num_traits::zero, InnerSpace, Vector2};
use crate::{parsed_scene::{self, PrimitiveType}, primitives::{Box, Ellipsoid, Plane, Triangle}, types::{Float, Quat, Vec3}};

#[derive(Debug, Clone)]
pub enum Material {
    Dielectric(Float /* ior */),
    Diffuse,
    Metallic,
}

#[derive(Debug, Clone)]
pub struct Metadata {
    pub material: Material,
    pub color: Vec3,
    pub emission: Vec3,
}

#[derive(Debug, Clone)]
pub struct Primitive<T> {
    pub primitive: T,
    pub position: Vec3,
    pub rotation: Quat,
    pub metadata: Metadata,
}

#[derive(Debug)]
pub struct CameraParams {
    pub position: Vec3,
    pub right: Vec3,
    pub up: Vec3,
    pub forward: Vec3,
    pub fov_x: Float,
}

type Planes = Vec<Primitive<Plane>>;
type Ellipsoids = Vec<Primitive<Ellipsoid>>;
type Boxes = Vec<Primitive<Box>>;
type Triangles = Vec<Primitive<Triangle>>;

#[derive(Debug)]
pub struct ScenePrimitives {
    pub planes: Planes,
    pub ellipsoids: Ellipsoids,
    pub boxes: Boxes,
    pub triangles: Triangles,
}

#[derive(Debug)]
pub struct LightPrimitives {
    pub ellipsoids: Ellipsoids,
    pub boxes: Boxes,
    pub triangles: Triangles,
}

impl LightPrimitives {
    pub fn len(&self) -> usize {
        self.ellipsoids.len() + self.boxes.len() + self.triangles.len()
    }

    pub fn is_empty(&self) -> bool {
        self.ellipsoids.is_empty() && self.boxes.is_empty() && self.triangles.is_empty()
    }
}

#[derive(Debug)]
pub struct Scene {
    pub primitives: ScenePrimitives,
    pub lights: LightPrimitives, // just subset of primitives
    pub ray_depth: u8,
    pub bg_color: Vec3,
    pub samples: usize,
    pub camera: CameraParams,
    pub dimensions: Vector2<usize>,
}

impl Metadata {
    pub fn new(properties: &parsed_scene::PrimitiveProperties) -> Self {
        Self {
            material: match &properties.material {
                Some(material) => match material {
                    parsed_scene::Material::Dielectric => Material::Dielectric(properties.ior.unwrap()),
                    parsed_scene::Material::Metallic   => Material::Metallic,
                },
                None => Material::Diffuse,
            },
            color: properties.color.unwrap_or(zero()),
            emission: properties.emission.unwrap_or(zero()),
        }
    }
}

impl<T> Primitive<T> {
    pub fn new(primitive: T, properties: parsed_scene::PrimitiveProperties) -> Self {
        Self {
            primitive,
            metadata: Metadata::new(&properties),
            position: properties.position.unwrap_or(zero()),
            rotation: properties.rotation.unwrap_or(Quat::from_sv(1.0, zero())),
        }
    }
}

impl ScenePrimitives {
    fn new(primitives: Vec<parsed_scene::Primitive>) -> Self {
        let mut planes: Planes = vec![];
        let mut ellipsoids: Ellipsoids = vec![];
        let mut boxes: Boxes = vec![];
        let mut triangles: Triangles = vec![];

        for primitive in primitives {
            match primitive.prim_type.unwrap() {
                PrimitiveType::Box(r#box) => boxes.push(Primitive::new(r#box, primitive.properties)),
                PrimitiveType::Ellipsoid(ellipsoid) => ellipsoids.push(Primitive::new(ellipsoid, primitive.properties)),
                PrimitiveType::Triangle(triangle) => triangles.push(Primitive::new(triangle, primitive.properties)),
                PrimitiveType::Plane(plane) => planes.push(Primitive::new(plane, primitive.properties)),
            }
        }

        Self { planes, ellipsoids, boxes, triangles }
    }
}

impl CameraParams {
    fn new(camera: parsed_scene::CameraParams) -> Self {
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
    pub fn new(scene: parsed_scene::Scene) -> Self {
        let primitives = ScenePrimitives::new(scene.primitives);
        let lights = LightPrimitives {
            boxes: primitives.boxes.iter().filter_map(copy_if_light).collect(),
            ellipsoids: primitives.ellipsoids.iter().filter_map(copy_if_light).collect(),
            triangles: primitives.triangles.iter().filter_map(copy_if_light).collect(),
        };
        Self {
            primitives,
            lights,
            ray_depth:  scene.ray_depth.unwrap_or(16),
            bg_color:   scene.bg_color.unwrap_or(zero()),
            samples:    scene.samples.unwrap_or(64),
            dimensions: scene.dimensions.unwrap(),
            camera: CameraParams::new(scene.camera),
        }
    }
}

fn is_light<T>(primitive: &Primitive<T>) -> bool {
    primitive.metadata.emission != zero()
}

fn copy_if_light<T: Clone>(primitive: &Primitive<T>) -> Option<Primitive<T>> {
    if is_light(primitive) {
        return Some(primitive.clone());
    }
    return None;
}
