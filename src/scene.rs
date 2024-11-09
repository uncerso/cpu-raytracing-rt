use std::{f64::consts::PI, fmt::Debug};

use cgmath::{num_traits::zero, vec3, InnerSpace, Rotation, Vector2};
use crate::{aabb::{HasAABB, AABB}, bvh::BVH, parsed_scene::{self, PrimitiveType}, primitives::{Box, Ellipsoid, Plane, Triangle}, types::{Float, Quat, Vec3}};

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
    pub(crate) aabb: AABB,
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
    pub ellipsoids: BVH<Primitive<Ellipsoid>>,
    pub boxes: BVH<Primitive<Box>>,
    pub triangles: BVH<Primitive<Triangle>>,
}

#[derive(Debug)]
pub struct LightPrimitives {
    pub ellipsoids: BVH<Primitive<Ellipsoid>>,
    pub boxes: BVH<Primitive<Box>>,
    pub triangles: BVH<Primitive<Triangle>>,
}

impl LightPrimitives {
    pub fn len(&self) -> usize {
        self.ellipsoids.primitives().len() + self.boxes.primitives().len() + self.triangles.primitives().len()
    }

    pub fn is_empty(&self) -> bool {
        self.ellipsoids.primitives().is_empty() && self.boxes.primitives().is_empty() && self.triangles.primitives().is_empty()
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

impl<T: HasAABB + Debug> Primitive<T> {
    pub fn new(primitive: T, properties: parsed_scene::PrimitiveProperties) -> Self {
        let position = properties.position.unwrap_or(zero());
        let rotation = properties.rotation.unwrap_or(Quat::from_sv(1.0, zero()));
        let mut aabb = rotated_aabb(primitive.aabb(), rotation);
        aabb.min += position;
        aabb.max += position;
        Self {
            primitive,
            position,
            rotation,
            aabb,
            metadata: Metadata::new(&properties),
        }
    }
}

impl<T> Primitive<T> {
    pub fn new_without_aabb(primitive: T, properties: parsed_scene::PrimitiveProperties) -> Self {
        let position = properties.position.unwrap_or(zero());
        let rotation = properties.rotation.unwrap_or(Quat::from_sv(1.0, zero()));
        Self {
            primitive,
            position,
            rotation,
            metadata: Metadata::new(&properties),
            aabb: AABB::empty(),
        }
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
        let (primitives, lights) = make_scenes(scene.primitives);
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

fn make_scenes(primitives: Vec<parsed_scene::Primitive>) -> (ScenePrimitives, LightPrimitives) {
    let mut planes: Planes = vec![];
    let mut ellipsoids: Ellipsoids = vec![];
    let mut boxes: Boxes = vec![];
    let mut triangles: Triangles = vec![];

    for primitive in primitives {
        match primitive.prim_type.unwrap() {
            PrimitiveType::Box(r#box) => boxes.push(Primitive::new(r#box, primitive.properties)),
            PrimitiveType::Ellipsoid(ellipsoid) => ellipsoids.push(Primitive::new(ellipsoid, primitive.properties)),
            PrimitiveType::Triangle(triangle) => triangles.push(Primitive::new(triangle, primitive.properties)),
            PrimitiveType::Plane(plane) => planes.push(Primitive::new_without_aabb(plane, primitive.properties)),
        }
    }

    let lights = LightPrimitives {
        boxes: BVH::new(boxes.iter().filter_map(copy_if_light).collect()),
        ellipsoids: BVH::new(ellipsoids.iter().filter_map(copy_if_light).collect()),
        triangles: BVH::new(triangles.iter().filter_map(copy_if_light).collect()),
    };

    let scene_primitives = ScenePrimitives {
        planes,
        ellipsoids: BVH::new(ellipsoids),
        boxes: BVH::new(boxes),
        triangles: BVH::new(triangles),
    };

    (scene_primitives, lights)
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

impl<T: HasAABB> HasAABB for Primitive<T> {
    fn aabb(self: &Self) -> &AABB {
        &self.aabb
    }
}

fn rotated_aabb(aabb: &AABB, r: Quat) -> AABB {
    let min = &aabb.min;
    let max = &aabb.max;
    let mut bbox = AABB::empty();
    bbox.extend(&r.rotate_vector(vec3(min.x, min.y, min.z)));
    bbox.extend(&r.rotate_vector(vec3(min.x, min.y, max.z)));
    bbox.extend(&r.rotate_vector(vec3(min.x, max.y, min.z)));
    bbox.extend(&r.rotate_vector(vec3(min.x, max.y, max.z)));
    bbox.extend(&r.rotate_vector(vec3(max.x, min.y, min.z)));
    bbox.extend(&r.rotate_vector(vec3(max.x, min.y, max.z)));
    bbox.extend(&r.rotate_vector(vec3(max.x, max.y, min.z)));
    bbox.extend(&r.rotate_vector(vec3(max.x, max.y, max.z)));
    bbox
}
