#![allow(dead_code)]
use crate::types::Float;

#[derive(Debug, serde::Deserialize)]
pub struct Node {
    #[serde(default)]
    pub children: Vec<usize>,
    pub mesh: Option<usize>,
    pub matrix: Option<Vec<Float>>,
    pub rotation: Option<Vec<Float>>,
    pub scale: Option<Vec<Float>>,
    pub translation: Option<Vec<Float>>,
    pub camera: Option<usize>,
}

#[derive(Debug, serde::Deserialize)]
pub struct Scene {
    pub nodes: Vec<usize>,
}

#[derive(Debug, serde::Deserialize)]
#[serde(rename_all = "camelCase")]
pub struct Buffer {
    pub uri: Option<String>,
    pub byte_length: usize,
}

#[derive(Debug, serde::Deserialize)]
#[serde(rename_all = "camelCase")]
pub struct BufferView {
    pub buffer: usize,
    pub byte_length: usize,
    pub byte_stride: Option<usize>,

    #[serde(default)]
    pub byte_offset: usize,
}

#[derive(Debug, serde::Deserialize)]
pub struct PerspectiveCamera {
    pub yfov: Float,
}

#[derive(Debug, serde::Deserialize)]
pub struct Camera {
    pub r#type: String,
    pub perspective: Option<PerspectiveCamera>,
}

#[derive(Debug, serde::Deserialize)]
#[serde(rename_all = "camelCase")]
pub struct Image {
    pub buffer_view: Option<usize>,
    pub mime_type: String,
    pub uri: Option<String>,
}

#[derive(Debug, serde::Deserialize)]
#[serde(rename_all = "camelCase")]
pub struct PbrMetallicRoughness {
    pub base_color_factor: Option<Vec<Float>>,

    #[serde(default = "default_one")]
    pub metallic_factor: Float,
}

#[derive(Debug, serde::Deserialize)]
#[serde(rename_all = "camelCase")]
pub struct MaterialsEmissiveStrength {
    pub emissive_strength: Float,
}

#[derive(Debug, serde::Deserialize)]
#[allow(non_snake_case)]
pub struct MaterialExtensions {
    #[serde(default)]
    pub KHR_materials_emissive_strength: MaterialsEmissiveStrength,
}

#[derive(Debug, serde::Deserialize)]
#[serde(rename_all = "camelCase")]
pub struct Material {
    #[serde(default)]
    pub extensions: MaterialExtensions,
    #[serde(default)]
    pub pbr_metallic_roughness: PbrMetallicRoughness,
    pub emissive_factor: Option<Vec<Float>>,
}


#[derive(Debug, serde::Deserialize)]
#[allow(non_snake_case)]
pub struct Attributes {
    pub POSITION: usize,
    pub NORMAL: Option<usize>,
}

#[derive(Debug, serde::Deserialize)]
pub struct Primitive {
    pub attributes: Attributes,
    pub indices: Option<usize>, // to accessor
    pub material: Option<usize>,

    #[serde(default = "default_primitive_mode")]
    pub mode: u32,
}

#[derive(Debug, serde::Deserialize)]
pub struct Mesh {
    pub primitives: Vec<Primitive>,
}

#[derive(Debug, serde::Deserialize)]
#[serde(rename_all = "camelCase")]
pub struct Sampler {
    pub mag_filter: Option<u32>,
    pub min_filter: Option<u32>,

    #[serde(default = "default_wrap")]
    pub wrap_s: u32,
    #[serde(default = "default_wrap")]
    pub wrap_t: u32,
}

#[derive(Debug, serde::Deserialize)]
#[serde(rename_all = "camelCase")]
pub struct Texture {
    pub sampler: Option<usize>,
    pub source: usize,
}

#[derive(Debug, serde::Deserialize)]
#[serde(rename_all = "camelCase")]
pub struct Accessor {
    pub buffer_view: Option<usize>,

    #[serde(default)]
    pub byte_offset: usize,

    #[serde(default)]
    pub normalized: bool,

    pub component_type: u32,
    pub count: usize,
    pub r#type: String,

}

#[derive(Debug, serde::Deserialize)]
#[serde(rename_all = "camelCase")]
pub struct Model {
    #[serde(default)]
    pub scene: usize,

    #[serde(default)]
    pub scenes: Vec<Scene>,

    #[serde(default)]
    pub nodes: Vec<Node>,

    #[serde(default)]
    pub buffers: Vec<Buffer>,

    #[serde(default)]
    pub buffer_views: Vec<BufferView>,

    #[serde(default)]
    pub cameras: Vec<Camera>,

    #[serde(default)]
    pub images: Vec<Image>,

    #[serde(default)]
    pub materials: Vec<Material>,

    #[serde(default)]
    pub meshes: Vec<Mesh>,

    #[serde(default)]
    pub samplers: Vec<Sampler>,

    #[serde(default)]
    pub textures: Vec<Texture>,

    #[serde(default)]
    pub accessors: Vec<Accessor>,
}

pub fn parse(data: &str) -> Model {
    return serde_json::from_str(&data).expect("can't parse glTF");
}

fn default_primitive_mode() -> u32 {
    4 // TRIANGLES
}

fn default_wrap() -> u32 {
    10497 // REPEAT
}

fn default_one() -> Float {
    1.0
}

impl Default for PbrMetallicRoughness {
    fn default() -> Self {
        Self { base_color_factor: None, metallic_factor: 1.0 }
    }
}

impl Default for MaterialsEmissiveStrength {
    fn default() -> Self {
        Self { emissive_strength: 1.0 }
    }
}

impl Default for MaterialExtensions {
    fn default() -> Self {
        Self { KHR_materials_emissive_strength: Default::default() }
    }
}
