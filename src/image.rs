use cgmath::num_traits::{zero, AsPrimitive};

pub type RGB = cgmath::Vector3<f32>;
pub struct Image {
    pub width: u32,
    pub height: u32,
    pub bytes: Vec<RGB>,
}

impl Image {
    pub fn new(width: u32, height: u32) -> Self {
        Self { width, height, bytes: vec![zero(); (width * height).as_()] }
    }
}