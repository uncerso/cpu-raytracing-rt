use cgmath::num_traits::zero;

use crate::types::Float;

pub type RGB = cgmath::Vector3<Float>;
pub struct Image {
    pub width: usize,
    pub height: usize,
    pub bytes: Vec<RGB>,
}

impl Image {
    pub fn new(width: usize, height: usize) -> Self {
        Self { width, height, bytes: vec![zero(); width * height] }
    }
}
