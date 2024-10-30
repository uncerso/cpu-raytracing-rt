use crate::types::{Float, Vec3};

#[derive(Debug)]
pub struct Ray {
    pub origin: Vec3,
    pub dir: Vec3,
}

impl Ray {
    pub fn position_at(&self, t: Float) -> Vec3 {
        self.origin + self.dir * t
    }
}
