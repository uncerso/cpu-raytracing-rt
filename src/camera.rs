use cgmath::{num_traits::ToPrimitive, Vector2};

use crate::{raytrace::Ray, scene::CameraParams, types::Vec3};

pub struct Camera {
    position: Vec3,
    right: Vec3,
    up: Vec3,
    forward: Vec3,
    tan_half_fov_x: f32,
    tan_half_fov_y: f32,
    width: f32,
    height: f32,
}

impl Camera {
    pub fn new(params: &CameraParams, width: u32, height: u32) -> Self {
        let fwidth = width.to_f32().unwrap();
        let fheight = height.to_f32().unwrap();
        let tan_half_fov_x = (params.fov_x / 2.0).tan();
        let aspect_ratio = fwidth / fheight;
        let tan_half_fov_y = tan_half_fov_x / aspect_ratio;
        Self {
            position: params.position,
            right: params.right,
            up: params.up,
            forward: params.forward,
            tan_half_fov_x,
            tan_half_fov_y,
            width: fwidth,
            height: fheight,
        }
    }

    pub fn ray(&self, pixel: Vector2<u32>) -> Ray {
        let px  = pixel.x.to_f32().unwrap() + 0.5;
        let py  = pixel.y.to_f32().unwrap() + 0.5;
        let x = (2.0 * px / self.width - 1.0) * self.tan_half_fov_x;
        let y = -(2.0 * py / self.height - 1.0) * self.tan_half_fov_y;
        let dir = x * self.right + y * self.up + 1.0 * self.forward;
        Ray { origin: self.position, dir }
    }
}