use cgmath::Vector2;
use rand::{rngs::ThreadRng, Rng};

use crate::{ray::Ray, scene::{CameraParams, Fov}, types::{Float, Vec3}};

pub struct Camera {
    position: Vec3,
    right: Vec3,
    up: Vec3,
    forward: Vec3,
    tan_half_fov_x: Float,
    tan_half_fov_y: Float,
    width: Float,
    height: Float,
}

impl Camera {
    pub fn new(params: &CameraParams, width: usize, height: usize) -> Self {
        let fwidth = width as Float;
        let fheight = height as Float;
        let tan_half_fov_x: Float;
        let tan_half_fov_y: Float;
        match params.fov {
            Fov::Y(yfov) => {
                tan_half_fov_y = (yfov / 2.0).tan();
                let aspect_ratio =  fheight / fwidth;
                tan_half_fov_x = tan_half_fov_y / aspect_ratio;

            },
            Fov::X(xfov) => {
                tan_half_fov_x = (xfov / 2.0).tan();
                let aspect_ratio = fwidth / fheight;
                tan_half_fov_y = tan_half_fov_x / aspect_ratio;
            },
        }
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

    pub fn fuzzy_ray(&self, pixel: Vector2<usize>, rng: &mut ThreadRng) -> Ray {
        let px = pixel.x as Float + rng.gen_range(0.0..1.0);
        let py = pixel.y as Float + rng.gen_range(0.0..1.0);
        let x = (2.0 * px / self.width - 1.0) * self.tan_half_fov_x;
        let y = -(2.0 * py / self.height - 1.0) * self.tan_half_fov_y;
        let dir = x * self.right + y * self.up + 1.0 * self.forward;
        Ray { origin: self.position, dir }
    }
}