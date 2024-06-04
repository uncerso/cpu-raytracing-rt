use cgmath::num_traits::ToPrimitive;

use crate::image::{Image, RGB};
use std::{fs::File, io::Write};

pub fn save_to_ppm(image: Image, mut file: File) {
    file.write_all(b"P6\n").unwrap();
    file.write_fmt(format_args!("{} {}\n", image.width, image.height)).unwrap();
    file.write_all(b"255\n").unwrap();
    for byte in &image.bytes {
        file.write_all(&to_byte(byte)).unwrap();
    }
}

fn f32_to_byte(v: f32) -> u8 {
    (v.clamp(0.0, 1.0) * 255.0).round().to_u8().unwrap()
}

fn to_byte(v: &RGB) -> [u8; 3] {
    [f32_to_byte(v.x), f32_to_byte(v.y), f32_to_byte(v.z)]
}
