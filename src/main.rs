use std::fs::File;

use cgmath::{num_traits::ToPrimitive, vec2};
use image::Image;
use raytrace::raytrace;
use scene::Scene;

mod scene_parser;
mod scene;
mod ppm;
mod image;
mod camera;
mod types;
mod raytrace;


fn main() {
    let Some(output_file_name) = std::env::args().nth(1) else {
        println!("You must specify the output file");
        return;
    };

    let scene = scene_parser::parse_scene();
    let img = generate_image(&scene);
    ppm::save_to_ppm(img, File::create(output_file_name).expect("Cannot create output file"))
}

fn generate_image(scene: &Scene) -> Image {
    let camera = camera::Camera::new(&scene.camera, scene.dimensions.x, scene.dimensions.y);
    let mut img = Image::new(scene.dimensions.x, scene.dimensions.y);

    for x in 0..scene.dimensions.x {
        for y in 0..scene.dimensions.y {
            let ray = camera.ray(vec2(x, y));
            img.bytes[(y * scene.dimensions.x + x).to_usize().unwrap()] = raytrace(&ray, &scene);
        }
    }

    img
}