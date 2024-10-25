use std::fs::File;

use cgmath::vec2;
use image::Image;
use postprocessing::{aces_tonemap, correct_gamma};
use rayon::iter::{IndexedParallelIterator, IntoParallelRefMutIterator, ParallelIterator};
use raytrace::raytrace;
use scene::Scene;

mod scene_parser;
mod scene;
mod parced_scene;
mod ppm;
mod image;
mod camera;
mod types;
mod raytrace;
mod postprocessing;


fn main() {
    let Some(output_file_name) = std::env::args().nth(1) else {
        println!("You must specify the output file");
        return;
    };

    let scene = Scene::new(scene_parser::parse_scene());
    let img = generate_image(&scene);
    ppm::save_to_ppm(img, File::create(output_file_name).expect("Cannot create output file"))
}

fn generate_image(scene: &Scene) -> Image {
    let camera = camera::Camera::new(&scene.camera, scene.dimensions.x, scene.dimensions.y);
    let mut img = Image::new(scene.dimensions.x, scene.dimensions.y);

    img.bytes.par_iter_mut().enumerate().for_each(|(index, pixel)| {
        let x = index % scene.dimensions.x;
        let y = index / scene.dimensions.x;
        let ray = camera.ray(vec2(x, y));
        *pixel = correct_gamma(aces_tonemap(raytrace(&ray, &scene)));
    });

    img
}
