use std::fs::File;

use cgmath::{num_traits::zero, vec2};
use image::{Image, RGB};
use postprocessing::{aces_tonemap, correct_gamma};
use rand::thread_rng;
use rayon::iter::{IndexedParallelIterator, IntoParallelRefMutIterator, ParallelIterator};
use raytrace::raytrace;
use scene::Scene;
use types::Float;

mod scene_parser;
mod scene;
mod parsed_scene;
mod ppm;
mod image;
mod camera;
mod types;
mod raytrace;
mod postprocessing;
mod ray_sampler;
mod intersections;
mod ray;
mod primitives;
mod intersection_probability;
mod bvh;
mod aabb;

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
        let mut rng = thread_rng();
        let x = index % scene.dimensions.x;
        let y = index / scene.dimensions.x;
        let px = vec2(x, y);

        let mut result_color: RGB = zero();
        for _ in 0..scene.samples {
            result_color += raytrace(&camera.fuzzy_ray(px, &mut rng), &scene, &mut rng);
        }
        *pixel = correct_gamma(aces_tonemap(result_color / (scene.samples as Float)));
    });

    img
}
