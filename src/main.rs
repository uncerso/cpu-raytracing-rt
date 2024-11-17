use std::{fs::{self, File}, sync::atomic::{self, AtomicUsize}};

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
mod gltf;

fn main() {
    let Some(input_format) = std::env::args().nth(1) else {
        println!("You must specify the input format");
        return;
    };
    if input_format != "custom" && input_format != "glTF" {
        println!("Input format must be 'custom' or 'glTF'");
        return;
    }

    let Some(output_file_name) = std::env::args().nth(if input_format == "custom" { 2 } else { 6 }) else {
        println!("You must specify the output file");
        return;
    };

    let scene: Scene;
    println!("Started scene parsing");
    if input_format == "glTF" {
        let mut args = std::env::args();
        let Some(input_file_name) = args.nth(2) else {
            println!("You must specify the output file");
            return;
        };

        let prefix = bin_file_prefix(&input_file_name);
        let gltf = fs::read_to_string(input_file_name).expect("Couldn't find or load gltf file.");
        let buffer_provider = |file_name: &str| -> Vec<u8>{
            let file = prefix.clone() + file_name;
            fs::read(file).expect(&format!("Couldn't find or load '{}' file.", file_name))
        };
        scene = gltf::build_scene(
            gltf::parse(&gltf),
            buffer_provider,
            usize::from_str_radix(std::env::args().nth(3).as_ref().unwrap(), 10).unwrap(),
            usize::from_str_radix(std::env::args().nth(4).as_ref().unwrap(), 10).unwrap(),
            usize::from_str_radix(std::env::args().nth(5).as_ref().unwrap(), 10).unwrap(),
        );
    } else if input_format == "custom" {
        scene = Scene::new(scene_parser::parse_scene());
    } else {
        panic!("unknown input_format '{input_format}'")
    }

    let img = generate_image(&scene);
    ppm::save_to_ppm(img, File::create(output_file_name).expect("Cannot create output file"))
}

fn bin_file_prefix(gltf_file_name: &str) -> String {
    let mut bytes = gltf_file_name.as_bytes().to_vec();
    while bytes.len() > 0 && *bytes.last().unwrap() != b'/' {
        bytes.pop();
    }
    return String::from_utf8(bytes).unwrap();
}

fn generate_image(scene: &Scene) -> Image {
    let camera = camera::Camera::new(&scene.camera, scene.dimensions.x, scene.dimensions.y);
    let mut img = Image::new(scene.dimensions.x, scene.dimensions.y);

    println!("Started raytracing");

    let cnt = AtomicUsize::new(0);
    let total = scene.dimensions.x * scene.dimensions.y;

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
        let prev = cnt.fetch_add(1, atomic::Ordering::Relaxed);
        let old_percent = prev * 20 / total;
        let new_percent = (prev + 1) * 20 / total;
        if old_percent != new_percent {
            println!("{}%", new_percent * 5)
        }
    });

    img
}
