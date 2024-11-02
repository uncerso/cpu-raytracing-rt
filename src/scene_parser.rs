use std::{fmt::Debug, io, str::{FromStr, SplitAsciiWhitespace}};
use crate::{parced_scene::{Material, Primitive, PrimitiveType, Scene}, scene::Triangle, types::Vec3};
use cgmath::{vec2, vec3, InnerSpace, Quaternion, Vector2, Vector3};

pub fn parse_scene() -> Scene {
    let mut scene = Scene::new();

    for line in io::stdin().lines() {
        let Ok(line) = line else {continue;};
        let mut parts = line.split_ascii_whitespace();
        match parts.next() {
            Some("NEW_PRIMITIVE") => scene.primitives.push(Primitive::new()),
            Some("BOX")       => scene.primitives.last_mut().unwrap().prim_type = Some(PrimitiveType::Box(next_vec3(&mut parts))),
            Some("PLANE")     => scene.primitives.last_mut().unwrap().prim_type = Some(PrimitiveType::Plane(next_vec3(&mut parts))),
            Some("ELLIPSOID") => scene.primitives.last_mut().unwrap().prim_type = Some(PrimitiveType::Ellipsoid(next_vec3(&mut parts))),
            Some("TRIANGLE")  => scene.primitives.last_mut().unwrap().prim_type = Some(PrimitiveType::Triangle(next_triangle(&mut parts))),
            Some("POSITION")  => scene.primitives.last_mut().unwrap().position  = Some(next_vec3(&mut parts)),
            Some("ROTATION")  => scene.primitives.last_mut().unwrap().rotation  = Some(next_quat(&mut parts)),
            Some("COLOR")     => scene.primitives.last_mut().unwrap().color     = Some(next_vec3(&mut parts)),
            Some("EMISSION")  => scene.primitives.last_mut().unwrap().emission  = Some(next_vec3(&mut parts)),

            Some("METALLIC")   => scene.primitives.last_mut().unwrap().material = Some(Material::Metallic),
            Some("DIELECTRIC") => scene.primitives.last_mut().unwrap().material = Some(Material::Dielectric),
            Some("IOR")        => scene.primitives.last_mut().unwrap().ior      = Some(next(&mut parts)),

            Some("CAMERA_POSITION") => scene.camera.position = Some(next_vec3(&mut parts)),
            Some("CAMERA_RIGHT")    => scene.camera.right    = Some(next_vec3(&mut parts)),
            Some("CAMERA_UP")       => scene.camera.up       = Some(next_vec3(&mut parts)),
            Some("CAMERA_FORWARD")  => scene.camera.forward  = Some(next_vec3(&mut parts)),
            Some("CAMERA_FOV_X")    => scene.camera.fov_x    = Some(next(&mut parts)),

            Some("DIMENSIONS")    => scene.dimensions    = Some(next_vec2(&mut parts)),
            Some("RAY_DEPTH")     => scene.ray_depth     = Some(next(&mut parts)),
            Some("BG_COLOR")      => scene.bg_color      = Some(next_vec3(&mut parts)),
            Some("SAMPLES")       => scene.samples       = Some(next(&mut parts)),

            Some(_) => { continue; }
            None => { continue; }
        }
    }

    scene
}

fn next<T: FromStr>(parts: &mut SplitAsciiWhitespace) -> T
    where <T as FromStr>::Err: Debug
{
    parts.next().unwrap().parse().unwrap()
}

fn next_quat<T: FromStr>(parts: &mut SplitAsciiWhitespace) -> Quaternion<T>
    where <T as FromStr>::Err: Debug
{
    let xyz = next_vec3(parts);
    let w = next(parts);
    Quaternion::from_sv(w, xyz)
}

fn next_vec3<T: FromStr>(parts: &mut SplitAsciiWhitespace) -> Vector3<T>
    where <T as FromStr>::Err: Debug
{
    vec3(next(parts), next(parts), next(parts))
}

fn next_vec2<T: FromStr>(parts: &mut SplitAsciiWhitespace) -> Vector2<T>
    where <T as FromStr>::Err: Debug
{
    vec2(next(parts), next(parts))
}

fn next_triangle(parts: &mut SplitAsciiWhitespace) -> Triangle {
    let a: Vec3 = next_vec3(parts);
    let b: Vec3 = next_vec3(parts);
    let c: Vec3 = next_vec3(parts);
    let ba = b - a;
    let ca = c - a;
    let sized_normal = ba.cross(ca);
    let area = sized_normal.dot(sized_normal).sqrt();
    Triangle { a, ba, ca, normal: sized_normal.normalize(), inverted_area: 1.0 / area }
}
