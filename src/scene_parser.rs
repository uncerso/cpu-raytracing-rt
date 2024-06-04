use std::{io, str::{SplitAsciiWhitespace, FromStr}, fmt::Debug};
use crate::scene::{Primitive, PrimitiveType, Scene};
use cgmath::{vec2, vec3, Quaternion, Vector2, Vector3};

pub fn parse_scene() -> Scene {
    let mut scene = Scene::new();

    for line in io::stdin().lines() {
        let Ok(line) = line else {continue;};
        let mut parts = line.split_ascii_whitespace();
        match parts.next() {
            Some("BOX") => {
                scene.primitives.push(Primitive::new(PrimitiveType::Box(next_vec3(&mut parts))));
            },
            Some("PLANE") => {
                scene.primitives.push(Primitive::new(PrimitiveType::Plane(next_vec3(&mut parts))));
            },
            Some("ELLIPSOID") => {
                scene.primitives.push(Primitive::new(PrimitiveType::Ellipsoid(next_vec3(&mut parts))));
            },
            Some("POSITION") => scene.primitives.last_mut().unwrap().position = next_vec3(&mut parts),
            Some("ROTATION") => scene.primitives.last_mut().unwrap().rotation = next_quat(&mut parts),
            Some("COLOR") => scene.primitives.last_mut().unwrap().color = next_vec3(&mut parts),
            Some("BG_COLOR") => scene.bg_color = next_vec3(&mut parts),
            Some("CAMERA_POSITION") => scene.camera.position = next_vec3(&mut parts),
            Some("CAMERA_RIGHT") => scene.camera.right = next_vec3(&mut parts),
            Some("CAMERA_UP") => scene.camera.up = next_vec3(&mut parts),
            Some("CAMERA_FORWARD") => scene.camera.forward = next_vec3(&mut parts),
            Some("CAMERA_FOV_X") => scene.camera.fov_x = next(&mut parts),
            Some("DIMENSIONS") => scene.dimensions = next_vec2(&mut parts),
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
