use cgmath::{InnerSpace as _, SquareMatrix as _};

use crate::{intersections::{Intersectable, Intersection, Intersections}, ray::Ray, types::{Float, Mat3, Vec3}};

#[derive(Debug, Clone)]
pub struct Triangle {
    pub a: Vec3,
    pub ba: Vec3,
    pub ca: Vec3,
    pub normal: Vec3,
    pub inverted_area: Float,
}

impl Triangle {
    pub fn new(a: Vec3, b: Vec3, c: Vec3) -> Self {
        let ba = b - a;
        let ca = c - a;
        let sized_normal = ba.cross(ca);
        let area = sized_normal.dot(sized_normal).sqrt() / 2.0;
        Self { a, ba, ca, normal: sized_normal.normalize(), inverted_area: 1.0 / area }
    }
}

impl Intersectable for Triangle {
    fn intersection(self: &Self, ray: &Ray) -> Option<Intersection> {
        let Some(matrix) = Mat3::from_cols(self.ba, self.ca, -ray.dir).invert() else { return None; };
        let Vec3 { x: u, y: v, z: t } = matrix * (ray.origin - self.a);
        if u < 0.0 || v < 0.0 || 1.0 < u + v || t < 0.0 {
            return None;
        }
        let normal = &self.normal;
        let inside = ray.dir.dot(*normal) > 0.0;
        return Some(Intersection {
            t,
            normal: if inside { -*normal } else { *normal },
            inside,
        });
    }

    fn all_intersections(self: &Self, ray: &Ray) -> Intersections {
        return match self.intersection(ray) {
            None => Intersections::None,
            Some(intersection) => Intersections::One(intersection),
        };
    }
}

#[cfg(test)]
mod test {
    use cgmath::{vec3, InnerSpace};

    use crate::{bvh::BVH, intersections::intersect_lights, parsed_scene, ray::Ray, scene::{LightPrimitives, TrianglePrimitive}};

    use super::Triangle;

    #[test]
    fn aaa() {
        let primitive = Triangle { a: vec3(-4.0, -2.0, 10.0), ba: vec3(1.0, 6.0, 0.0), ca: vec3(3.0, 0.0, 0.0), normal: vec3(0.0, 0.0, -1.0), inverted_area: 0.05555555555555555 };
        let properties = parsed_scene::PrimitiveProperties { material: None, ior: None, position: Some(vec3(0.0, 0.0, -6.0)), rotation: None, color: None, emission: None };

        let triangle = TrianglePrimitive::new(primitive, properties);

        let u = 0.6;
        let v = 0.3;

        let world = triangle.primitive.ba * u + triangle.primitive.ca * v + triangle.primitive.a;

        let pos = vec3(-3.0, 2.0, 4.0);
        let dir = (world + pos).normalize();
        let ray = Ray {dir, origin: pos};

        let lights = LightPrimitives { ellipsoids: BVH::new(vec![]), boxes: BVH::new(vec![]), triangles: BVH::new(vec![triangle]) };

        let mut called = false;
        intersect_lights(&ray, &lights, &mut |_, _| { called = true; });
        assert!(called);
    }
}
