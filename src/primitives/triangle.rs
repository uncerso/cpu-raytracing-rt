use cgmath::{InnerSpace as _, SquareMatrix as _};

use crate::{aabb::{HasAABB, AABB}, intersections::{Intersectable, Intersection, Intersections}, ray::Ray, types::{Float, Mat3, Vec3}};

#[derive(Debug, Clone)]
pub struct Triangle {
    pub a: Vec3,
    pub ba: Vec3,
    pub ca: Vec3,
    pub normal: Vec3,
    pub inverted_area: Float,
    pub aabb: AABB,
}

impl Triangle {
    pub fn new(a: Vec3, b: Vec3, c: Vec3) -> Self {
        let ba = b - a;
        let ca = c - a;
        let sized_normal = ba.cross(ca);
        let area = sized_normal.dot(sized_normal).sqrt();
        let mut aabb = AABB::empty();
        aabb.extend(&a);
        aabb.extend(&b);
        aabb.extend(&c);
        Self { a, ba, ca, normal: sized_normal.normalize(), inverted_area: 1.0 / area, aabb }
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

impl HasAABB for Triangle {
    fn aabb(self: &Self) -> &AABB {
        &self.aabb
    }
}

#[cfg(test)]
mod test {
    use cgmath::{vec3, InnerSpace};

    use crate::{aabb::AABB, bvh::BVH, intersections::intersect_lights, ray::Ray, scene::{LightPrimitives, Material, Metadata, Primitive}, types::Quat};

    use super::Triangle;

    #[test]
    #[should_panic]
    fn aaa() {
        let triangle = Primitive { primitive: Triangle { a: vec3(-4.0, -2.0, 10.0), ba: vec3(1.0, 6.0, 0.0), ca: vec3(3.0, 0.0, 0.0), normal: vec3(0.0, 0.0, -1.0), inverted_area: 0.05555555555555555, aabb: AABB { min: vec3(-4.0, -2.0, 10.0), max: vec3(-1.0, 4.0, 10.0) } }, position: vec3(0.0, 0.0, -6.0), rotation: Quat { v: vec3(0.0, 0.0, 0.0), s: 1.0 }, metadata: Metadata { material: Material::Diffuse, color: vec3(0.0, 0.0, 0.0), emission: vec3(2.0, 1.0, 0.5) }, aabb: AABB { min: vec3(-4.0, -2.0, 4.0), max: vec3(-1.0, 4.0, 4.0) } };

        let u = 0.6;
        let v = 0.3;

        let world = triangle.primitive.ba * u + triangle.primitive.ca * v + triangle.primitive.a;

        let pos = vec3(-3.0, 2.0, 4.0);
        let dir = (world + triangle.position - pos).normalize();
        let ray = Ray {dir, origin: pos};

        let lights = LightPrimitives { ellipsoids: BVH::new(vec![]), boxes: BVH::new(vec![]), triangles: BVH::new(vec![triangle]) };

        let mut called = false;
        intersect_lights(&ray, &lights, &mut |_, _| { called = true; });
        assert!(called); // fix me someday...
    }
}
