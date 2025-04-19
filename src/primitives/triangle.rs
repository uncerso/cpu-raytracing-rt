use cgmath::{InnerSpace as _, Matrix as _, SquareMatrix as _};

use crate::{intersections::{Intersectable, Intersection, Intersections}, ray::Ray, types::{Float, Mat3, Vec3}};

#[derive(Debug, Clone)]
pub struct Triangle {
    pub a: Vec3,
    pub ba: Vec3,
    pub ca: Vec3,
    pub geometry_normal: Vec3,
    pub inverted_area: Float,

    // smooth normals
    pub na: Vec3,
    pub nb: Vec3,
    pub nc: Vec3,
}

impl Triangle {
    pub fn new_with_smooth_normal(a: Vec3, b: Vec3, c: Vec3, na: Vec3, nb: Vec3, nc: Vec3) -> Self {
        let Props { ba, ca, geometry_normal, inverted_area } = props(&a, &b, &c);
        Self { a, ba, ca, geometry_normal, inverted_area, na, nb, nc }
    }

    pub fn new_with_geometry_normals(a: Vec3, b: Vec3, c: Vec3) -> Self {
        let Props { ba, ca, geometry_normal, inverted_area } = props(&a, &b, &c);
        Self { a, ba, ca, geometry_normal, inverted_area, na: geometry_normal, nb: geometry_normal, nc: geometry_normal }
    }

    fn shading_normal(&self, u: Float, v: Float) -> Vec3 {
        (self.na + u * (self.nb - self.na) + v * (self.nc - self.na)).normalize()
    }
}

struct Props {
    ba: Vec3,
    ca: Vec3,
    geometry_normal: Vec3,
    inverted_area: Float,
}
fn props(a: &Vec3, b: &Vec3, c: &Vec3) -> Props {
    let ba = b - a;
    let ca = c - a;
    let sized_normal = ba.cross(ca);
    let area = sized_normal.dot(sized_normal).sqrt() / 2.0;
    Props { ba, ca, geometry_normal: sized_normal.normalize(), inverted_area: 1.0 / area }
}

fn invert(mt: Mat3) -> Option<Mat3> {
    let det = mt.determinant();
    if det.abs() < 1e-11 {
        None
    } else {
        Some(
            Mat3::from_cols(
                mt[1].cross(mt[2]) / det,
                mt[2].cross(mt[0]) / det,
                mt[0].cross(mt[1]) / det,
            ).transpose(),
        )
    }
}

impl Intersectable for Triangle {
    fn intersection(self: &Self, ray: &Ray) -> Option<Intersection> {
        let Some(matrix) = invert(Mat3::from_cols(self.ba, self.ca, -ray.dir)) else { return None; };
        let Vec3 { x: u, y: v, z: t } = matrix * (ray.origin - self.a);
        if u < 0.0 || v < 0.0 || 1.0 < u + v || t < 0.0 {
            return None;
        }
        let normal = &self.geometry_normal;
        let shading_normal = self.shading_normal(u, v);
        let inside = ray.dir.dot(*normal) > 0.0;
        return Some(Intersection {
            t,
            geometry_normal: if inside { -*normal } else { *normal },
            shading_normal: if inside { -shading_normal } else { shading_normal },
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
    use cgmath::{num_traits::zero, vec3, InnerSpace};

    use crate::{bvh::BVH, intersections::{intersect_lights, Intersectable}, parsed_scene, ray::Ray, scene::{LightPrimitives, TrianglePrimitive}};

    use super::Triangle;

    #[test]
    fn aaa() {
        let primitive = Triangle {
            a: vec3(-4.0, -2.0, 10.0),
            ba: vec3(1.0, 6.0, 0.0),
            ca: vec3(3.0, 0.0, 0.0),
            geometry_normal: vec3(0.0, 0.0, -1.0),
            inverted_area: 0.05555555555555555,
            na: zero(),
            nb: zero(),
            nc: zero(),
        };
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

    #[test]
    fn bbb() {
        let a = vec3(0.0, 0.0, 2.0);
        let b = vec3(1.0, 0.0, 2.0);
        let c = vec3(0.0, 1.0, 0.0);
        let primitive = Triangle::new_with_geometry_normals(a, b, c);
        let properties = parsed_scene::PrimitiveProperties { material: None, ior: None, position: None, rotation: None, color: None, emission: None };

        let triangle = TrianglePrimitive::new(primitive, properties);

        let ray = Ray { origin: vec3(0.1541891385674881, 0.7047585918803002, 0.5904828162393995), dir: vec3(-0.0759650747603601, -0.4459213624433466, 0.8918427248866934) };

        let intersection = triangle.intersection(&ray);
        assert!(intersection.is_none());
    }
}
