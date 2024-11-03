use cgmath::{Array as _, ElementWise as _, InnerSpace as _};

use crate::{intersections::Intersection, primitives::{Box, Ellipsoid, Triangle}, types::{Float, Vec3, PI}};

pub trait IntersectionProbability {
    fn intersection_probability(self: &Self, intersection: &Intersection) -> Float;
}

impl IntersectionProbability for Triangle {
    fn intersection_probability(self: &Self, _intersection: &Intersection) -> Float {
        self.inverted_area
    }
}

impl IntersectionProbability for Box {
    fn intersection_probability(self: &Self, _intersection: &Intersection) -> Float {
        1.0 / Vec3 {
            x: self.sizes.y * self.sizes.z,
            y: self.sizes.x * self.sizes.z,
            z: self.sizes.x * self.sizes.y,
        }.sum() / 8.0
    }
}

impl IntersectionProbability for Ellipsoid {
    fn intersection_probability(self: &Self, intersection: &Intersection) -> Float {
        let coef = Vec3 {
            x: self.radiuses.y * self.radiuses.z,
            y: self.radiuses.x * self.radiuses.z,
            z: self.radiuses.x * self.radiuses.y,
        }.mul_element_wise(intersection.normal);

        return 1.0 / (4.0 * PI * coef.dot(coef).sqrt());
    }
}
