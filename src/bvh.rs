use std::{ops::Range, usize};

use crate::{aabb::{HasAABB, AABB}, intersectable_aabb::IntersectableAABB, intersections::{Intersectable, Intersection, Intersections}, ray::Ray};

#[derive(Debug)]
pub struct BVH<T: Intersectable + HasAABB> {
    primitives: Vec<T>,
    nodes: Vec<Node>,
}

impl<T: Intersectable + HasAABB> BVH<T> {
    pub fn new(primitives: Vec<T>) -> Self {
        let mut res = Self { primitives, nodes: vec![] };
        build_nodes(&mut res.primitives, &mut res.nodes);
        return res;
    }

    pub fn primitives(&self) -> &[T] {
        &self.primitives
    }

    pub fn intersection(self: &Self, ray: &Ray) -> Option<(Intersection, &T)> {
        let mut best_result = None;
        if !self.primitives.is_empty() {
            self.nodes[0].intersection(ray, self, &mut best_result);
        }
        return best_result;
    }

    pub fn intersections(self: &Self, ray: &Ray, callback: &mut impl FnMut(Intersection, &T)) {
        if !self.primitives.is_empty() {
            self.nodes[0].intersections(ray, self, callback);
        }
    }
}

#[derive(Debug)]
struct Node {
    aabb: IntersectableAABB,
    left_child: usize,
    right_child: usize,
    primitive_indices: Range<usize>,
}

fn compute_aabb<'a, T: HasAABB>(vs: &'a[T]) -> AABB {
    let mut aabb = AABB::empty();
    for v in vs {
        aabb.extend_aabb(v.aabb());
    }
    return aabb;
}

fn build_nodes<'a, T: HasAABB>(primitives: &'a mut [T], nodes: &mut Vec<Node>) {
    let aabb = compute_aabb(primitives);
    let range = 0..primitives.len();
    let node = Node { aabb: IntersectableAABB::new(&aabb), left_child: usize::MAX, right_child: usize::MAX, primitive_indices: range };
    nodes.push(node);
}

impl Node {
    fn intersection<'a, T: Intersectable + HasAABB>(&self, ray: &Ray, bvh: &'a BVH<T>, best_result: &mut Option<(Intersection, &'a T)>) {
        if !self.aabb.intersects(ray) {
            return;
        }

        for i in self.primitive_indices.clone() {
            let primitive = &bvh.primitives[i];
            let Some(intersection) = primitive.intersection(ray) else { continue; };
            update_best_intersection(intersection, primitive, best_result);
        }

        if self.left_child != usize::MAX {
            bvh.nodes[self.left_child].intersection(ray, bvh, best_result);
        }

        if self.right_child != usize::MAX {
            bvh.nodes[self.right_child].intersection(ray, bvh, best_result);
        }
    }

    fn intersections<'a, T: Intersectable + HasAABB>(&self, ray: &Ray, bvh: &'a BVH<T>, callback: &mut impl FnMut(Intersection, &T)) {
        if !self.aabb.intersects(ray) {
            return;
        }

        for i in self.primitive_indices.clone() {
            let primitive = &bvh.primitives[i];
            match primitive.all_intersections(ray) {
                Intersections::None => (),
                Intersections::One(intersection) => {
                    callback(intersection, primitive);
                }
                Intersections::Two(intersection1, intersection2) => {
                    callback(intersection1, primitive);
                    callback(intersection2, primitive);
                },
            }
        }

        if self.left_child != usize::MAX {
            bvh.nodes[self.left_child].intersections(ray, bvh, callback);
        }

        if self.right_child != usize::MAX {
            bvh.nodes[self.right_child].intersections(ray, bvh, callback);
        }
    }
}

fn update_best_intersection<'a, T>(intersection: Intersection, primitive: &'a T, best_result: &mut Option<(Intersection, &'a T)>) {
    match best_result {
        Some((old_best, _)) => {
            if intersection.t < old_best.t {
                *best_result = Some((intersection, &primitive))
            }
        },
        None => *best_result = Some((intersection, &primitive)),
    }
}
