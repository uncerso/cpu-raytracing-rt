use std::ops::Range;

use crate::{aabb::{HasAABB, AABB}, intersectable_aabb::IntersectableAABB, intersections::{Intersectable, Intersection, Intersections}, ray::Ray, types::{Float, Vec3}};

#[derive(Debug)]
pub struct BVH<T: Intersectable + HasAABB> {
    primitives: Vec<T>,
    nodes: Vec<Node>,
}

impl<T: Intersectable + HasAABB> BVH<T> {
    pub fn new(primitives: Vec<T>) -> Self {
        let mut res = Self { primitives, nodes: vec![] };
        build_nodes(&mut res.primitives, &mut res.nodes, 0);
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

fn build_nodes<'a, T: HasAABB>(primitives: &'a mut [T], nodes: &mut Vec<Node>, global_offset: usize) -> usize {
    let aabb = compute_aabb(primitives);
    if primitives.len() <= 4 {
        nodes.push(Node::make_leaf(&aabb, range_from(global_offset, primitives.len())));
        return nodes.len() - 1;
    }

    let range = aabb.max - aabb.min;
    let key: fn(&Vec3) -> Float = if range.x > range.y && range.x > range.z {
        |a| a.x
    } else if range.y > range.x && range.y > range.z {
        |a| a.y
    } else {
        |a| a.z
    };

    let midpoint = |a: &AABB| (key(&a.min) + key(&a.max)) / 2.0;
    primitives.sort_unstable_by(|a, b| midpoint(a.aabb()).total_cmp(&midpoint(b.aabb())));

    let range_midpoint = midpoint(&aabb);

    let first_bucket_size = primitives.partition_point(|a| midpoint(a.aabb()) < range_midpoint);
    if first_bucket_size == 0 || first_bucket_size == primitives.len() {
        nodes.push(Node::make_leaf(&aabb, range_from(global_offset, primitives.len())));
        return nodes.len() - 1;
    }

    let node_index = nodes.len();
    nodes.push(Node::make_leaf(&aabb, 0..0));

    let (left_primitives, right_primitives) = primitives.split_at_mut(first_bucket_size);
    let left_children_index = build_nodes(left_primitives, nodes, global_offset);
    let right_children_index = build_nodes(right_primitives, nodes, global_offset + left_primitives.len());
    nodes[node_index].left_child = left_children_index;
    nodes[node_index].right_child = right_children_index;
    return node_index;
}

fn range_from(offset: usize, size: usize) -> Range<usize> {
    return offset..(offset + size);
}

impl Node {
    fn make_leaf(aabb: &AABB, primitive_indices: Range<usize>) -> Self {
        Self { aabb: IntersectableAABB::new(&aabb), left_child: usize::MAX, right_child: usize::MAX, primitive_indices }
    }

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
