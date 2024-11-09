use std::{cmp::Ordering, ops::Range};

use crate::{aabb::{HasAABB, AABB}, intersectable_aabb::IntersectableAABB, intersections::{Intersectable, Intersection, Intersections}, ray::Ray, types::{Float, Vec3}};

#[derive(Debug)]
pub struct BVH<T: Intersectable + HasAABB> {
    primitives: Vec<T>,
    nodes: Vec<Node>,
}

impl<T: Intersectable + HasAABB> BVH<T> {
    pub fn new(primitives: Vec<T>) -> Self {
        let mut res = Self { primitives, nodes: vec![] };
        let mut splits_builder = AABBSplitsBuilder::new(std::cmp::max(res.primitives.len(), 1) - 1);
        build_nodes(&mut res.primitives, &mut res.nodes, 0, &mut splits_builder);
        return res;
    }

    pub fn primitives(&self) -> &[T] {
        &self.primitives
    }

    pub fn intersection(self: &Self, ray: &Ray) -> Option<(Intersection, &T)> {
        let mut best_result = None;
        if !self.primitives.is_empty() {
            let root = &self.nodes[0];
            if root.aabb.intersects(ray).is_some() {
                root.intersection(ray, self, &mut best_result);
            }
        }
        return best_result;
    }

    pub fn intersections(self: &Self, ray: &Ray, callback: &mut impl FnMut(Intersection, &T)) {
        if !self.primitives.is_empty() {
            let root = &self.nodes[0];
            if root.aabb.intersects(ray).is_some() {
                root.intersections(ray, self, callback);
            }
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

#[derive(Debug, Clone, Copy)]
enum SubdivisionType {
    SameNode, X, Y, Z,
}

struct Subdivision {
    first_bucket_size: usize,
    best_score: Float,
    subdivision_type: SubdivisionType
}

fn build_nodes<'a, T: HasAABB>(primitives: &'a mut [T], nodes: &mut Vec<Node>, global_offset: usize, splits_builder: &mut AABBSplitsBuilder) -> usize {
    let aabb = compute_aabb(primitives);
    if primitives.len() <= 4 {
        nodes.push(Node::make_leaf(&aabb, range_from(global_offset, primitives.len())));
        return nodes.len() - 1;
    }

    let mut best_subdivision = Subdivision {
        first_bucket_size: primitives.len(),
        best_score: aabb_score(&aabb) * primitives.len() as Float,
        subdivision_type: SubdivisionType::SameNode,
    };

    subdivision_score(primitives, |a| a.x, splits_builder, &mut best_subdivision, SubdivisionType::X);
    subdivision_score(primitives, |a| a.y, splits_builder, &mut best_subdivision, SubdivisionType::Y);
    subdivision_score(primitives, |a| a.z, splits_builder, &mut best_subdivision, SubdivisionType::Z);

    let key = match best_subdivision.subdivision_type {
        SubdivisionType::SameNode => {
            nodes.push(Node::make_leaf(&aabb, range_from(global_offset, primitives.len())));
            return nodes.len() - 1;
        },
        SubdivisionType::X => |a: &Vec3| a.x,
        SubdivisionType::Y => |a: &Vec3| a.y,
        SubdivisionType::Z => |a: &Vec3| a.z,
    };

    primitives.sort_unstable_by(midpoint_comparator(key));

    let node_index = nodes.len();
    nodes.push(Node::make_leaf(&aabb, 0..0));

    let (left_primitives, right_primitives) = primitives.split_at_mut(best_subdivision.first_bucket_size);
    let left_children_index = build_nodes(left_primitives, nodes, global_offset, splits_builder);
    let right_children_index = build_nodes(right_primitives, nodes, global_offset + left_primitives.len(), splits_builder);
    nodes[node_index].left_child = left_children_index;
    nodes[node_index].right_child = right_children_index;
    return node_index;
}

fn aabb_score(aabb: &AABB) -> Float {
    let s = aabb.max - aabb.min;
    return s.x * s.y + s.x * s.z + s.y * s.z;
}

fn subdivision_score<T: HasAABB>(primitives: &mut [T], key: fn(&Vec3) -> Float, splits_builder: &mut AABBSplitsBuilder, best_subdivision: &mut Subdivision, subdivision_type: SubdivisionType) {
    primitives.sort_unstable_by(midpoint_comparator(key));
    let (ltor, rtol) = splits_builder.make_splits(primitives);
    for i in 0..primitives.len()-1 {
        let left_cnt = i + 1;
        let right_cnt = primitives.len() - left_cnt;
        let score = aabb_score(&ltor[i]) * (left_cnt as Float) + aabb_score(&rtol[rtol.len() - i - 1]) * (right_cnt as Float);
        if score < best_subdivision.best_score {
            *best_subdivision = Subdivision {
                first_bucket_size: left_cnt,
                best_score: score,
                subdivision_type,
            }
        }
    }
}

fn midpoint_comparator<T: HasAABB>(key: fn(&Vec3) -> Float) -> impl Fn(&T, &T) -> Ordering {
    let midpoint = move |a: &AABB| (key(&a.min) + key(&a.max)) / 2.0;
    move |a: &T, b: &T| midpoint(a.aabb()).total_cmp(&midpoint(b.aabb()))
}

fn range_from(offset: usize, size: usize) -> Range<usize> {
    return offset..(offset + size);
}

impl Node {
    fn make_leaf(aabb: &AABB, primitive_indices: Range<usize>) -> Self {
        Self { aabb: IntersectableAABB::new(&aabb), left_child: usize::MAX, right_child: usize::MAX, primitive_indices }
    }

    fn intersection<'a, T: Intersectable + HasAABB>(&self, ray: &Ray, bvh: &'a BVH<T>, best_result: &mut Option<(Intersection, &'a T)>) {
        for i in self.primitive_indices.clone() {
            let primitive = &bvh.primitives[i];
            let Some(intersection) = primitive.intersection(ray) else { continue; };
            update_best_intersection(intersection, primitive, best_result);
        }

        let left_intersection = if self.left_child != usize::MAX { bvh.nodes[self.left_child].aabb.intersects(ray) } else { None };
        let right_intersection = if self.right_child != usize::MAX { bvh.nodes[self.right_child].aabb.intersects(ray) } else { None };

        let best = best_result.as_ref().map(|v| v.0.t).unwrap_or(Float::INFINITY);
        let left_intersection = left_intersection.map(|v| if v < best { v } else { best }).unwrap_or(best);
        let right_intersection = right_intersection.map(|v| if v < best { v } else { best }).unwrap_or(best);

        if left_intersection < best {
            if right_intersection < best {
                if left_intersection < right_intersection {
                    bvh.nodes[self.left_child].intersection(ray, bvh, best_result);
                    let best = best_result.as_ref().map(|v| v.0.t).unwrap_or(Float::INFINITY);
                    if right_intersection < best {
                        bvh.nodes[self.right_child].intersection(ray, bvh, best_result)
                    }
                } else {
                    bvh.nodes[self.right_child].intersection(ray, bvh, best_result);
                    let best = best_result.as_ref().map(|v| v.0.t).unwrap_or(Float::INFINITY);
                    if left_intersection < best {
                        bvh.nodes[self.left_child].intersection(ray, bvh, best_result)
                    }
                }
            } else {
                bvh.nodes[self.left_child].intersection(ray, bvh, best_result);
            }
        } else if right_intersection < best {
                bvh.nodes[self.right_child].intersection(ray, bvh, best_result)
        }
    }

    fn intersections<'a, T: Intersectable + HasAABB>(&self, ray: &Ray, bvh: &'a BVH<T>, callback: &mut impl FnMut(Intersection, &T)) {
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

        if self.left_child != usize::MAX && bvh.nodes[self.left_child].aabb.intersects(ray).is_some() {
            bvh.nodes[self.left_child].intersections(ray, bvh, callback);
        }

        if self.right_child != usize::MAX && bvh.nodes[self.right_child].aabb.intersects(ray).is_some() {
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

struct AABBSplitsBuilder {
    forward: Vec<AABB>,
    backward: Vec<AABB>,
}

impl AABBSplitsBuilder {
    fn new(capacity: usize) -> Self {
        let mut forward = vec![];
        let mut backward = vec![];
        forward.reserve_exact(capacity);
        backward.reserve_exact(capacity);
        Self { forward, backward }
    }

    fn make_splits<T: HasAABB>(&mut self, elements: &[T]) -> (&[AABB], &[AABB]) {
        self.forward.clear();
        self.backward.clear();
        let mut aabb = AABB::empty();
        let cnt = elements.len() - 1;
        let forward_elements = &elements[0..cnt];
        let backward_elements = &elements[1..cnt+1];
        for element in forward_elements {
            aabb.extend_aabb(element.aabb());
            self.forward.push(aabb.clone());
        }
        aabb = AABB::empty();
        for element in backward_elements.iter().rev() {
            aabb.extend_aabb(element.aabb());
            self.backward.push(aabb.clone());
        }
        return (&self.forward, &self.backward);
    }
}
