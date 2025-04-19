use std::collections::HashMap;

use cgmath::{num_traits::zero, vec3, vec4, InnerSpace, SquareMatrix, Vector2};

use crate::{aabb::AABB, bvh::BVH, primitives::Triangle, scene::{is_light, CameraParams, Fov, LightPrimitives, Material, Metadata, Scene, ScenePrimitives, TrianglePrimitive}, types::{Float, Mat2, Mat3, Mat4, Quat, Vec3, Vec4}};

use super::parser::{self, Camera, Model};

pub fn build_scene(model: Model, buffer_provider: impl Fn(&str) -> Vec<u8>, width: usize, height: usize, samples: usize) -> Scene {
    let nodes = convert_nodes(&model);

    let triangles = convert_model(&model, &nodes, buffer_provider);
    Scene {
        primitives: ScenePrimitives { planes: vec![], ellipsoids: BVH::new_empty(), boxes: BVH::new_empty(), triangles: BVH::new(flat_triangles(&triangles)) },
        lights: LightPrimitives { ellipsoids: BVH::new_empty(), boxes: BVH::new_empty(), triangles: BVH::new(flat_light_triangles(&triangles)) },
        ray_depth: 8, // ok or not?
        bg_color: zero(),
        samples,
        camera: extract_camera_params(&model.cameras, &nodes),
        dimensions: Vector2 { x: width, y: height },
    }
}

fn flat_triangles(triangles: &ColoredTriangles) -> Vec<TrianglePrimitive> {
    let mut res: Vec<TrianglePrimitive> = vec![];
    for (triangles, metadata) in triangles {
        instantiate(triangles, metadata, &mut res);
    }
    return res;
}

fn flat_light_triangles(triangles: &ColoredTriangles) -> Vec<TrianglePrimitive> {
    let mut res: Vec<TrianglePrimitive> = vec![];
    for (triangles, metadata) in triangles {
        if is_light(metadata) {
            instantiate(triangles, metadata, &mut res);
        }
    }
    return res;
}

fn instantiate(triangles: &Vec<Triangle>, metadata: &Metadata, res: &mut Vec<TrianglePrimitive>) {
    for triangle in triangles {
        let mut aabb = AABB::empty();
        aabb.extend(&triangle.a);
        aabb.extend(&(triangle.a + triangle.ba));
        aabb.extend(&(triangle.a + triangle.ca));

        res.push(TrianglePrimitive {
            primitive: triangle.clone(),
            metadata: metadata.clone(),
            aabb,
        });
    }
}

fn extract_camera_params(cameras: &[Camera], nodes: &[Node]) -> CameraParams {
    assert!(cameras.len() == 1 && cameras[0].r#type == "perspective" && cameras[0].perspective.is_some(), "Supported only single perspective camera");
    let yfov = Fov::Y(cameras[0].perspective.as_ref().unwrap().yfov);

    let mut node_with_camera_trs: Option<&Mat4> = None;
    for node in nodes {
        if node.camera.is_some() {
            assert!(node_with_camera_trs.is_none(), "You must specify only one a node with the camera");
            node_with_camera_trs = Some(&node.trs);
        }
    }

    let trs = node_with_camera_trs.expect("You must specify a node with the camera");

    CameraParams {
        position: trs.w.truncate(),
        right: trs.x.truncate(),
        up: trs.y.truncate(),
        forward: -trs.z.truncate(),
        fov: yfov,
    }
}

fn load_buffers(model: &Model, buffer_provider: impl Fn(&str) -> Vec<u8>) -> HashMap<&str, Vec<u8>> {
    let mut buffers = HashMap::<&str, Vec<u8>>::new();
    for buffer in &model.buffers {
        let uri = buffer.uri.as_ref().expect("expected uri for buffer");
        buffers.insert(uri, buffer_provider(uri));
    }
    return buffers;
}

#[derive(Debug)]
struct Node<'a> {
    trs: Mat4,
    children: &'a [usize],
    camera: Option<usize>,
    mesh: Option<usize>,
}

impl<'a> Node<'a> {
    fn new(node: &'a parser::Node) -> Self {
        Self {
            trs: extract_trs(node),
            children: &node.children,
            camera: node.camera,
            mesh: node.mesh,
        }
    }
}

fn extract_trs(node: &parser::Node) -> Mat4 {
    if let Some(mt) = &node.matrix {
        assert!(mt.len() == 16);
        return Mat4::from_cols(
            extract_column(&mt, 0),
            extract_column(&mt, 1),
            extract_column(&mt, 2),
            extract_column(&mt, 3),
        );
    }
    let translation = node.translation.as_ref().map(|v| extract_vec3(v)).unwrap_or(zero());
    let rotation = node.rotation.as_ref().map(|v| extract_quat(v)).unwrap_or(Quat::new(1.0, 0.0, 0.0, 0.0));
    let scale = node.scale.as_ref().map(|v| extract_vec3(v)).unwrap_or(vec3(1.0, 1.0, 1.0));

    Mat4::from_translation(translation) * Into::<Mat4>::into(rotation) * Mat4::from_nonuniform_scale(scale[0], scale[1], scale[2])
}

fn extract_column(mt: &[Float], index: usize) -> Vec4 {
    let offset = index * 4;
    vec4(mt[offset + 0], mt[offset + 1], mt[offset + 2], mt[offset + 3])
}

fn extract_vec3(vec: &[Float]) -> Vec3 {
    assert!(vec.len() == 3, "len: {}", vec.len());
    vec3(vec[0], vec[1], vec[2])
}

fn extract_vec4(vec: &[Float]) -> Vec4 {
    assert!(vec.len() == 4, "len: {}", vec.len());
    vec4(vec[0], vec[1], vec[2], vec[3])
}

fn extract_quat(vec: &[Float]) -> Quat {
    assert!(vec.len() == 4);
    Quat::new(vec[3], vec[0], vec[1], vec[2])
}

fn convert_nodes<'a>(model: &'a Model) -> Vec<Node<'a>> {
    let mut res = vec![];
    res.reserve_exact(model.nodes.len());
    for node in &model.nodes {
        res.push(Node::new(node));
    }
    propagate_trs(&model, &mut res);
    return res;
}

fn propagate_trs<'a>(model: &'a Model, nodes: &mut [Node<'a>]) {
    for scene in &model.scenes {
        for node in &scene.nodes {
            propagate_trs_nodes(*node, nodes, Mat4::identity());
        }
    }
}

fn propagate_trs_nodes<'a>(index: usize, nodes: &mut [Node<'a>], parent_to_world: Mat4) {
    let node = &mut nodes[index];
    node.trs = parent_to_world * node.trs;
    for child in node.children {
        propagate_trs_nodes(*child, nodes, nodes[index].trs.clone());
    }
}

struct Context<'a> {
    model: &'a Model,
    nodes: &'a [Node<'a>],
    buffers: HashMap<&'a str, Vec<u8>>,
}

type ColoredTriangles = Vec<(Vec<Triangle>, Metadata)>;

fn convert_model(model: &Model, nodes: &[Node], buffer_provider: impl Fn(&str) -> Vec<u8>) -> ColoredTriangles {
    let mut res: ColoredTriangles = vec![];
    let context = Context { model, nodes, buffers: load_buffers(&model, buffer_provider) };

    for root_node in &model.scenes[model.scene].nodes {
        convert_node(*root_node, &context, &mut res);
    }

    return res;
}

fn convert_node(node_index: usize, context: &Context, res: &mut ColoredTriangles) {
    let node = &context.nodes[node_index];

    if let Some(mesh) = node.mesh {
        convert_mesh(mesh, context, res, &node.trs);
    }

    for child in node.children {
        convert_node(*child, &context, res);
    }
}

fn convert_mesh(mesh_index: usize, context: &Context, res: &mut ColoredTriangles, trs: &Mat4) {
    let mesh = &context.model.meshes[mesh_index];
    for primitive in &mesh.primitives {
        convert_primitive(primitive, context, res, trs);
    }
}

fn convert_primitive(primitive: &parser::Primitive, context: &Context, res: &mut ColoredTriangles, trs: &Mat4) {
    assert!(primitive.mode == 4, "supported only triangles for primitive.mode");
    let vertices = read_vertices(&context.model.accessors[primitive.attributes.POSITION], context, trs);
    let normals = read_normals(&context.model.accessors[primitive.attributes.NORMAL.expect("empty normals")], context, trs);

    let triangles = if let Some(index_accessor) = primitive.indices {
        let indices = read_indices(&context.model.accessors[index_accessor], context);
        make_triangles_by_indices(&vertices, &normals, &indices)
    } else {
        make_triangles(&vertices, &normals)
    };

    res.push((
        triangles,
        make_metadata(primitive.material.map(|inx| &context.model.materials[inx]).unwrap_or(&parser::Material::default())),
    ));
}

fn make_metadata(material: &parser::Material) -> Metadata {
    let color = material.pbr_metallic_roughness.base_color_factor.as_ref().map(|v| extract_vec4(v)).unwrap_or(vec4(1.0, 1.0, 1.0, 1.0));
    let emission = material.emissive_factor.as_ref().map(|v| extract_vec3(v)).unwrap_or(zero());
    Metadata {
        material: if color.w < 1.0 { Material::Dielectric(1.5) } else if material.pbr_metallic_roughness.metallic_factor > 0.0 { Material::Metallic } else { Material::Diffuse },
        color: color.truncate(),
        emission: emission * material.extensions.KHR_materials_emissive_strength.emissive_strength,
    }
}

fn read_indices(accessor: &parser::Accessor, context: &Context) -> Vec<u32> {
    let mut res: Vec<u32> = vec![];
    let Some(view) = accessor.buffer_view else { return res; };

    assert!(accessor.component_type == UNSIGNED_SHORT || accessor.component_type == UNSIGNED_INT);
    assert!(accessor.r#type == "SCALAR", "indices must have SCALAR type");

    let buffer_view = &context.model.buffer_views[view];
    let buffer = &context.model.buffers[buffer_view.buffer];
    let bytes = &context.buffers[buffer.uri.as_ref().unwrap().as_str()];
    let offset = buffer_view.byte_offset + accessor.byte_offset;
    let element_size = if accessor.component_type == UNSIGNED_SHORT { 2 } else { 4 };
    let stride = buffer_view.byte_stride.unwrap_or(element_size);

    let mut pos = offset;
    res.reserve_exact(accessor.count);
    if accessor.component_type == UNSIGNED_SHORT {
        for _ in 0..accessor.count {
            let storage = [bytes[pos+0], bytes[pos+1]];
            res.push(u16::from_le_bytes(storage) as u32);
            pos += stride;
        }
    } else {
        for _ in 0..accessor.count {
            let storage = [bytes[pos+0], bytes[pos+1], bytes[pos+2], bytes[pos+3]];
            res.push(u32::from_le_bytes(storage));
            pos += stride;
        }
    }
    return res;
}

fn read_vertices(accessor: &parser::Accessor, context: &Context, trs: &Mat4) -> Vec<Vec3> {
    let mut res: Vec<Vec3> = vec![];
    let Some(view) = accessor.buffer_view else { return res; };

    assert!(accessor.component_type == FLOAT);
    assert!(accessor.r#type == "VEC3", "vertices must have VEC3 type");

    let buffer_view = &context.model.buffer_views[view];
    let buffer = &context.model.buffers[buffer_view.buffer];
    let bytes = &context.buffers[buffer.uri.as_ref().unwrap().as_str()];
    let mut offset = buffer_view.byte_offset + accessor.byte_offset;
    let element_size = 4; // sizeof FLOAT equals 4
    let elements_in_pack = 3; // vec3 has 3 components
    let stride = buffer_view.byte_stride.unwrap_or(element_size * elements_in_pack);

    for _ in 0..accessor.count {
        let pos = trs * Vec4 {
            x: read_float(&bytes, offset + element_size * 0) as Float,
            y: read_float(&bytes, offset + element_size * 1) as Float,
            z: read_float(&bytes, offset + element_size * 2) as Float,
            w: 1.0,
        };
        assert!(pos.w == 1.0);
        res.push(pos.truncate());

        offset += stride;
    }
    return res;
}

fn read_normals(accessor: &parser::Accessor, context: &Context, trs: &Mat4) -> Vec<Vec3> {
    let mut res: Vec<Vec3> = vec![];
    let Some(view) = accessor.buffer_view else { return res; };

    assert!(accessor.component_type == FLOAT);
    assert!(accessor.r#type == "VEC3", "vertices must have VEC3 type");

    let buffer_view = &context.model.buffer_views[view];
    let buffer = &context.model.buffers[buffer_view.buffer];
    let bytes = &context.buffers[buffer.uri.as_ref().unwrap().as_str()];
    let mut offset = buffer_view.byte_offset + accessor.byte_offset;
    let element_size = 4; // sizeof FLOAT equals 4
    let elements_in_pack = 3; // vec3 has 3 components
    let stride = buffer_view.byte_stride.unwrap_or(element_size * elements_in_pack);

    let rs = cof(&mat4_to_mat3(trs));

    for _ in 0..accessor.count {
        let pos = rs * Vec3 {
            x: read_float(&bytes, offset + element_size * 0) as Float,
            y: read_float(&bytes, offset + element_size * 1) as Float,
            z: read_float(&bytes, offset + element_size * 2) as Float,
        };
        res.push(pos.normalize());

        offset += stride;
    }
    return res;
}

fn make_triangles(vertices: &[Vec3], normals: &[Vec3]) -> Vec<Triangle> {
    assert_eq!(vertices.len(), normals.len());
    assert_eq!(vertices.len() % 3, 0);
    let mut triangles = vec![];
    for i in (0..vertices.len()).step_by(3) {
        triangles.push(Triangle::new_with_smooth_normal(
            vertices[i+0], vertices[i+1], vertices[i+2],
            normals[i+0], normals[i+1], normals[i+2]),
        );
    }
    return triangles;
}

fn make_triangles_by_indices(vertices: &[Vec3], normals: &[Vec3], indices: &[u32]) -> Vec<Triangle> {
    assert_eq!(vertices.len(), normals.len());
    assert_eq!(indices.len() % 3, 0);
    let mut triangles = vec![];
    for i in (0..indices.len()).step_by(3) {
        let a = indices[i+0] as usize;
        let b = indices[i+1] as usize;
        let c = indices[i+2] as usize;
        triangles.push(Triangle::new_with_smooth_normal(
            vertices[a], vertices[b], vertices[c],
            normals[a], normals[b], normals[c]),
        );
    }
    return triangles;
}

fn read_float(bytes: &[u8], pos: usize) -> f32 {
    let storage = [bytes[pos+0], bytes[pos+1], bytes[pos+2], bytes[pos+3]];
    f32::from_le_bytes(storage)
}

fn mat4_to_mat3(mat4: &Mat4) -> Mat3 {
    Mat3::from_cols(mat4.x.truncate(), mat4.y.truncate(), mat4.z.truncate())
}

fn cof(m: &Mat3) -> Mat3 {
    let cofactor = |col: usize, row: usize| {
        let other_indices = |index| match index {
            0 => (1, 2),
            1 => (0, 2),
            2 => (0, 1),
            _ => panic!(),
        };
        let (left_col, right_col) = other_indices(col);
        let (top_row, bottom_row) = other_indices(row);
        let det = Mat2::new(
            m[left_col][top_row], m[left_col][bottom_row],
            m[right_col][top_row], m[right_col][bottom_row],
        ).determinant();
        return if (col + row) & 1 == 1 { -det } else { det };
    };
    Mat3::new(
        cofactor(0, 0), cofactor(0, 1), cofactor(0, 2),
        cofactor(1, 0), cofactor(1, 1), cofactor(1, 2),
        cofactor(2, 0), cofactor(2, 1), cofactor(2, 2),
    )
}

const UNSIGNED_SHORT: u32 = 5123;
const UNSIGNED_INT: u32 = 5125;
const FLOAT: u32 = 5126;

impl Default for parser::Material {
    fn default() -> Self {
        Self { extensions: Default::default(), pbr_metallic_roughness: Default::default(), emissive_factor: Default::default() }
    }
}

#[cfg(test)]
mod test {
    use cgmath::{assert_abs_diff_eq, Deg, InnerSpace, Matrix, SquareMatrix};

    use crate::types::{Mat3, Vec3};

    use super::cof;

    #[test]
    fn aaa() {
        let scales = Mat3::new(2.0, 0.0, 0.0, 0.0, 3.0, 0.0, 0.0, 0.0, 4.0);
        let rot1 = Mat3::from_angle_x(Deg(10.0));
        let rot2 = Mat3::from_angle_y(Deg(20.0));
        let rot3 = Mat3::from_angle_z(Deg(30.0));
        let mt = rot1 * rot2 * rot3 * scales;

        let cof = cof(&mt);
        let inverted_transposed = mt.transpose().invert().unwrap();

        let n1 = Vec3::new(1.0, 2.0, 3.0).normalize();
        let n2 = Vec3::new(-1.0, 2.0, 3.0).normalize();
        let n3 = Vec3::new(-1.0, -2.0, 1.0).normalize();

        assert_abs_diff_eq!((cof * n1).normalize(), (inverted_transposed * n1).normalize());
        assert_abs_diff_eq!((cof * n2).normalize(), (inverted_transposed * n2).normalize());
        assert_abs_diff_eq!((cof * n3).normalize(), (inverted_transposed * n3).normalize());
    }

}
