use bevy::asset::RenderAssetUsages;
use bevy::prelude::*;
use bevy::render::mesh::{Indices, Mesh, PrimitiveTopology};

/// Embedded URDF and OBJ asset bytes for zero-friction cross-platform & WASM support
pub const URDF_CONTENT: &str = include_str!("../assets/spot_simple.urdf");
pub const BODY_OBJ: &[u8] = include_bytes!("../assets/meshes/body.obj");
pub const HIP_OBJ: &[u8] = include_bytes!("../assets/meshes/hip.obj");
pub const ULEG_OBJ: &[u8] = include_bytes!("../assets/meshes/uleg.obj");
pub const LLEG_OBJ: &[u8] = include_bytes!("../assets/meshes/lleg.obj");

/// Helper to parse OBJ mesh bytes into a Bevy Mesh
pub fn parse_obj_mesh(bytes: &[u8]) -> Result<Mesh, String> {
    let mut cursor = std::io::Cursor::new(bytes);
    let (models, _) = tobj::load_obj_buf(
        &mut cursor,
        &tobj::LoadOptions {
            single_index: true,
            triangulate: true,
            ..Default::default()
        },
        |_| Err(tobj::LoadError::GenericFailure),
    )
    .map_err(|e| format!("Failed to parse OBJ data: {e:?}"))?;

    let mut positions: Vec<[f32; 3]> = Vec::new();
    let mut normals: Vec<[f32; 3]> = Vec::new();
    let mut uvs: Vec<[f32; 2]> = Vec::new();
    let mut indices: Vec<u32> = Vec::new();

    let mut index_offset: u32 = 0;
    for model in models {
        let mesh = model.mesh;
        for chunk in mesh.positions.chunks_exact(3) {
            positions.push([chunk[0], chunk[1], chunk[2]]);
        }
        for chunk in mesh.normals.chunks_exact(3) {
            normals.push([chunk[0], chunk[1], chunk[2]]);
        }
        for chunk in mesh.texcoords.chunks_exact(2) {
            uvs.push([chunk[0], chunk[1]]);
        }
        for idx in mesh.indices {
            indices.push(idx + index_offset);
        }
        index_offset = positions.len() as u32;
    }

    let mut bevy_mesh = Mesh::new(
        PrimitiveTopology::TriangleList,
        RenderAssetUsages::default(),
    );
    bevy_mesh.insert_attribute(Mesh::ATTRIBUTE_POSITION, positions);
    if !normals.is_empty() {
        bevy_mesh.insert_attribute(Mesh::ATTRIBUTE_NORMAL, normals);
    }
    if !uvs.is_empty() {
        bevy_mesh.insert_attribute(Mesh::ATTRIBUTE_UV_0, uvs);
    }
    bevy_mesh.insert_indices(Indices::U32(indices));
    Ok(bevy_mesh)
}
