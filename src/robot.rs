#![allow(dead_code)]
use bevy::asset::RenderAssetUsages;
use bevy::prelude::*;
use bevy::render::mesh::{Indices, Mesh, PrimitiveTopology};
use std::collections::HashMap;

/// Embedded URDF and OBJ asset bytes for zero-friction cross-platform & WASM support
pub const URDF_CONTENT: &str = include_str!("assets/spot_simple.urdf");
pub const BODY_OBJ: &[u8] = include_bytes!("assets/meshes/base_simple/visual/body.obj");
pub const HIP_OBJ: &[u8] = include_bytes!("assets/meshes/base_simple/visual/hip.obj");
pub const ULEG_OBJ: &[u8] = include_bytes!("assets/meshes/base_simple/visual/uleg.obj");
pub const LLEG_OBJ: &[u8] = include_bytes!("assets/meshes/base_simple/visual/lleg.obj");

/// Default standing joint positions for Spot (in radians)
pub const DEFAULT_JOINT_ANGLES: [(&str, f32); 12] = [
    ("fl_hx", 0.1),
    ("fl_hy", 0.9),
    ("fl_kn", -1.5),
    ("fr_hx", -0.1),
    ("fr_hy", 0.9),
    ("fr_kn", -1.5),
    ("hl_hx", 0.1),
    ("hl_hy", 1.1),
    ("hl_kn", -1.5),
    ("hr_hx", -0.1),
    ("hr_hy", 1.1),
    ("hr_kn", -1.5),
];

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

/// A revolute or fixed joint in the robot tree
#[derive(Debug, Clone)]
pub struct RobotJoint {
    pub name: String,
    pub parent_link: usize,
    pub child_link: usize,
    pub origin_transform: Transform,
    pub axis: Vec3,
    pub angle: f32,         // in radians
    pub default_angle: f32, // in radians
    pub lower_limit: f32,   // in radians
    pub upper_limit: f32,   // in radians
}

/// A rigid link in the robot tree
#[derive(Debug, Clone)]
pub struct RobotLink {
    pub name: String,
    pub parent_joint: Option<usize>,
    pub child_joints: Vec<usize>,
    pub visual_origin: Transform,
    pub mesh_name: Option<String>,
    pub color: Color,
    pub world_transform: Transform,
}

/// Marker component for link entities
#[derive(Component, Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub struct LinkEntityIndex(pub usize);

/// Complete Spot Robot Model Resource
#[derive(Debug, Clone, Resource)]
pub struct RobotModel {
    pub name: String,
    pub links: Vec<RobotLink>,
    pub joints: Vec<RobotJoint>,
    pub root_link: usize,
    pub root_transform: Transform,
    pub link_name_to_idx: HashMap<String, usize>,
    pub joint_name_to_idx: HashMap<String, usize>,
}

impl RobotModel {
    /// Resets all 12 joint angles to Spot's default standing pose
    pub fn reset_to_default_pose(&mut self) {
        for (name, angle) in DEFAULT_JOINT_ANGLES {
            if let Some(&idx) = self.joint_name_to_idx.get(name) {
                self.joints[idx].angle = angle;
            }
        }
        self.update_kinematics();
    }

    /// Computes hierarchical forward kinematics for all links
    pub fn update_kinematics(&mut self) {
        // Root link takes the base model transform
        self.links[self.root_link].world_transform = self.root_transform;

        // Propagate forward through joints
        let mut queue = vec![self.root_link];
        while let Some(parent_link_idx) = queue.pop() {
            let parent_world = self.links[parent_link_idx].world_transform;
            let child_joints = self.links[parent_link_idx].child_joints.clone();

            for joint_idx in child_joints {
                let joint = &self.joints[joint_idx];
                let child_link_idx = joint.child_link;

                // Local joint transform = origin * rotation_around_axis
                let joint_rotation = Quat::from_axis_angle(joint.axis, joint.angle);
                let joint_local = Transform {
                    translation: joint.origin_transform.translation,
                    rotation: joint.origin_transform.rotation * joint_rotation,
                    scale: joint.origin_transform.scale,
                };

                let child_world = parent_world * joint_local;
                self.links[child_link_idx].world_transform = child_world;

                queue.push(child_link_idx);
            }
        }
    }
}

/// Builds the Spot Robot model by parsing the URDF XML
pub fn load_spot_model() -> Result<RobotModel, String> {
    let robot = urdf_rs::read_from_string(URDF_CONTENT)
        .map_err(|e| format!("Failed to parse URDF: {e:?}"))?;

    let mut links = Vec::new();
    let mut link_name_to_idx = HashMap::new();

    // 1. Collect Links
    for (i, link) in robot.links.iter().enumerate() {
        link_name_to_idx.insert(link.name.clone(), i);

        let mut visual_origin = Transform::IDENTITY;
        let mut mesh_name = None;
        let mut color = Color::srgb(0.8, 0.8, 0.8);

        if let Some(visual) = link.visual.first() {
            let xyz = visual.origin.xyz;
            let rpy = visual.origin.rpy;
            let rot = Quat::from_euler(EulerRot::ZYX, rpy[2] as f32, rpy[1] as f32, rpy[0] as f32);
            visual_origin = Transform {
                translation: Vec3::new(xyz[0] as f32, xyz[1] as f32, xyz[2] as f32),
                rotation: rot,
                scale: Vec3::ONE,
            };

            if let urdf_rs::Geometry::Mesh { filename, .. } = &visual.geometry {
                if filename.contains("body.obj") {
                    mesh_name = Some("body.obj".to_string());
                } else if filename.contains("hip.obj") {
                    mesh_name = Some("hip.obj".to_string());
                } else if filename.contains("uleg.obj") {
                    mesh_name = Some("uleg.obj".to_string());
                } else if filename.contains("lleg.obj") {
                    mesh_name = Some("lleg.obj".to_string());
                }
            }

            if let Some(mat) = &visual.material {
                if let Some(col) = &mat.color {
                    let rgba = col.rgba;
                    color = Color::srgba(
                        rgba[0] as f32,
                        rgba[1] as f32,
                        rgba[2] as f32,
                        rgba[3] as f32,
                    );
                }
            }
        }

        links.push(RobotLink {
            name: link.name.clone(),
            parent_joint: None,
            child_joints: Vec::new(),
            visual_origin,
            mesh_name,
            color,
            world_transform: Transform::IDENTITY,
        });
    }

    let mut joints = Vec::new();
    let mut joint_name_to_idx = HashMap::new();

    // 2. Collect Joints
    for joint in &robot.joints {
        let parent_link = *link_name_to_idx
            .get(&joint.parent.link)
            .ok_or_else(|| format!("Parent link {} not found", joint.parent.link))?;
        let child_link = *link_name_to_idx
            .get(&joint.child.link)
            .ok_or_else(|| format!("Child link {} not found", joint.child.link))?;

        let xyz = joint.origin.xyz;
        let rpy = joint.origin.rpy;
        let rot = Quat::from_euler(EulerRot::ZYX, rpy[2] as f32, rpy[1] as f32, rpy[0] as f32);
        let origin_transform = Transform {
            translation: Vec3::new(xyz[0] as f32, xyz[1] as f32, xyz[2] as f32),
            rotation: rot,
            scale: Vec3::ONE,
        };

        let axis = Vec3::new(
            joint.axis.xyz[0] as f32,
            joint.axis.xyz[1] as f32,
            joint.axis.xyz[2] as f32,
        )
        .normalize_or_zero();

        let lower_limit = joint.limit.lower as f32;
        let upper_limit = joint.limit.upper as f32;

        let joint_idx = joints.len();
        joint_name_to_idx.insert(joint.name.clone(), joint_idx);

        joints.push(RobotJoint {
            name: joint.name.clone(),
            parent_link,
            child_link,
            origin_transform,
            axis,
            angle: 0.0,
            default_angle: 0.0,
            lower_limit,
            upper_limit,
        });

        links[parent_link].child_joints.push(joint_idx);
        links[child_link].parent_joint = Some(joint_idx);
    }

    let root_link = *link_name_to_idx
        .get("root")
        .or_else(|| link_name_to_idx.get("body"))
        .unwrap_or(&0);

    // Coordinate conversion from URDF (Z-up, X-fwd, Y-left) to Bevy (Y-up, -Z-fwd, +X-right)
    // Rotation matrix: X_bevy = -Y_urdf, Y_bevy = Z_urdf, Z_bevy = -X_urdf
    let urdf_to_bevy_rot = Quat::from_mat3(&Mat3::from_cols(
        Vec3::new(0.0, 0.0, -1.0),
        Vec3::new(-1.0, 0.0, 0.0),
        Vec3::new(0.0, 1.0, 0.0),
    ));

    // Place robot body at height 0.55m so its feet touch the grid y=0 plane
    let root_transform = Transform {
        translation: Vec3::new(0.0, 0.55, 0.0),
        rotation: urdf_to_bevy_rot,
        scale: Vec3::ONE,
    };

    let mut model = RobotModel {
        name: robot.name,
        links,
        joints,
        root_link,
        root_transform,
        link_name_to_idx,
        joint_name_to_idx,
    };

    // Initialize default standing pose
    model.reset_to_default_pose();

    Ok(model)
}

/// Spawns the Spot robot into the Bevy ECS scene
pub fn setup_robot(
    mut commands: Commands,
    mut meshes: ResMut<Assets<Mesh>>,
    mut materials: ResMut<Assets<StandardMaterial>>,
) {
    let model = match load_spot_model() {
        Ok(m) => m,
        Err(err) => {
            error!("Failed to load Spot URDF model: {err}");
            return;
        }
    };

    // Load and cache the 4 OBJ visual meshes
    let body_mesh = parse_obj_mesh(BODY_OBJ)
        .map(|m| meshes.add(m))
        .expect("body.obj parse failed");
    let hip_mesh = parse_obj_mesh(HIP_OBJ)
        .map(|m| meshes.add(m))
        .expect("hip.obj parse failed");
    let uleg_mesh = parse_obj_mesh(ULEG_OBJ)
        .map(|m| meshes.add(m))
        .expect("uleg.obj parse failed");
    let lleg_mesh = parse_obj_mesh(LLEG_OBJ)
        .map(|m| meshes.add(m))
        .expect("lleg.obj parse failed");

    // Spawn an entity for each link that has a visual mesh
    for (link_idx, link) in model.links.iter().enumerate() {
        let mesh_handle = match link.mesh_name.as_deref() {
            Some("body.obj") => Some(body_mesh.clone()),
            Some("hip.obj") => Some(hip_mesh.clone()),
            Some("uleg.obj") => Some(uleg_mesh.clone()),
            Some("lleg.obj") => Some(lleg_mesh.clone()),
            _ => None,
        };

        if let Some(mesh) = mesh_handle {
            let material = materials.add(StandardMaterial {
                base_color: link.color,
                metallic: 0.2,
                perceptual_roughness: 0.4,
                ..default()
            });

            // Compute final visual transform: link_world * visual_origin
            let visual_world_transform = link.world_transform * link.visual_origin;

            commands.spawn((
                LinkEntityIndex(link_idx),
                Name::new(link.name.clone()),
                Mesh3d(mesh),
                MeshMaterial3d(material),
                visual_world_transform,
            ));
        }
    }

    commands.insert_resource(model);
}

/// System to synchronize Bevy entity transforms with the robot's kinematic state
pub fn update_robot_transforms(
    mut robot: ResMut<RobotModel>,
    mut query: Query<(&LinkEntityIndex, &mut Transform)>,
) {
    robot.update_kinematics();

    for (link_idx, mut transform) in &mut query {
        let link = &robot.links[link_idx.0];
        *transform = link.world_transform * link.visual_origin;
    }
}

pub struct RobotPlugin;

impl Plugin for RobotPlugin {
    fn build(&self, app: &mut App) {
        app.add_systems(Startup, setup_robot)
            .add_systems(Update, update_robot_transforms);
    }
}
