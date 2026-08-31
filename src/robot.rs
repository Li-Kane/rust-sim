pub mod kinematics;
pub mod loader;
pub mod mesh;
pub mod types;

pub use loader::load_spot_model;
pub use mesh::*;
pub use types::*;

use bevy::prelude::*;

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
