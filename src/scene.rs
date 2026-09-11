use bevy::prelude::*;
use bevy_rapier3d::prelude::*;

use crate::robot::spawn_robot;
use crate::robot::types::Parse;
use crate::robot::urdf_loader::UrdfLoader;

pub const GROUND_SIZE: f32 = 100.0;
pub const GROUND_THICKNESS: f32 = 0.2;
pub const WORLD_AXES_LENGTH: f32 = 2.0;

pub struct ScenePlugin;

impl Plugin for ScenePlugin {
    fn build(&self, app: &mut App) {
        app.add_systems(Startup, (setup_environment, setup_camera, setup_robot))
            .add_systems(Update, draw_gridlines);
    }
}

/// Spawns the ground plane and primary directional light.
pub fn setup_environment(
    mut commands: Commands,
    mut meshes: ResMut<Assets<Mesh>>,
    mut materials: ResMut<Assets<StandardMaterial>>,
) {
    // 3D Solid Ground Plane (top surface at y = 0.0)
    let ground_mesh = meshes.add(Cuboid::new(GROUND_SIZE, GROUND_THICKNESS, GROUND_SIZE));
    let ground_material = materials.add(StandardMaterial {
        base_color: Color::srgb(0.05, 0.08, 0.25),
        metallic: 0.1,
        perceptual_roughness: 0.8,
        ..default()
    });
    commands.spawn((
        Mesh3d(ground_mesh),
        MeshMaterial3d(ground_material),
        Transform::from_xyz(0.0, -GROUND_THICKNESS / 2.0, 0.0),
        RigidBody::Fixed,
        Collider::cuboid(GROUND_SIZE / 2.0, GROUND_THICKNESS / 2.0, GROUND_SIZE / 2.0),
        Friction::coefficient(1.0),
        Restitution::coefficient(0.0),
    ));

    // Primary Directional light (Key light)
    commands.spawn((
        DirectionalLight {
            shadow_maps_enabled: true,
            illuminance: 6000.0,
            ..default()
        },
        Transform::from_xyz(4.0, 8.0, 4.0).looking_at(Vec3::ZERO, Vec3::Y),
    ));
}

/// Spawns the main 3D simulation camera.
pub fn setup_camera(mut commands: Commands) {
    // Camera facing the Spot robot model
    commands.spawn((
        Camera3d::default(),
        Transform::from_xyz(1.5, 1.0, 1.8).looking_at(Vec3::new(0.0, 0.4, 0.0), Vec3::Y),
    ));
}

/// Spawns the default Spot robot model from URDF.
pub fn setup_robot(commands: Commands, asset_server: Res<AssetServer>) {
    let blueprint = UrdfLoader::parse(
        include_str!("../assets/spot.urdf").as_bytes(),
        &asset_server,
    );
    let robot_transform = Transform::from_xyz(0.0, 1.0, 0.0);
    let starting_pose: [f32; 12] = [
        0.0, 0.75, -1.50, // FL
        0.0, 0.75, -1.50, // FR
        0.0, 0.75, -1.50, // HL
        0.0, 0.75, -1.50, // HR
    ];
    spawn_robot(
        commands,
        blueprint,
        Some(robot_transform),
        Some(&starting_pose),
    );
}

/// System to draw ground plane gridlines and world origin axes using Gizmos
pub fn draw_gridlines(mut gizmos: Gizmos) {
    let half_size = 50;
    let step = 1.0;
    let grid_color = Color::srgba(0.35, 0.35, 0.35, 0.5);
    let floor_y = 0.002;

    for i in -half_size..=half_size {
        let coord = i as f32 * step;
        let limit = half_size as f32 * step;

        gizmos.line(
            Vec3::new(coord, floor_y, -limit),
            Vec3::new(coord, floor_y, limit),
            grid_color,
        );
        gizmos.line(
            Vec3::new(-limit, floor_y, coord),
            Vec3::new(limit, floor_y, coord),
            grid_color,
        );
    }

    // World X, Y, Z axes
    let origin = Vec3::new(0.0, floor_y, 0.0);
    gizmos.arrow(
        origin,
        origin + Vec3::X * WORLD_AXES_LENGTH,
        Color::srgb(1.0, 0.0, 0.0),
    );
    gizmos.arrow(
        origin,
        origin + Vec3::Y * WORLD_AXES_LENGTH,
        Color::srgb(0.0, 1.0, 0.0),
    );
    gizmos.arrow(
        origin,
        origin + Vec3::Z * WORLD_AXES_LENGTH,
        Color::srgb(0.0, 0.0, 1.0),
    );
}
