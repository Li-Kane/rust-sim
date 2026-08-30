use bevy::prelude::*;

pub const WORLD_AXES_LENGTH: f32 = 2.0;

pub struct EnvironmentPlugin;

impl Plugin for EnvironmentPlugin {
    fn build(&self, app: &mut App) {
        app.add_systems(Startup, setup_environment)
            .add_systems(Update, draw_gridlines);
    }
}

pub fn setup_environment(mut commands: Commands) {
    // Primary Directional light (Key light)
    commands.spawn((
        DirectionalLight {
            shadow_maps_enabled: true,
            illuminance: 6000.0,
            ..default()
        },
        Transform::from_xyz(4.0, 8.0, 4.0).looking_at(Vec3::ZERO, Vec3::Y),
    ));

    // Secondary Fill light
    commands.spawn((
        DirectionalLight {
            shadow_maps_enabled: false,
            illuminance: 2000.0,
            ..default()
        },
        Transform::from_xyz(-4.0, 5.0, -4.0).looking_at(Vec3::ZERO, Vec3::Y),
    ));

    // Camera facing the Spot robot model
    commands.spawn((
        Camera3d::default(),
        Transform::from_xyz(1.5, 1.0, 1.8).looking_at(Vec3::new(0.0, 0.4, 0.0), Vec3::Y),
    ));
}

/// System to draw ground plane gridlines and world origin axes using Gizmos
pub fn draw_gridlines(mut gizmos: Gizmos) {
    let half_size = 50;
    let step = 1.0;
    let grid_color = Color::srgba(0.35, 0.35, 0.35, 0.5);

    for i in -half_size..=half_size {
        let coord = i as f32 * step;
        let limit = half_size as f32 * step;

        gizmos.line(
            Vec3::new(coord, 0.0, -limit),
            Vec3::new(coord, 0.0, limit),
            grid_color,
        );
        gizmos.line(
            Vec3::new(-limit, 0.0, coord),
            Vec3::new(limit, 0.0, coord),
            grid_color,
        );
    }

    // World X, Y, Z axes
    let origin = Vec3::ZERO;
    gizmos.arrow(origin, origin + Vec3::X * WORLD_AXES_LENGTH, Color::srgb(1.0, 0.0, 0.0));
    gizmos.arrow(origin, origin + Vec3::Y * WORLD_AXES_LENGTH, Color::srgb(0.0, 1.0, 0.0));
    gizmos.arrow(origin, origin + Vec3::Z * WORLD_AXES_LENGTH, Color::srgb(0.0, 0.0, 1.0));
}
