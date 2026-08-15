use bevy::input::mouse::{AccumulatedMouseMotion, AccumulatedMouseScroll};
use bevy::prelude::*;
use bevy::window::CursorGrabMode;

fn main() {
    App::new()
        .add_plugins(DefaultPlugins)
        .add_plugins((SetupWorldPlugin, HandleInputPlugin))
        .run();
}

#[derive(States, Default, Clone, Copy, PartialEq, Eq, Hash, Debug)]
pub enum SimState {
    #[default]
    InGame,
    Paused,
}

// ----------------------------------------------------------------------------
// SetupWorld Plugin
// ----------------------------------------------------------------------------
pub struct SetupWorldPlugin;

impl Plugin for SetupWorldPlugin {
    fn build(&self, app: &mut App) {
        app.add_systems(Startup, (scene.spawn(), setup_cursor));
    }
}

fn setup_cursor(mut cursor_options: Single<&mut bevy::window::CursorOptions>) {
    cursor_options.grab_mode = CursorGrabMode::Locked;
    cursor_options.visible = false;
}

/// set up a simple 3D scene
fn scene() -> impl SceneList {
    bsn_list! [
        (
            #Cube
            Mesh3d(asset_value(Cuboid::new(1.0, 1.0, 1.0)))
            MeshMaterial3d::<StandardMaterial>(asset_value(Color::srgb_u8(124, 144, 255)))
            Transform::from_xyz(0.0, 0.5, 0.0)
        ),
        (
            PointLight {
                shadow_maps_enabled: true,
            }
            Transform::from_xyz(4.0, 8.0, 4.0)
        ),
        (
            Camera3d
            template_value(Transform::from_xyz(-2.5, 4.5, 9.0).looking_at(Vec3::ZERO, Vec3::Y))
        )
    ]
}

// ----------------------------------------------------------------------------
// HandleInput Plugin
// ----------------------------------------------------------------------------
pub struct HandleInputPlugin;

impl Plugin for HandleInputPlugin {
    fn build(&self, app: &mut App) {
        app.init_state::<SimState>()
            .add_systems(
                Update,
                (
                    menu_screen,
                    move_camera_system.run_if(in_state(SimState::InGame)),
                ),
            )
            .add_systems(OnEnter(SimState::InGame), enter_in_game)
            .add_systems(OnEnter(SimState::Paused), enter_paused);
    }
}

fn menu_screen(
    keyboard: Res<ButtonInput<KeyCode>>,
    state: Res<State<SimState>>,
    mut next_state: ResMut<NextState<SimState>>,
) {
    if keyboard.just_pressed(KeyCode::Escape) {
        match state.get() {
            SimState::InGame => next_state.set(SimState::Paused),
            SimState::Paused => next_state.set(SimState::InGame),
        }
    }
}

fn enter_in_game(mut cursor_options: Single<&mut bevy::window::CursorOptions>) {
    cursor_options.grab_mode = CursorGrabMode::Locked;
    cursor_options.visible = false;
}

fn enter_paused(mut cursor_options: Single<&mut bevy::window::CursorOptions>) {
    cursor_options.grab_mode = CursorGrabMode::None;
    cursor_options.visible = true;
}

fn move_camera_system(
    time: Res<Time>,
    keyboard: Res<ButtonInput<KeyCode>>,
    mouse_motion: Res<AccumulatedMouseMotion>,
    mouse_scroll: Res<AccumulatedMouseScroll>,
    mut query: Query<&mut Transform, With<Camera3d>>,
) {
    for mut transform in &mut query {
        // 1. Mouse Look
        let sensitivity = 0.003;
        let delta = mouse_motion.delta;

        if delta != Vec2::ZERO {
            let yaw = Quat::from_rotation_y(-delta.x * sensitivity);
            let pitch = Quat::from_rotation_x(-delta.y * sensitivity);
            transform.rotation = yaw * transform.rotation * pitch;
        }

        // 2. Mouse Scroll Zoom
        let scroll_y = mouse_scroll.delta.y;
        if scroll_y != 0.0 {
            let zoom_speed = 1.5;
            let forward = transform.forward();
            transform.translation += *forward * scroll_y * zoom_speed;
        }

        // 3. Keyboard Movement (Camera-relative directions)
        let speed = 5.0 * time.delta_secs();
        let forward = transform.forward();
        let right = transform.right();

        if keyboard.pressed(KeyCode::KeyW) {
            transform.translation += *forward * speed;
        }
        if keyboard.pressed(KeyCode::KeyS) {
            transform.translation -= *forward * speed;
        }
        if keyboard.pressed(KeyCode::KeyA) {
            transform.translation -= *right * speed;
        }
        if keyboard.pressed(KeyCode::KeyD) {
            transform.translation += *right * speed;
        }
        if keyboard.pressed(KeyCode::Space) {
            transform.translation += Vec3::Y * speed;
        }
        if keyboard.pressed(KeyCode::ShiftLeft) {
            transform.translation -= Vec3::Y * speed;
        }
    }
}
