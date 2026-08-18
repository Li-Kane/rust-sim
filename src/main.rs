use bevy::input::mouse::{AccumulatedMouseMotion, AccumulatedMouseScroll};
use bevy::prelude::*;
use bevy::window::CursorGrabMode;
mod gui;
mod skeleton;
use gui::SimGuiPlugin;
use skeleton::{draw_skeleton_axes, redraw_skeleton, update_skeleton_transform, Skeleton};

fn main() {
    App::new()
        .add_plugins(DefaultPlugins)
        .add_plugins((SetupWorldPlugin, HandleInputPlugin, SimGuiPlugin))
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
        app.add_systems(Startup, (setup_environment, setup_skeleton, setup_cursor))
            .add_systems(
                Update,
                (
                    update_skeleton_transform,
                    redraw_skeleton,
                    draw_skeleton_axes,
                )
                    .chain(),
            );
    }
}

fn setup_cursor(mut cursor_options: Single<&mut bevy::window::CursorOptions>) {
    cursor_options.grab_mode = CursorGrabMode::Locked;
    cursor_options.visible = false;
}

fn setup_environment(mut commands: Commands) {
    // Point light
    commands.spawn((
        PointLight {
            shadow_maps_enabled: true,
            ..default()
        },
        Transform::from_xyz(4.0, 8.0, 4.0),
    ));

    // Camera facing the model
    commands.spawn((
        Camera3d::default(),
        Transform::from_xyz(0.0, 1.8, 4.0).looking_at(Vec3::new(0.0, 1.0, 0.0), Vec3::Y),
    ));
}

fn setup_skeleton(
    mut commands: Commands,
    mut meshes: ResMut<Assets<Mesh>>,
    mut materials: ResMut<Assets<StandardMaterial>>,
) {
    let mut skeleton = Skeleton::build_two_link();
    let bone_material = materials.add(StandardMaterial {
        base_color: Color::srgb_u8(124, 144, 255),
        ..default()
    });
    let joint_material = materials.add(StandardMaterial {
        base_color: Color::srgb_u8(250, 204, 21), // Yellow
        ..default()
    });

    skeleton.spawn(&mut commands, &mut meshes, bone_material, joint_material);
    commands.insert_resource(skeleton);
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
