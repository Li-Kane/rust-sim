use bevy::input::mouse::AccumulatedMouseMotion;
use bevy::prelude::*;
use bevy::window::CursorGrabMode;

#[derive(States, Default, Clone, Copy, PartialEq, Eq, Hash, Debug)]
pub enum SimState {
    #[default]
    Loading,
    InGame,
    Config,
}

#[derive(Resource, Debug, Clone)]
pub struct CameraSettings {
    pub fly_speed: f32,
}

impl Default for CameraSettings {
    fn default() -> Self {
        Self { fly_speed: 4.0 }
    }
}

pub struct HandleInputPlugin;

impl Plugin for HandleInputPlugin {
    fn build(&self, app: &mut App) {
        app.init_state::<SimState>()
            .init_resource::<CameraSettings>()
            .add_systems(Startup, setup_cursor)
            .add_systems(
                Update,
                (
                    menu_screen,
                    move_camera_system.run_if(in_state(SimState::InGame)),
                ),
            )
            .add_systems(OnEnter(SimState::InGame), enter_in_game)
            .add_systems(OnEnter(SimState::Config), enter_config);
    }
}

fn setup_cursor(mut cursor_options: Single<&mut bevy::window::CursorOptions>) {
    cursor_options.grab_mode = CursorGrabMode::Locked;
    cursor_options.visible = false;
}

fn menu_screen(
    keyboard: Res<ButtonInput<KeyCode>>,
    state: Res<State<SimState>>,
    mut next_state: ResMut<NextState<SimState>>,
) {
    if keyboard.just_pressed(KeyCode::Escape) {
        match state.get() {
            SimState::InGame => next_state.set(SimState::Config),
            SimState::Config => next_state.set(SimState::InGame),
            SimState::Loading => {}
        }
    }
}

fn enter_in_game(mut cursor_options: Single<&mut bevy::window::CursorOptions>) {
    cursor_options.grab_mode = CursorGrabMode::Locked;
    cursor_options.visible = false;
}

fn enter_config(mut cursor_options: Single<&mut bevy::window::CursorOptions>) {
    cursor_options.grab_mode = CursorGrabMode::None;
    cursor_options.visible = true;
}

fn move_camera_system(
    time: Res<Time>,
    keyboard: Res<ButtonInput<KeyCode>>,
    mouse_motion: Res<AccumulatedMouseMotion>,
    camera_settings: Res<CameraSettings>,
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

        // 2. Keyboard Movement (Camera-relative directions)
        let speed = camera_settings.fly_speed * time.delta_secs();
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
