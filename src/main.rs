use bevy::prelude::*;

mod controller;
mod environment;
mod gui;
mod input;
mod physics;
mod robot;

pub use controller::JointPdController;
pub use gui::SimulationSpeed;
pub use input::{CameraSettings, SimState};

use controller::ControllerPlugin;
use environment::EnvironmentPlugin;
use gui::SimGuiPlugin;
use input::HandleInputPlugin;
use physics::PhysicsPlugin;
use robot::RobotPlugin;

fn main() {
    App::new()
        .add_plugins(DefaultPlugins.set(WindowPlugin {
            primary_window: Some(Window {
                title: "Spot Robot Dog Sim".into(),
                canvas: Some("#bevy-canvas".into()),
                fit_canvas_to_parent: true,
                prevent_default_event_handling: true,
                ..default()
            }),
            ..default()
        }))
        .add_plugins((
            EnvironmentPlugin,
            RobotPlugin,
            PhysicsPlugin,
            ControllerPlugin,
            HandleInputPlugin,
            SimGuiPlugin,
        ))
        .run();
}
