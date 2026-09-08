use bevy::prelude::*;

mod gui;
mod input;
mod physics;
mod robot;
mod scene;

pub use gui::SimulationSpeed;
pub use input::{CameraSettings, SimState};

use gui::SimGuiPlugin;
use input::HandleInputPlugin;
use physics::PhysicsPlugin;
use scene::ScenePlugin;

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
        .add_plugins((ScenePlugin, PhysicsPlugin, HandleInputPlugin, SimGuiPlugin))
        .run();
}
