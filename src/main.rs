use bevy::asset::AssetMetaCheck;
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
use robot::RobotPlugin;
use scene::ScenePlugin;

fn main() {
    App::new()
        .add_plugins(
            DefaultPlugins
                .set(WindowPlugin {
                    primary_window: Some(Window {
                        title: "Spot Robot Dog Sim".into(),
                        canvas: Some("#bevy-canvas".into()),
                        fit_canvas_to_parent: true,
                        prevent_default_event_handling: false,
                        ..default()
                    }),
                    ..default()
                })
                .set(AssetPlugin {
                    meta_check: AssetMetaCheck::Never,
                    ..default()
                }),
        )
        .add_plugins((
            ScenePlugin,
            PhysicsPlugin,
            HandleInputPlugin,
            SimGuiPlugin,
            RobotPlugin,
        ))
        .run();
}
