mod types;
mod urdf_loader;

use bevy::prelude::*;
use urdf_loader::load_robot;

pub struct RobotPlugin;

impl Plugin for RobotPlugin {
    fn build(&self, app: &mut App) {
        app.add_systems(Startup, load_robot);
    }
}
