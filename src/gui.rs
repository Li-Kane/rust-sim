pub mod camera_controls;
pub mod robot_controls;
pub mod sim_controls;

use bevy::prelude::*;
use bevy_egui::{EguiContexts, EguiPlugin, EguiPrimaryContextPass, egui};
use bevy_rapier3d::prelude::*;

use crate::SimState;
use crate::input::CameraSettings;
use crate::robot::types::{Robot, RobotJoints, RobotName, RobotPose, SpawnPose};
use camera_controls::render_camera_controls;
use robot_controls::render_robot_controls;
use sim_controls::render_sim_controls;

#[derive(Resource, Debug, Clone)]
pub struct SimulationSpeed {
    pub speed: f32,
}

impl Default for SimulationSpeed {
    fn default() -> Self {
        Self { speed: 1.0 }
    }
}

pub struct SimGuiPlugin;

impl Plugin for SimGuiPlugin {
    fn build(&self, app: &mut App) {
        app.init_resource::<SimulationSpeed>()
            .add_plugins(EguiPlugin::default())
            .add_systems(
                EguiPrimaryContextPass,
                sim_ui.run_if(in_state(SimState::Config)),
            )
            .add_systems(
                Update,
                sync_simulation_speed.run_if(resource_changed::<SimulationSpeed>),
            );
    }
}

pub fn sim_ui(
    mut contexts: EguiContexts,
    camera_settings: Option<ResMut<CameraSettings>>,
    mut sim_speed: Option<ResMut<SimulationSpeed>>,
    mut robot_query: Query<(&Robot, &RobotName, &RobotJoints, &mut RobotPose, &SpawnPose)>,
    mut link_query: Query<(&mut Transform, Option<&mut Velocity>)>,
    mut rapier_context: Query<&mut RapierContextJoints>,
) {
    let Ok(ctx) = contexts.ctx_mut() else { return };

    egui::Window::new("Sim Configuration")
        .default_open(true)
        .collapsible(true)
        .resizable(true)
        .default_size([400.0, 560.0])
        .show(ctx, |ui| {
            egui::ScrollArea::vertical().show(ui, |ui| {
                ui.label("Press ESC to resume physics & camera controls.");
                ui.separator();

                if let Some(ref mut speed) = sim_speed {
                    render_sim_controls(ui, speed);
                    ui.add_space(8.0);
                    ui.separator();
                }

                if let Some(mut cam) = camera_settings {
                    render_camera_controls(ui, &mut cam);
                    ui.add_space(8.0);
                    ui.separator();
                }

                let mut rapier_ctx = rapier_context.iter_mut().next();

                for (robot, robot_name, robot_joints, mut robot_pose, spawn_pose) in
                    robot_query.iter_mut()
                {
                    let rapier_context_joints = rapier_ctx.as_deref_mut();

                    render_robot_controls(
                        ui,
                        robot,
                        robot_name,
                        robot_joints,
                        robot_pose.as_mut(),
                        spawn_pose,
                        &mut link_query,
                        rapier_context_joints,
                    );
                    ui.add_space(8.0);
                    ui.separator();
                }
            });
        });
}

pub fn sync_simulation_speed(
    sim_speed: Res<SimulationSpeed>,
    mut timestep_mode: Option<ResMut<TimestepMode>>,
    mut virtual_time: Option<ResMut<Time<Virtual>>>,
) {
    let speed = sim_speed.speed;

    // update physics speed
    if let Some(mode) = timestep_mode.as_deref_mut() {
        match mode {
            TimestepMode::Fixed { dt, .. } => {
                *dt = (1.0 / 60.0) * speed;
            }
            TimestepMode::Variable { time_scale, .. } => {
                *time_scale = speed;
            }
            TimestepMode::Interpolated { time_scale, .. } => {
                *time_scale = speed;
            }
        }
    }

    // update bevy's internal time resource
    if let Some(time) = virtual_time.as_deref_mut() {
        time.set_relative_speed(speed);
    }
}
