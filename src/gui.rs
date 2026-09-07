use bevy::prelude::*;
use bevy_egui::{EguiContexts, EguiPlugin, EguiPrimaryContextPass, egui};
use bevy_rapier3d::prelude::*;

use crate::SimState;
use crate::input::CameraSettings;
use crate::robot::{LinkEntityIndex, RobotBaseMarker, RobotJointIndex, RobotModel};

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
                joint_inspector_ui.run_if(in_state(SimState::Config)),
            );
    }
}

pub fn joint_inspector_ui(
    mut contexts: EguiContexts,
    robot: Option<ResMut<RobotModel>>,
    mut base_query: Query<(&mut Transform, &mut Velocity), With<RobotBaseMarker>>,
    mut link_query: Query<
        (&LinkEntityIndex, &mut Transform, &mut Velocity),
        Without<RobotBaseMarker>,
    >,
    mut joint_query: Query<(&RobotJointIndex, &mut MultibodyJoint)>,
    camera_settings: Option<ResMut<CameraSettings>>,
    mut debug_context: Option<ResMut<DebugRenderContext>>,
    mut sim_speed: Option<ResMut<SimulationSpeed>>,
    mut timestep_mode: Option<ResMut<TimestepMode>>,
    mut virtual_time: Option<ResMut<Time<Virtual>>>,
) {
    let Ok(ctx) = contexts.ctx_mut() else { return };
    let Some(mut robot) = robot else { return };

    egui::Window::new("Sim Configuration")
        .default_open(true)
        .collapsible(true)
        .resizable(true)
        .default_size([400.0, 560.0])
        .show(ctx, |ui| {
            egui::ScrollArea::vertical().show(ui, |ui| {
                ui.label("Press ESC to resume physics & camera controls.");
                ui.separator();

                ui.horizontal(|ui| {
                    if ui.button("Reset Robot (Center & Pose)").clicked() {
                        robot.reset_full();

                        if let Ok((mut base_tf, mut base_vel)) = base_query.single_mut() {
                            *base_tf = robot.root_transform;
                            *base_vel = Velocity::zero();
                        }

                        for (link_idx, mut tf, mut vel) in link_query.iter_mut() {
                            if link_idx.0 < robot.links.len() {
                                *tf = robot.links[link_idx.0].world_transform;
                            }
                            *vel = Velocity::zero();
                        }

                        for (joint_idx, mut mb_joint) in joint_query.iter_mut() {
                            if joint_idx.0 < robot.joints.len() {
                                let target = robot.joints[joint_idx.0].desired_angle;
                                if let TypedJoint::RevoluteJoint(ref mut rev) = mb_joint.data {
                                    rev.set_motor_position(target, 80.0, 1.5);
                                }
                            }
                        }
                    }
                    if let Some(ref mut debug) = debug_context {
                        ui.checkbox(&mut debug.enabled, "Show Collision Shapes");
                    }
                });

                ui.add_space(8.0);
                ui.separator();

                if let Some(ref mut sim_speed) = sim_speed {
                    egui::CollapsingHeader::new("Simulation Controls")
                        .default_open(true)
                        .show(ui, |ui| {
                            let mut speed = sim_speed.speed;
                            let mut changed = false;

                            ui.horizontal(|ui| {
                                ui.label("Simulation Speed:");
                                let slider_res = ui.add(
                                    egui::Slider::new(&mut speed, 0.1..=3.0)
                                        .suffix("x")
                                        .fixed_decimals(2),
                                );
                                if slider_res.changed() {
                                    changed = true;
                                }
                                if ui.button("1.0x").clicked() {
                                    speed = 1.0;
                                    changed = true;
                                }
                            });

                            ui.horizontal(|ui| {
                                ui.label("Presets:");
                                for &preset in &[0.25, 0.5, 1.0, 1.5, 2.0] {
                                    if ui
                                        .selectable_label(
                                            (speed - preset).abs() < 0.01,
                                            format!("{preset}x"),
                                        )
                                        .clicked()
                                    {
                                        speed = preset;
                                        changed = true;
                                    }
                                }
                            });

                            if changed {
                                sim_speed.speed = speed;
                                update_sim_speed(speed, &mut timestep_mode, &mut virtual_time);
                            }
                        });
                    ui.add_space(8.0);
                    ui.separator();
                }

                if let Some(mut cam) = camera_settings {
                    egui::CollapsingHeader::new("Camera Controls").show(ui, |ui| {
                        ui.horizontal(|ui| {
                            ui.label("Flying Speed:");
                            ui.add(
                                egui::Slider::new(&mut cam.fly_speed, 0.5..=30.0)
                                    .suffix(" m/s")
                                    .fixed_decimals(1),
                            );
                        });
                    });
                    ui.add_space(8.0);
                    ui.separator();
                }
            });
        });
}

fn update_sim_speed(
    speed: f32,
    timestep_mode: &mut Option<ResMut<TimestepMode>>,
    virtual_time: &mut Option<ResMut<Time<Virtual>>>,
) {
    if let Some(mode) = timestep_mode {
        match mode.as_mut() {
            TimestepMode::Fixed { dt, substeps } => {
                *dt = (1.0 / 60.0) * speed;
                *substeps = (4.0 * speed.max(1.0)).round() as usize;
            }
            TimestepMode::Variable { time_scale, .. } => {
                *time_scale = speed;
            }
            TimestepMode::Interpolated { time_scale, .. } => {
                *time_scale = speed;
            }
        }
    }
    if let Some(time) = virtual_time {
        time.set_relative_speed(speed);
    }
}
