use bevy::prelude::*;
use bevy_egui::{EguiContexts, EguiPlugin, EguiPrimaryContextPass, egui};

use crate::SimState;
use crate::input::CameraSettings;
use crate::physics::PhysicsRigidBody;
use crate::robot::RobotModel;

pub struct SimGuiPlugin;

impl Plugin for SimGuiPlugin {
    fn build(&self, app: &mut App) {
        app.add_plugins(EguiPlugin::default()).add_systems(
            EguiPrimaryContextPass,
            joint_inspector_ui.run_if(in_state(SimState::Config)),
        );
    }
}

pub fn joint_inspector_ui(
    mut contexts: EguiContexts,
    robot: Option<ResMut<RobotModel>>,
    rb: Option<ResMut<PhysicsRigidBody>>,
    camera_settings: Option<ResMut<CameraSettings>>,
) {
    let Ok(ctx) = contexts.ctx_mut() else { return };
    let Some(mut robot) = robot else { return };
    let mut rb = rb;

    egui::Window::new("Spot Robot Configuration")
        .default_open(true)
        .collapsible(true)
        .resizable(true)
        .default_size([400.0, 560.0])
        .show(ctx, |ui| {
            egui::ScrollArea::vertical().show(ui, |ui| {
                ui.heading("Sim Config");
                ui.label("Press ESC to resume physics & camera controls.");
                ui.separator();

                ui.horizontal(|ui| {
                    if ui.button("Reset Robot (Center & Pose)").clicked() {
                        robot.reset_full();
                        if let Some(ref mut rb_state) = rb {
                            rb_state.reset_velocities();
                        }
                    }
                });

                ui.add_space(8.0);
                ui.separator();

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

                egui::CollapsingHeader::new("Joints").show(ui, |ui| {
                    for joint in &mut robot.joints {
                        // Only show movable joints (where limits allow movement)
                        if joint.lower_limit < joint.upper_limit {
                            let mut deg = joint.angle.to_degrees();
                            let min_deg = joint.lower_limit.to_degrees();
                            let max_deg = joint.upper_limit.to_degrees();

                            ui.horizontal(|ui| {
                                ui.label(egui::RichText::new(&joint.name).strong());
                                if ui
                                    .add(
                                        egui::Slider::new(&mut deg, min_deg..=max_deg)
                                            .suffix("°")
                                            .fixed_decimals(1),
                                    )
                                    .changed()
                                {
                                    joint.angle = deg.to_radians();
                                }
                            });
                        }
                    }
                });

                ui.add_space(8.0);
                ui.separator();

                egui::CollapsingHeader::new("Base Root Transform").show(ui, |ui| {
                    ui.label("World position of the robot base:");
                    let mut pos = robot.root_transform.translation;
                    let mut changed = false;

                    ui.horizontal(|ui| {
                        ui.label("X (m):");
                        changed |= ui
                            .add(egui::Slider::new(&mut pos.x, -5.0..=5.0).fixed_decimals(2))
                            .changed();
                    });
                    ui.horizontal(|ui| {
                        ui.label("Y Height (m):");
                        changed |= ui
                            .add(egui::Slider::new(&mut pos.y, 0.0..=3.0).fixed_decimals(2))
                            .changed();
                    });
                    ui.horizontal(|ui| {
                        ui.label("Z (m):");
                        changed |= ui
                            .add(egui::Slider::new(&mut pos.z, -5.0..=5.0).fixed_decimals(2))
                            .changed();
                    });

                    if changed {
                        robot.root_transform.translation = pos;
                    }
                });
            });
        });
}
