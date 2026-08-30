use bevy::prelude::*;
use bevy_egui::{EguiContexts, EguiPlugin, EguiPrimaryContextPass, egui};

use crate::SimState;
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

pub fn joint_inspector_ui(mut contexts: EguiContexts, robot: Option<ResMut<RobotModel>>) {
    let Ok(ctx) = contexts.ctx_mut() else { return };
    let Some(mut robot) = robot else { return };

    egui::Window::new("Spot Robot Configuration")
        .default_open(true)
        .collapsible(true)
        .resizable(true)
        .default_size([380.0, 520.0])
        .show(ctx, |ui| {
            egui::ScrollArea::vertical().show(ui, |ui| {
                ui.heading("Robot Joint Controls");
                ui.label("Press ESC to toggle camera controls.");
                ui.separator();

                if ui.button("Default Standing Pose").clicked() {
                    robot.reset_to_default_pose();
                }

                ui.add_space(8.0);
                ui.heading("Joints");

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

                ui.add_space(10.0);
                ui.separator();
                ui.collapsing("Base Root Transform", |ui| {
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
