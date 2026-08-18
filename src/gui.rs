use bevy::prelude::*;
use bevy_egui::{egui, EguiContexts, EguiPlugin, EguiPrimaryContextPass};

use crate::skeleton::Skeleton;
use crate::SimState;

pub struct SimGuiPlugin;

impl Plugin for SimGuiPlugin {
    fn build(&self, app: &mut App) {
        app.add_plugins(EguiPlugin::default())
            .add_systems(
                EguiPrimaryContextPass,
                joint_inspector_ui.run_if(in_state(SimState::Paused)),
            );
    }
}

pub fn joint_inspector_ui(
    mut contexts: EguiContexts,
    skeleton: Option<ResMut<Skeleton>>,
) {
    let Ok(ctx) = contexts.ctx_mut() else { return };
    let Some(mut skeleton) = skeleton else { return };

    egui::Window::new("Joint Inspector")
        .default_open(true)
        .collapsible(true)
        .resizable(true)
        .default_size([300.0, 400.0])
        .show(ctx, |ui| {
            egui::ScrollArea::vertical().show(ui, |ui| {
                ui.heading("Skeleton Joints");
                ui.separator();

                if skeleton.joints.is_empty() {
                    ui.label("No joints available.");
                    return;
                }

                for joint in &mut skeleton.joints {
                    ui.group(|ui| {
                        ui.label(egui::RichText::new(&joint.name).strong());
                        ui.indent(&joint.name, |ui| {
                            let dof_labels = ["dof x:", "dof y:", "dof z:"];
                            for (i, label) in dof_labels.iter().enumerate() {
                                ui.horizontal(|ui| {
                                    ui.label(*label);
                                    ui.add(
                                        egui::DragValue::new(&mut joint.dofs[i].value)
                                            .speed(0.01),
                                    );
                                });
                            }
                        });
                    });
                    ui.add_space(4.0);
                }
            });
        });
}
