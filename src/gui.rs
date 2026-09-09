use bevy::prelude::*;
use bevy_egui::{EguiContexts, EguiPlugin, EguiPrimaryContextPass, egui};
use bevy_rapier3d::prelude::*;

use crate::SimState;
use crate::input::CameraSettings;

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
            );
    }
}

pub fn sim_ui(
    mut contexts: EguiContexts,
    camera_settings: Option<ResMut<CameraSettings>>,
    mut debug_context: Option<ResMut<DebugRenderContext>>,
    mut sim_speed: Option<ResMut<SimulationSpeed>>,
    mut timestep_mode: Option<ResMut<TimestepMode>>,
    mut virtual_time: Option<ResMut<Time<Virtual>>>,
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

                            if changed {
                                sim_speed.speed = speed;
                                update_sim_speed(speed, &mut timestep_mode, &mut virtual_time);
                            }
                        });
                    ui.add_space(8.0);
                    ui.separator();
                }

                if let Some(mut cam) = camera_settings {
                    egui::CollapsingHeader::new("Camera Controls")
                        .default_open(true)
                        .show(ui, |ui| {
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
    // update physics speed
    if let Some(mode) = timestep_mode {
        match mode.as_mut() {
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
    if let Some(time) = virtual_time {
        time.set_relative_speed(speed);
    }
}
