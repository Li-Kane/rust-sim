pub mod camera_controls;
pub mod sim_controls;

use bevy::prelude::*;
use bevy_egui::{egui, EguiContexts, EguiPlugin, EguiPrimaryContextPass};
use bevy_rapier3d::prelude::*;

use crate::SimState;
use crate::input::CameraSettings;
use camera_controls::render_camera_controls;
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
    mut debug_context: Option<ResMut<DebugRenderContext>>,
    mut sim_speed: Option<ResMut<SimulationSpeed>>,
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

                // Preserved for upcoming robot joint editor / debug rendering
                let _ = &mut debug_context;
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
