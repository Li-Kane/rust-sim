use bevy_egui::egui;

use crate::input::CameraSettings;

pub fn render_camera_controls(ui: &mut egui::Ui, cam_settings: &mut CameraSettings) {
    egui::CollapsingHeader::new("Camera Controls")
        .default_open(true)
        .show(ui, |ui| {
            ui.horizontal(|ui| {
                ui.label("Flying Speed:");
                ui.add(
                    egui::Slider::new(&mut cam_settings.fly_speed, 0.5..=30.0)
                        .suffix(" m/s")
                        .fixed_decimals(1),
                );
            });
        });
}
