use bevy_egui::egui;

use super::SimulationSpeed;

pub fn render_sim_controls(ui: &mut egui::Ui, sim_speed: &mut SimulationSpeed) {
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
            }
        });
}
