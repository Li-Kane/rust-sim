use bevy::prelude::*;
use bevy_egui::egui;
use bevy_rapier3d::prelude::*;

use crate::robot::kinematics::reset_robot_physics_state;
use crate::robot::types::{Robot, RobotJoints, RobotName, RobotPose, SpawnPose};

/// Generates the UI to control a given robot
pub fn render_robot_controls(
    ui: &mut egui::Ui,
    robot: &Robot,
    robot_name: &RobotName,
    robot_joints: &RobotJoints,
    robot_pose: &mut RobotPose,
    spawn_pose: &SpawnPose,
    link_query: &mut Query<(&mut Transform, Option<&mut Velocity>)>,
    mut rapier_context_joints: Option<&mut RapierContextJoints>,
) {
    egui::CollapsingHeader::new(format!("Robot Controls ({})", robot_name.0))
        .default_open(true)
        .show(ui, |ui| {
            ui.label(format!("Pose ({} DOFs):", robot_pose.positions.len()));

            // Button to reset all joint positions to spawn pose
            if ui
                .button("Reset to Spawn Pose")
                .on_hover_text("Reset robot transform and joint positions to their spawn pose")
                .clicked()
            {
                let count = robot_pose.positions.len().min(spawn_pose.positions.len());
                robot_pose.positions[..count].copy_from_slice(&spawn_pose.positions[..count]);

                if let Some(joints) = rapier_context_joints.as_deref_mut() {
                    reset_robot_physics_state(robot, robot_joints, spawn_pose, link_query, joints);
                }
            }
            ui.separator();

            // Input per position in the pose grouped by joint
            let mut start_idx = 0;
            for joint in robot_joints.joints.iter() {
                let dof_count = joint.dofs.len();
                if dof_count == 0 {
                    continue;
                }

                // get joint dof positions
                let end_idx = (start_idx + dof_count).min(robot_pose.positions.len());
                if start_idx >= end_idx {
                    break;
                }
                let joint_dofs = &mut robot_pose.positions[start_idx..end_idx];
                start_idx = end_idx;

                egui::CollapsingHeader::new(&joint.name)
                    .id_salt(format!("{}_{}", robot_name.0, joint.name))
                    .default_open(true)
                    .show(ui, |ui| {
                        egui::Grid::new(format!("{}_{}_grid", robot_name.0, joint.name))
                            .num_columns(3)
                            .spacing([8.0, 4.0])
                            .striped(true)
                            .show(ui, |ui| {
                                for (i, dof) in joint.dofs.iter().enumerate() {
                                    let axis_name = dof
                                        .iter_names()
                                        .next()
                                        .map(|(name, _)| name)
                                        .unwrap_or("Unknown");
                                    ui.label(axis_name);
                                    ui.add(
                                        egui::Slider::new(
                                            &mut joint_dofs[i],
                                            -std::f32::consts::PI..=std::f32::consts::PI,
                                        )
                                        .suffix(" rad")
                                        .clamping(egui::SliderClamping::Never)
                                        .fixed_decimals(2),
                                    );
                                    if ui.button("0.0").on_hover_text("Reset to 0.0").clicked() {
                                        joint_dofs[i] = 0.0;
                                    }
                                    ui.end_row();
                                }
                            });
                    });
            }
        });
}
