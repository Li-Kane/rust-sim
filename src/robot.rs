pub mod kinematics;
pub mod spawner;
pub mod types;
pub mod urdf_loader;

use bevy::prelude::*;
use bevy_rapier3d::prelude::*;

pub use spawner::spawn_robot;
use crate::SimState;
use types::{RobotEntity, RobotPose};

pub struct RobotPlugin;

impl Plugin for RobotPlugin {
    fn build(&self, app: &mut App) {
        app.add_systems(
            Update,
            (
                apply_robot_pose,
                check_robot_loaded.run_if(in_state(SimState::Loading)),
            ),
        );
    }
}

/// Transitions to `SimState::InGame` once all robot collision meshes are loaded and colliders generated.
fn check_robot_loaded(
    mut next_state: ResMut<NextState<SimState>>,
    robots: Query<&RobotEntity>,
    async_colliders: Query<&AsyncSceneCollider>,
) {
    if !robots.is_empty() && async_colliders.is_empty() {
        next_state.set(SimState::InGame);
    }
}

/// Applies the robot pose to the multibody joints.
pub fn apply_robot_pose(
    robot_query: Query<(&RobotEntity, &RobotPose), Changed<RobotPose>>,
    mut joint_query: Query<&mut MultibodyJoint>,
) {
    for (robot, pose) in robot_query.iter() {
        // size check
        if pose.positions.len() != robot.num_dofs {
            warn!(
                "Pose length {} does not match robot num_dofs {} for '{}'",
                pose.positions.len(),
                robot.num_dofs,
                robot.name
            );
            continue;
        }

        // Apply the robot pose to the multibody joints
        let mut dof_idx = 0;
        for joint in &robot.joints {
            if let Ok(mut multibody_joint) = joint_query.get_mut(joint.entity) {
                let generic = &mut multibody_joint.data.as_mut().raw;
                for dof in &joint.dofs {
                    generic.motors[*dof].target_pos = pose.positions[dof_idx];
                    dof_idx += 1;
                }
            } else {
                dof_idx += joint.dofs.len();
            }
        }
    }
}
