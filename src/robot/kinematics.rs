use std::collections::VecDeque;

use bevy::prelude::*;
use bevy_rapier3d::prelude::*;

use super::types::{JointBlueprint, LinkBlueprint, Robot, RobotJoints, SpawnPose};
use super::urdf_loader::URDF_TO_BEVY_ROT;

/// Computes world transforms for each link in the robot using forward kinematics.
/// Uses a breadth-first search starting from the blueprint's root_link to guarantee correct topological order.
pub fn compute_link_world_transforms(
    links: &[LinkBlueprint],
    joints: &[JointBlueprint],
    root_link: usize,
    root_transform: Transform,
) -> Vec<Transform> {
    let mut link_world_transforms = vec![Transform::IDENTITY; links.len()];
    let mut queue = VecDeque::new();

    // apply root transform and enqueue
    if root_link < links.len() {
        link_world_transforms[root_link] = root_transform * links[root_link].local_transform;
        queue.push_back(root_link);
    }

    // traverse links in breadth-first order
    while let Some(parent_idx) = queue.pop_front() {
        let parent_world = link_world_transforms[parent_idx];

        for &child_joint_idx in &links[parent_idx].children_joints {
            let joint = &joints[child_joint_idx];
            let joint_world = parent_world * joint.local_transform;
            let child_idx = joint.child_link;

            link_world_transforms[child_idx] = joint_world * links[child_idx].local_transform;
            queue.push_back(child_idx);
        }
    }

    link_world_transforms
}

/// Resets the robot to its spawn pose, enforcing a deterministic physical state.
pub fn reset_robot_physics_state(
    robot: &Robot,
    robot_joints: &RobotJoints,
    spawn_pose: &SpawnPose,
    link_query: &mut Query<(&mut Transform, Option<&mut Velocity>)>,
    rapier_context_joints: &mut RapierContextJoints,
) {
    // 1. Reset root transform and zero velocities in Bevy (Bevy-Rapier auto-syncs this to Rapier's RigidBodies)
    if let Ok((mut transform, velocity)) = link_query.get_mut(robot.root) {
        *transform = spawn_pose.root_transform * URDF_TO_BEVY_ROT;
        if let Some(mut vel) = velocity {
            *vel = Velocity::zero();
        }
    }

    for joint in &robot_joints.joints {
        if let Ok((_, Some(mut vel))) = link_query.get_mut(joint.entity) {
            *vel = Velocity::zero();
        }
    }

    // 2. Pre-collect link IDs to satisfy the borrow checker
    let joint_link_ids: Vec<Option<usize>> = robot_joints
        .joints
        .iter()
        .map(|joint_ref| {
            rapier_context_joints
                .entity2multibody_joint()
                .get(&joint_ref.entity)
                .and_then(|handle| rapier_context_joints.multibody_joints.get(*handle))
                .map(|(_, link_id)| link_id)
        })
        .collect();

    // 3. Reset Rapier's internal Multibody state
    let Some(first_joint) = robot_joints.joints.first() else {
        return;
    };
    let Some(handle) = rapier_context_joints
        .entity2multibody_joint()
        .get(&first_joint.entity)
    else {
        return;
    };
    let Some((multibody, _)) = rapier_context_joints.multibody_joints.get_mut(*handle) else {
        return;
    };

    // Clear reduced-coordinate momentum
    multibody.generalized_velocity_mut().fill(0.0);

    // Snap joint coords to spawn pose
    let mut dof_idx = 0;
    for (joint_ref, link_id_opt) in robot_joints.joints.iter().zip(joint_link_ids) {
        let num_dofs = joint_ref.dofs.len();
        if let Some(link_id) = link_id_opt {
            if let Some(link) = multibody.link_mut(link_id) {
                let coords = link.joint.coords();
                let disp: Vec<f32> = joint_ref
                    .dofs
                    .iter()
                    .enumerate()
                    .map(|(i, dof)| {
                        let axis = dof.bits().trailing_zeros() as usize;
                        spawn_pose.positions[dof_idx + i] - coords[axis]
                    })
                    .collect();

                if !disp.is_empty() {
                    link.joint.apply_displacement(&disp);
                }
            }
        }
        dof_idx += num_dofs;
    }
}
