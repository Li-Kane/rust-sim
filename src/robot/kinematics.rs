use std::collections::VecDeque;

use bevy::prelude::*;

use super::types::{JointBlueprint, LinkBlueprint};

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
