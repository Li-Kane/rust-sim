use bevy::prelude::*;

use crate::robot::types::{DEFAULT_JOINT_ANGLES, RobotModel, urdf_to_bevy_orientation};

impl RobotModel {
    /// Resets all 12 joint angles to Spot's default standing pose and clears velocities
    pub fn reset_to_default_pose(&mut self) {
        for (name, angle) in DEFAULT_JOINT_ANGLES {
            if let Some(&idx) = self.joint_name_to_idx.get(name) {
                self.joints[idx].angle = angle;
                self.joints[idx].default_angle = angle;
                self.joints[idx].desired_angle = angle;
                self.joints[idx].velocity = 0.0;
            }
        }
        self.update_kinematics();
    }

    /// Resets full robot state (default pose + center drop transform)
    pub fn reset_full(&mut self) {
        self.root_transform = Transform {
            translation: Vec3::new(0.0, 0.65, 0.0),
            rotation: urdf_to_bevy_orientation(),
            scale: Vec3::ONE,
        };
        self.reset_to_default_pose();
    }

    /// Computes hierarchical forward kinematics for all links
    pub fn update_kinematics(&mut self) {
        // Root link takes the base model transform
        self.links[self.root_link].world_transform = self.root_transform;

        // Propagate forward through joints
        let mut queue = vec![self.root_link];
        while let Some(parent_link_idx) = queue.pop() {
            let parent_world = self.links[parent_link_idx].world_transform;
            let child_joints = self.links[parent_link_idx].child_joints.clone();

            for joint_idx in child_joints {
                let joint = &self.joints[joint_idx];
                let child_link_idx = joint.child_link;

                // Local joint transform = origin * rotation_around_axis
                let joint_rotation = Quat::from_axis_angle(joint.axis, joint.angle);
                let joint_local = Transform {
                    translation: joint.origin_transform.translation,
                    rotation: joint.origin_transform.rotation * joint_rotation,
                    scale: joint.origin_transform.scale,
                };

                let child_world = parent_world * joint_local;
                self.links[child_link_idx].world_transform = child_world;

                queue.push(child_link_idx);
            }
        }
    }
}
