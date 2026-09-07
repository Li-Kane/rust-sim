use bevy::prelude::*;
use bevy_rapier3d::prelude::*;

use crate::SimState;
use crate::robot::{LinkEntityIndex, RobotBaseMarker, RobotJoint, RobotJointIndex, RobotModel};

pub struct ControllerPlugin;

impl Plugin for ControllerPlugin {
    fn build(&self, app: &mut App) {
        app.init_resource::<JointPdController>()
            .add_systems(Update, apply_joint_pd_system)
            .add_systems(
                Update,
                sync_robot_state_system.run_if(in_state(SimState::InGame)),
            );
    }
}

/// PD Controller configuration for robot joint actuators
#[derive(Resource, Debug, Clone)]
pub struct JointPdController {
    pub kp: f32, // Proportional gain / stiffness (N*m/rad)
    pub kd: f32, // Derivative gain / damping (N*m*s/rad)
}

impl Default for JointPdController {
    fn default() -> Self {
        Self {
            kp: 80.0,
            kd: 1.5,
        }
    }
}

impl JointPdController {
    pub fn new(kp: f32, kd: f32) -> Self {
        Self { kp, kd }
    }

    /// Computes the control torque for a single joint given its current state and limits
    pub fn compute_torque(&self, joint: &RobotJoint) -> f32 {
        let err = joint.desired_angle - joint.angle;
        let tau_act = self.kp * err - self.kd * joint.velocity;
        tau_act.clamp(-joint.max_effort, joint.max_effort)
    }

    /// Computes control torques for all joints
    pub fn compute_all_torques(&self, joints: &[RobotJoint]) -> Vec<f32> {
        joints
            .iter()
            .map(|joint| self.compute_torque(joint))
            .collect()
    }
}

/// Applies PD setpoints and gains from `RobotModel` and `JointPdController` to Rapier `MultibodyJoint`s
pub fn apply_joint_pd_system(
    robot: Option<Res<RobotModel>>,
    pd: Res<JointPdController>,
    mut joint_query: Query<(&RobotJointIndex, &mut MultibodyJoint)>,
) {
    let Some(robot) = robot else { return };

    for (joint_idx, mut mb_joint) in joint_query.iter_mut() {
        if let Some(joint) = robot.joints.get(joint_idx.0)
            && let TypedJoint::RevoluteJoint(ref mut rev) = mb_joint.data
        {
            rev.set_motor_position(joint.desired_angle, pd.kp, pd.kd);
            rev.set_motor_max_force(joint.max_effort);
        }
    }
}

/// Reads the rigid body transforms and velocities from Rapier back into the `RobotModel` resource
pub fn sync_robot_state_system(
    mut robot: Option<ResMut<RobotModel>>,
    base_query: Query<&Transform, With<RobotBaseMarker>>,
    link_query: Query<(&LinkEntityIndex, &Transform, Option<&Velocity>)>,
) {
    let Some(ref mut robot) = robot else { return };

    if let Ok(base_tf) = base_query.single() {
        robot.root_transform = *base_tf;
    }

    for (link_idx, tf, _vel) in link_query.iter() {
        if link_idx.0 < robot.links.len() {
            robot.links[link_idx.0].world_transform = *tf;
        }
    }

    // Update joint angles from relative link transforms
    let num_joints = robot.joints.len();
    let num_links = robot.links.len();
    for i in 0..num_joints {
        let p_idx = robot.joints[i].parent_link;
        let c_idx = robot.joints[i].child_link;
        if p_idx >= num_links || c_idx >= num_links {
            continue;
        }
        let parent_tf = robot.links[p_idx].world_transform;
        let child_tf = robot.links[c_idx].world_transform;
        let origin_rot = robot.joints[i].origin_transform.rotation;
        let axis = robot.joints[i].axis;

        let q_rel = parent_tf.rotation.inverse() * child_tf.rotation;
        let q_joint = origin_rot.inverse() * q_rel;

        let v = q_joint.xyz();
        let sin_half = v.dot(axis);
        let cos_half = q_joint.w;
        let theta = 2.0 * sin_half.atan2(cos_half);

        let angle = (theta + std::f32::consts::PI).rem_euclid(2.0 * std::f32::consts::PI)
            - std::f32::consts::PI;
        robot.joints[i].angle = angle;
    }
}
