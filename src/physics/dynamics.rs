use bevy::prelude::*;

use crate::physics::types::{PhysicsRigidBody, PhysicsSettings};
use crate::robot::RobotJoint;

/// Steps the floating base 6-DOF Newton-Euler dynamics and quaternion orientation
pub fn step_floating_base(
    rb: &mut PhysicsRigidBody,
    root_transform: &mut Transform,
    mut total_force: Vec3,
    mut total_torque: Vec3,
    dt_sub: f32,
) {
    let mass = rb.mass.max(1.0);
    let inertia = rb.inertia;

    // Stabilizing aerodynamic/numerical drag on base
    let linear_damping = -0.3 * rb.linear_velocity;
    let angular_damping = -0.8 * rb.angular_velocity;
    total_force += linear_damping;
    total_torque += angular_damping;

    // Linear motion integration
    let linear_accel = total_force / mass;
    rb.linear_velocity += linear_accel * dt_sub;
    root_transform.translation += rb.linear_velocity * dt_sub;

    // Angular motion integration with principal inertia tensor transformation
    let rot_mat = Mat3::from_quat(root_transform.rotation);
    let inv_i_local = Vec3::new(1.0 / inertia.x, 1.0 / inertia.y, 1.0 / inertia.z);
    let inv_i_world = rot_mat * Mat3::from_diagonal(inv_i_local) * rot_mat.transpose();

    let gyro_torque = rb.angular_velocity.cross(
        (rot_mat * Mat3::from_diagonal(inertia) * rot_mat.transpose()) * rb.angular_velocity,
    );
    let net_rot_torque = total_torque - gyro_torque;
    let angular_accel = inv_i_world * net_rot_torque;

    rb.angular_velocity += angular_accel * dt_sub;

    // Quaternion orientation integration
    let w_quat = Quat::from_xyzw(
        rb.angular_velocity.x,
        rb.angular_velocity.y,
        rb.angular_velocity.z,
        0.0,
    );
    let q_curr = root_transform.rotation;
    let q_dot = w_quat * q_curr * 0.5;
    let q_new = Quat::from_xyzw(
        q_curr.x + q_dot.x * dt_sub,
        q_curr.y + q_dot.y * dt_sub,
        q_curr.z + q_dot.z * dt_sub,
        q_curr.w + q_dot.w * dt_sub,
    );
    root_transform.rotation = q_new.normalize();
}

/// Evaluates joint PD actuator efforts, applies external torques, and integrates joint angles
pub fn step_joint_actuators(
    joints: &mut [RobotJoint],
    external_torques: &[f32],
    settings: &PhysicsSettings,
    dt_sub: f32,
) {
    for (j_idx, joint) in joints.iter_mut().enumerate() {
        let err = joint.desired_angle - joint.angle;
        let tau_act = (settings.joint_kp * err - settings.joint_kd * joint.velocity)
            .clamp(-joint.max_effort, joint.max_effort);

        let tau_net = tau_act + external_torques[j_idx];
        let q_accel = tau_net / joint.inertia.max(0.01);

        joint.velocity += q_accel * dt_sub;
        joint.angle = (joint.angle + joint.velocity * dt_sub).clamp(joint.lower_limit, joint.upper_limit);

        // Damp velocities at physical hard limits
        if joint.angle <= joint.lower_limit && joint.velocity < 0.0 {
            joint.velocity = 0.0;
        } else if joint.angle >= joint.upper_limit && joint.velocity > 0.0 {
            joint.velocity = 0.0;
        }
    }
}
