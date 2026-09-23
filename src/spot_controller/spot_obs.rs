use bevy::prelude::*;
use bevy_rapier3d::prelude::*;

use crate::robot::types::{Robot, RobotJoints, SpawnPose, SpotRobot};
use crate::spot_controller::spot_policy;

pub fn get_observation(
    robot: &Robot,
    joints: &RobotJoints,
    spawn_pose: &SpawnPose,
    state: &SpotRobot,
    transform_query: &Query<&Transform>,
    velocity_query: &Query<&Velocity>,
    rapier_context_joints: &RapierContextJoints,
    dt: f32,
) -> ([f32; spot_policy::OBSERVATION_DIM], [f32; 12]) {
    let mut obs = [0.0f32; spot_policy::OBSERVATION_DIM];

    let root_transform = transform_query
        .get(robot.root)
        .unwrap_or(&Transform::IDENTITY);
    let default_vel = Velocity::zero();
    let root_velocity = velocity_query.get(robot.root).unwrap_or(&default_vel);

    // --- COORDINATE MAPPING ---
    // Because the root entity's Transform contains URDF_TO_BEVY_ROT,
    // multiplying by rotation.inverse() brings world vectors DIRECTLY into URDF/Policy frame (ROS Z-up).
    
    // 1. Base Linear Velocity (0..3)
    let linvel_policy = root_transform.rotation.inverse() * root_velocity.linear;
    obs[0..3].copy_from_slice(&linvel_policy.to_array());

    // 2. Base Angular Velocity (3..6)
    let angvel_policy = root_transform.rotation.inverse() * root_velocity.angular;
    obs[3..6].copy_from_slice(&angvel_policy.to_array());

    // 3. Projected Gravity (6..9)
    let gravity_world = Vec3::new(0.0, -1.0, 0.0); // Unit gravity vector
    let gravity_policy = root_transform.rotation.inverse() * gravity_world;
    obs[6..9].copy_from_slice(&gravity_policy.to_array());

    // 4. Velocity Commands (9..12)
    obs[9..12].copy_from_slice(&state.velocity_command);

    // --- READ EXACT JOINT POSITIONS FROM RAPIER ---
    let mut urdf_pos = [0.0; 12];
    if spawn_pose.positions.len() >= 12 {
        urdf_pos.copy_from_slice(&spawn_pose.positions[0..12]);
    }

    let multibody = joints.joints.first().and_then(|first_joint| {
        rapier_context_joints
            .entity2multibody_joint()
            .get(&first_joint.entity)
            .and_then(|handle| rapier_context_joints.multibody_joints.get(*handle))
            .map(|(mb, _)| mb)
    });

    if let Some(mb) = multibody {
        let mut dof_idx = 0;
        for joint_ref in &joints.joints {
            if let Some(handle) = rapier_context_joints
                .entity2multibody_joint()
                .get(&joint_ref.entity)
            {
                if let Some((_, link_id)) = rapier_context_joints.multibody_joints.get(*handle) {
                    if let Some(link) = mb.link(link_id) {
                        let coords = link.joint.coords();
                        for (i, dof) in joint_ref.dofs.iter().enumerate() {
                            let axis = dof.bits().trailing_zeros() as usize;
                            if dof_idx + i < 12 {
                                urdf_pos[dof_idx + i] = coords[axis];
                            }
                        }
                    }
                }
            }
            dof_idx += joint_ref.dofs.len();
        }
    }

    // Calculate Joint Velocities (URDF Order)
    let safe_dt = dt.max(0.0001); // Prevent division by zero
    let mut urdf_vel = [0.0f32; 12];
    for i in 0..12 {
        urdf_vel[i] = (urdf_pos[i] - state.previous_joint_pos[i]) / safe_dt;
    }

    // --- APPLY POLICY REMAPPING ---
    // 5 & 6. Relative Joint Pos (12..24) and Joint Vel (24..36) in POLICY order
    for (policy_idx, &urdf_idx) in spot_policy::URDF_TO_POLICY.iter().enumerate() {
        obs[12 + policy_idx] = urdf_pos[urdf_idx] - spawn_pose.positions[urdf_idx];
        obs[24 + policy_idx] = urdf_vel[urdf_idx];
    }

    // 7. Previous Action (36..48)
    obs[36..48].copy_from_slice(&state.previous_action);

    // Return the observation and the physical urdf_pos so we can easily save it in state
    (obs, urdf_pos)
}
