pub mod contact;
pub mod debug;
pub mod dynamics;
pub mod types;

pub use types::*;

use bevy::prelude::*;

use crate::SimState;
use crate::robot::RobotModel;
use contact::{compute_contact_force, get_robot_contact_points, propagate_contact_force_to_joints};
use debug::debug_draw_colliders_system;
use dynamics::{step_floating_base, step_joint_actuators};

pub struct PhysicsPlugin;

impl Plugin for PhysicsPlugin {
    fn build(&self, app: &mut App) {
        app.init_resource::<PhysicsSettings>()
            .init_resource::<PhysicsRigidBody>()
            .add_systems(
                Update,
                (
                    physics_step_system.run_if(in_state(SimState::InGame)),
                    debug_draw_colliders_system,
                ),
            );
    }
}

/// Physics stepping system using Hunt-Crossley contact model and dynamic compliant joint actuation
pub fn physics_step_system(
    time: Res<Time>,
    mut robot: ResMut<RobotModel>,
    mut rb: ResMut<PhysicsRigidBody>,
    settings: Res<PhysicsSettings>,
) {
    let dt = time.delta_secs().min(0.033);
    if dt <= 0.0 {
        return;
    }

    let substeps = settings.substeps.max(1);
    let dt_sub = dt / substeps as f32;

    for _ in 0..substeps {
        // 1. Update forward kinematics to get current world transforms for all links
        robot.update_kinematics();

        let com_pos = robot.root_transform.translation;
        let mut total_force = rb.mass.max(1.0) * settings.gravity;
        let mut total_torque = Vec3::ZERO;
        let num_joints = robot.joints.len();
        let mut joint_torques = vec![0.0f32; num_joints];

        // 2. Evaluate ground contact forces for each contact point
        let contact_points = get_robot_contact_points(&robot, &settings);

        for contact in contact_points {
            let r_arm = contact.world_pos - com_pos;
            let pt_vel = rb.linear_velocity + rb.angular_velocity.cross(r_arm);

            if let Some(contact_force) = compute_contact_force(&contact, pt_vel, &settings) {
                total_force += contact_force;
                total_torque += r_arm.cross(contact_force);

                // Propagate contact force to ancestor joints via Jacobian transpose
                propagate_contact_force_to_joints(
                    &robot,
                    contact.link_idx,
                    contact.world_pos,
                    contact_force,
                    &mut joint_torques,
                );
            }
        }

        // 3. Step floating base rigid body dynamics
        step_floating_base(
            &mut rb,
            &mut robot.root_transform,
            total_force,
            total_torque,
            dt_sub,
        );

        // 4. Step joint actuators and integrate joint angles
        step_joint_actuators(
            &mut robot.joints,
            &joint_torques,
            &settings,
            dt_sub,
        );
    }

    // Keep kinematics synchronized after substeps
    robot.update_kinematics();
}
