use bevy::prelude::*;
use bevy_rapier3d::prelude::*;

use crate::robot::types::{Robot, RobotJoints, RobotPose, SpawnPose, SpotRobot};

pub mod spot_obs;
pub mod spot_policy;

use spot_policy::SpotPolicy;

pub struct SpotControllerPlugin;

impl Plugin for SpotControllerPlugin {
    fn build(&self, app: &mut App) {
        app.insert_resource(Time::<Fixed>::from_hz(50.0));
        app.add_systems(Startup, setup);
        app.add_systems(FixedUpdate, step.run_if(resource_exists::<SpotPolicy>));
    }
}

fn setup(mut commands: Commands) {
    match SpotPolicy::load_default() {
        Ok(policy) => {
            info!("Successfully loaded Spot ONNX policy");
            commands.insert_resource(policy);
        }
        Err(err) => {
            error!("Failed to load Spot ONNX policy: {err}");
        }
    }
}

fn step(
    policy: Res<SpotPolicy>,
    time: Res<Time<Fixed>>,
    transform_query: Query<&Transform>,
    velocity_query: Query<&Velocity>,
    rapier_context_query: Query<&RapierContextJoints>,
    mut spot_query: Query<(
        &mut SpotRobot,
        &Robot,
        &RobotJoints,
        &SpawnPose,
        &mut RobotPose,
    )>,
) {
    return;
    const ACTION_SCALE: f32 = 0.2;

    let rapier_context_joints = match rapier_context_query.iter().next() {
        Some(ctx) => ctx,
        None => return,
    };

    for (mut state, robot, joints, spawn_pose, mut pose) in &mut spot_query {
        if !state.policy_enabled {
            continue;
        }

        let (obs, urdf_pos) = spot_obs::get_observation(
            robot,
            joints,
            spawn_pose,
            &state,
            &transform_query,
            &velocity_query,
            rapier_context_joints,
            time.delta_secs(),
        );

        if let Ok(actions) = policy.step(&obs) {
            // Target Pos = Default Pos + Action * Scale (in URDF Order)
            for (urdf_idx, &policy_idx) in spot_policy::POLICY_TO_URDF.iter().enumerate() {
                let target = spawn_pose.positions[urdf_idx] + actions[policy_idx] * ACTION_SCALE;
                pose.positions[urdf_idx] = target;
            }

            // save previous states
            state.previous_action = actions;
            state.previous_joint_pos = urdf_pos;
        }
    }
}
