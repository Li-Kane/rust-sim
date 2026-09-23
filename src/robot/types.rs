use std::io::Read;

use bevy::prelude::*;
use bevy_rapier3d::dynamics::{AdditionalMassProperties, JointAxesMask, TypedJoint};

pub struct RobotBlueprint {
    pub name: String,
    pub root_link: usize,
    pub joints: Vec<JointBlueprint>,
    pub links: Vec<LinkBlueprint>,
}

pub struct JointBlueprint {
    pub name: String,
    pub joint_data: TypedJoint,
    pub parent_link: usize,
    pub child_link: usize,
    pub local_transform: Transform,
}

pub struct LinkBlueprint {
    pub name: String,
    pub additional_mass_properties: AdditionalMassProperties,
    pub visuals: Vec<Handle<WorldAsset>>,
    pub collisions: Vec<Handle<WorldAsset>>,
    pub parent_joint: Option<usize>,
    pub children_joints: Vec<usize>,
    pub local_transform: Transform,
}

pub trait Parse {
    /// Convert a file path to a [`RobotBluePrint`].
    fn parse<R: Read>(reader: R, asset_server: &AssetServer) -> RobotBlueprint;
}

#[derive(Component, Debug, Default)]
pub struct SpawnPose {
    pub root_transform: Transform,
    pub positions: Box<[f32]>,
}

#[derive(Component, Debug)]
pub struct SpotRobot {
    pub policy_enabled: bool,
    pub previous_action: [f32; 12],
    pub velocity_command: [f32; 3],    //[forward, lateral, yaw_rate]
    pub previous_joint_pos: [f32; 12], // URDF order
}

impl Default for SpotRobot {
    fn default() -> Self {
        Self {
            policy_enabled: true,
            previous_action: [0.0; 12],
            velocity_command: [0.0; 3],
            previous_joint_pos: [0.0; 12],
        }
    }
}

#[derive(Component, Debug)]
pub struct RobotName(pub String);

#[derive(Component, Debug)]
pub struct Robot {
    pub root: Entity,
}

#[derive(Component, Debug)]
pub struct RobotJoints {
    pub joints: Vec<JointRef>,
    pub num_dofs: usize,
}

#[derive(Component, Debug, Clone, Default)]
pub struct RobotPose {
    pub positions: Box<[f32]>,
}

#[derive(Debug, Clone)]
pub struct JointRef {
    pub name: String,
    pub entity: Entity,
    pub dofs: Vec<JointAxesMask>,
}
