use std::io::Read;

use bevy::prelude::*;
use bevy_rapier3d::dynamics::{AdditionalMassProperties, TypedJoint};

pub struct RobotBlueprint {
    pub name: String,
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
