use std::path::Path;

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
    pub parent: String,
    pub child: String,
}

pub struct LinkBlueprint {
    pub name: String,
    pub additional_mass_properties: AdditionalMassProperties,
    pub visuals: Vec<Handle<WorldAsset>>,
}

pub trait Parse {
    /// Convert a file path to a [`RobotBluePrint`].
    fn parse(file_path: &Path, asset_server: &AssetServer) -> RobotBlueprint;
}
