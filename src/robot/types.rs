use bevy::prelude::*;
use bevy_rapier3d::dynamics::{MultibodyJoint, RigidBody};

pub struct RobotBluePrint {
    pub name: String,
    pub joints: Vec<MultibodyJoint>,
    pub links: Vec<RigidBody>,
}

#[derive(Component, Debug, Clone)]
pub struct Robot {
    pub name: String,
    pub joints: Vec<Entity>,
    pub links: Vec<Entity>,
    pub joints_name_to_idx: std::collections::HashMap<String, usize>,
    pub links_name_to_idx: std::collections::HashMap<String, usize>,
}

impl Robot {
    pub fn get_link(&self, name: &str) -> Option<Entity> {
        self.links_name_to_idx.get(name).map(|&idx| self.links[idx])
    }

    pub fn get_joint(&self, name: &str) -> Option<Entity> {
        self.joints_name_to_idx
            .get(name)
            .map(|&idx| self.joints[idx])
    }
}
