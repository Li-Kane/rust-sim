#![allow(dead_code)]
use bevy::prelude::*;
use std::collections::HashMap;

/// Default standing joint positions for Spot (in radians)
pub const DEFAULT_JOINT_ANGLES: [(&str, f32); 12] = [
    ("fl_hx", 0.1),
    ("fl_hy", 0.9),
    ("fl_kn", -1.5),
    ("fr_hx", -0.1),
    ("fr_hy", 0.9),
    ("fr_kn", -1.5),
    ("hl_hx", 0.1),
    ("hl_hy", 1.1),
    ("hl_kn", -1.5),
    ("hr_hx", -0.1),
    ("hr_hy", 1.1),
    ("hr_kn", -1.5),
];

/// Returns the coordinate conversion rotation from URDF (Z-up, X-fwd, Y-left) to Bevy (Y-up, -Z-fwd, +X-right)
pub fn urdf_to_bevy_orientation() -> Quat {
    Quat::from_mat3(&Mat3::from_cols(
        Vec3::new(0.0, 0.0, -1.0),
        Vec3::new(-1.0, 0.0, 0.0),
        Vec3::new(0.0, 1.0, 0.0),
    ))
}

/// A revolute or fixed joint in the robot tree
#[derive(Debug, Clone)]
pub struct RobotJoint {
    pub name: String,
    pub parent_link: usize,
    pub child_link: usize,
    pub origin_transform: Transform,
    pub axis: Vec3,
    pub angle: f32,          // Current angle in radians
    pub velocity: f32,       // Current angular velocity in rad/s
    pub desired_angle: f32,  // PD setpoint in radians
    pub default_angle: f32,  // in radians
    pub lower_limit: f32,    // in radians
    pub upper_limit: f32,    // in radians
    pub max_effort: f32,     // Max actuator effort in Nm
    pub inertia: f32,        // Apparent joint inertia in kg*m^2
}

/// Body collision box dimensions in URDF frame (Length: 0.58m, Width: 0.26m, Height: 0.16m)
pub const BODY_BOX_SIZE: Vec3 = Vec3::new(0.58, 0.26, 0.16);

/// Foot collision sphere radius (m)
pub const FOOT_RADIUS: f32 = 0.02;

/// Foot offset in URDF lower leg local frame
pub const FOOT_OFFSET: Vec3 = Vec3::new(0.0, 0.0, -0.34);

/// Marker component for link entities
#[derive(Component, Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub struct LinkEntityIndex(pub usize);

/// Marker component for robot revolute joint entities
#[derive(Component, Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub struct RobotJointIndex(pub usize);

/// Marker component for the robot floating base entity
#[derive(Component, Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub struct RobotBaseMarker;

/// A rigid link in the robot tree
#[derive(Debug, Clone)]
pub struct RobotLink {
    pub name: String,
    pub parent_joint: Option<usize>,
    pub child_joints: Vec<usize>,
    pub visual_origin: Transform,
    pub mesh_name: Option<String>,
    pub color: Color,
    pub world_transform: Transform,
    pub mass: f32,
    pub inertia: Vec3,
}

/// Complete Spot Robot Model Resource
#[derive(Debug, Clone, Resource)]
pub struct RobotModel {
    pub name: String,
    pub links: Vec<RobotLink>,
    pub joints: Vec<RobotJoint>,
    pub root_link: usize,
    pub root_transform: Transform,
    pub link_name_to_idx: HashMap<String, usize>,
    pub joint_name_to_idx: HashMap<String, usize>,
}
