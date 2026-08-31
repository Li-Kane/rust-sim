use bevy::prelude::*;
use std::collections::HashMap;

use crate::robot::mesh::URDF_CONTENT;
use crate::robot::types::{RobotJoint, RobotLink, RobotModel, urdf_to_bevy_orientation};

/// Builds the Spot Robot model by parsing the embedded URDF XML
pub fn load_spot_model() -> Result<RobotModel, String> {
    let robot = urdf_rs::read_from_string(URDF_CONTENT)
        .map_err(|e| format!("Failed to parse URDF: {e:?}"))?;

    let mut links = Vec::new();
    let mut link_name_to_idx = HashMap::new();

    // 1. Collect Links
    for (i, link) in robot.links.iter().enumerate() {
        link_name_to_idx.insert(link.name.clone(), i);

        let mut visual_origin = Transform::IDENTITY;
        let mut mesh_name = None;
        let mut color = Color::srgb(0.8, 0.8, 0.8);

        if let Some(visual) = link.visual.first() {
            let xyz = visual.origin.xyz;
            let rpy = visual.origin.rpy;
            let rot = Quat::from_euler(EulerRot::ZYX, rpy[2] as f32, rpy[1] as f32, rpy[0] as f32);
            visual_origin = Transform {
                translation: Vec3::new(xyz[0] as f32, xyz[1] as f32, xyz[2] as f32),
                rotation: rot,
                scale: Vec3::ONE,
            };

            if let urdf_rs::Geometry::Mesh { filename, .. } = &visual.geometry {
                if filename.contains("body.obj") {
                    mesh_name = Some("body.obj".to_string());
                } else if filename.contains("hip.obj") {
                    mesh_name = Some("hip.obj".to_string());
                } else if filename.contains("uleg.obj") {
                    mesh_name = Some("uleg.obj".to_string());
                } else if filename.contains("lleg.obj") {
                    mesh_name = Some("lleg.obj".to_string());
                }
            }

            if let Some(mat) = &visual.material {
                if let Some(col) = &mat.color {
                    let rgba = col.rgba;
                    color = Color::srgba(
                        rgba[0] as f32,
                        rgba[1] as f32,
                        rgba[2] as f32,
                        rgba[3] as f32,
                    );
                }
            }
        }

        links.push(RobotLink {
            name: link.name.clone(),
            parent_joint: None,
            child_joints: Vec::new(),
            visual_origin,
            mesh_name,
            color,
            world_transform: Transform::IDENTITY,
        });
    }

    let mut joints = Vec::new();
    let mut joint_name_to_idx = HashMap::new();

    // 2. Collect Joints
    for joint in &robot.joints {
        let parent_link = *link_name_to_idx
            .get(&joint.parent.link)
            .ok_or_else(|| format!("Parent link {} not found", joint.parent.link))?;
        let child_link = *link_name_to_idx
            .get(&joint.child.link)
            .ok_or_else(|| format!("Child link {} not found", joint.child.link))?;

        let xyz = joint.origin.xyz;
        let rpy = joint.origin.rpy;
        let rot = Quat::from_euler(EulerRot::ZYX, rpy[2] as f32, rpy[1] as f32, rpy[0] as f32);
        let origin_transform = Transform {
            translation: Vec3::new(xyz[0] as f32, xyz[1] as f32, xyz[2] as f32),
            rotation: rot,
            scale: Vec3::ONE,
        };

        let axis = Vec3::new(
            joint.axis.xyz[0] as f32,
            joint.axis.xyz[1] as f32,
            joint.axis.xyz[2] as f32,
        )
        .normalize_or_zero();

        let lower_limit = joint.limit.lower as f32;
        let upper_limit = joint.limit.upper as f32;

        let max_effort = if joint.name.contains("kn") {
            115.0
        } else {
            45.0
        };

        let inertia = if joint.name.contains("kn") {
            0.04
        } else if joint.name.contains("hy") {
            0.08
        } else {
            0.12
        };

        let joint_idx = joints.len();
        joint_name_to_idx.insert(joint.name.clone(), joint_idx);

        joints.push(RobotJoint {
            name: joint.name.clone(),
            parent_link,
            child_link,
            origin_transform,
            axis,
            angle: 0.0,
            velocity: 0.0,
            desired_angle: 0.0,
            default_angle: 0.0,
            lower_limit,
            upper_limit,
            max_effort,
            inertia,
        });

        links[parent_link].child_joints.push(joint_idx);
        links[child_link].parent_joint = Some(joint_idx);
    }

    let root_link = *link_name_to_idx
        .get("root")
        .or_else(|| link_name_to_idx.get("body"))
        .unwrap_or(&0);

    // Place robot body at height 0.65m drop height
    let root_transform = Transform {
        translation: Vec3::new(0.0, 0.65, 0.0),
        rotation: urdf_to_bevy_orientation(),
        scale: Vec3::ONE,
    };

    let mut model = RobotModel {
        name: robot.name,
        links,
        joints,
        root_link,
        root_transform,
        link_name_to_idx,
        joint_name_to_idx,
    };

    // Initialize default standing pose
    model.reset_to_default_pose();

    Ok(model)
}
