use std::collections::HashMap;
use std::io::Read;

use super::types::*;
use bevy::prelude::*;
use bevy_rapier3d::prelude::*;
use bevy_rapier3d::rapier::dynamics::MassProperties as RapierMassProperties;
use urdf_rs::Robot as RobotURDF;

pub struct UrdfLoader {}

impl Parse for UrdfLoader {
    /// Parses a URDF file from a reader and returns a [`RobotBlueprint`].
    fn parse<R: Read>(mut reader: R, asset_server: &AssetServer) -> RobotBlueprint {
        let mut urdf_str = String::new();
        reader
            .read_to_string(&mut urdf_str)
            .expect("Failed to read URDF stream");
        let robot: RobotURDF = urdf_rs::read_from_string(&urdf_str).expect("Failed to parse URDF");

        let name_to_link: HashMap<String, usize> = robot
            .links
            .iter()
            .enumerate()
            .map(|(idx, link)| (link.name.clone(), idx))
            .collect();

        let mut links: Vec<LinkBlueprint> = robot
            .links
            .into_iter()
            .map(|link| urdf_link_to_blueprint(link, asset_server))
            .collect();

        let mut joints = Vec::with_capacity(robot.joints.len());
        for (joint_idx, joint) in robot.joints.into_iter().enumerate() {
            let parent_link = name_to_link[&joint.parent.link];
            let child_link = name_to_link[&joint.child.link];
            let local_transform = urdf_pose_to_transform(&joint.origin);
            let joint_data = urdf_joint_to_typed_joint(&joint);

            joints.push(JointBlueprint {
                name: joint.name,
                joint_data,
                parent_link,
                child_link,
                local_transform,
            });

            links[parent_link].children_joints.push(joint_idx);
            links[child_link].parent_joint = Some(joint_idx);
        }

        // convert root link to bevy coordinates
        let root_link = links
            .iter()
            .position(|l| l.parent_joint.is_none())
            .expect("Failed to find root link in URDF");
        links[root_link].local_transform = URDF_TO_BEVY_ROT;

        RobotBlueprint {
            name: robot.name,
            root_link,
            joints,
            links,
        }
    }
}

/// Bevy uses a Y-up coordinate system, so we need to convert URDF's Z-up to Bevy's Y-up
pub const URDF_TO_BEVY_ROT: Transform =
    Transform::from_rotation(Quat::from_xyzw(-0.5, 0.5, 0.5, 0.5));

/// Helper trait to convert a urdff-rs [`Vec3`] to a Bevy [`Vec3`].
pub trait UrdfVec3Ext {
    fn to_bevy(&self) -> Vec3;
}

/// Converts a urdf-rs [`Vec3`] to a Bevy [`Vec3`].
impl UrdfVec3Ext for urdf_rs::Vec3 {
    fn to_bevy(&self) -> Vec3 {
        Vec3::new(self[0] as f32, self[1] as f32, self[2] as f32)
    }
}

/// Converts a urdf-rs [`Link`] to a [`LinkBlueprint`].
fn urdf_link_to_blueprint(link: urdf_rs::Link, asset_server: &AssetServer) -> LinkBlueprint {
    let visuals = link
        .visual
        .into_iter()
        .map(|visual| urdf_geometry_to_bevy_asset(visual.geometry, asset_server))
        .collect();

    let collisions = link
        .collision
        .into_iter()
        .map(|collision| urdf_geometry_to_bevy_asset(collision.geometry, asset_server))
        .collect();

    let local_transform = urdf_pose_to_transform(&link.inertial.origin);

    LinkBlueprint {
        name: link.name,
        additional_mass_properties: inertial_to_additional_mass_properties(&link.inertial),
        visuals,
        collisions,
        parent_joint: None,
        children_joints: Vec::new(),
        local_transform,
    }
}

/// Converts a urdf-rs [`Pose`] to a Bevy [`Transform`].
pub fn urdf_pose_to_transform(origin: &urdf_rs::Pose) -> Transform {
    let euler = origin.rpy.to_bevy();
    let rotation = Quat::from_euler(EulerRot::XYZ, euler.x, euler.y, euler.z);
    Transform::from_translation(origin.xyz.to_bevy()).with_rotation(rotation)
}

/// Converts an urdf-rs [`Inertial`] struct to a Bevy Rapier [`AdditionalMassProperties`] struct.
pub fn inertial_to_additional_mass_properties(
    inertial: &urdf_rs::Inertial,
) -> AdditionalMassProperties {
    let mass = inertial.mass.value as f32;

    // URDF center of mass in URDF coords
    let local_com = inertial.origin.xyz.to_bevy();

    // URDF inertia matrix in the inertial frame
    let i = &inertial.inertia;
    let urdf_inertia = Mat3::from_cols(
        Vec3::new(i.ixx as f32, i.ixy as f32, i.ixz as f32),
        Vec3::new(i.ixy as f32, i.iyy as f32, i.iyz as f32),
        Vec3::new(i.ixz as f32, i.iyz as f32, i.izz as f32),
    );

    // Orientation of the inertial frame relative to the URDF link frame
    let euler = inertial.origin.rpy.to_bevy();
    let r_rpy = Mat3::from_euler(EulerRot::XYZ, euler.x, euler.y, euler.z);

    // Transform inertia matrix to Bevy coordinate frame: I_bevy = R_total * I_urdf * R_total^T
    let bevy_inertia = r_rpy * urdf_inertia * r_rpy.transpose();

    let rapier_mprops = RapierMassProperties::with_inertia_matrix(local_com, mass, bevy_inertia);
    AdditionalMassProperties::MassProperties(MassProperties::from_rapier(rapier_mprops))
}

/// Converts a urdf-rs ['Geometry'] to a Bevy [`Handle<WorldAsset>`].
pub fn urdf_geometry_to_bevy_asset(
    geometry: urdf_rs::Geometry,
    asset_server: &AssetServer,
) -> Handle<WorldAsset> {
    match geometry {
        urdf_rs::Geometry::Mesh { filename, .. } => {
            asset_server.load(GltfAssetLabel::Scene(0).from_asset(filename))
        }
        urdf_rs::Geometry::Box { .. } => {
            todo!()
        }
        urdf_rs::Geometry::Cylinder { .. } => {
            todo!()
        }
        urdf_rs::Geometry::Sphere { .. } => {
            todo!()
        }
        urdf_rs::Geometry::Capsule { .. } => {
            todo!()
        }
    }
}

/// Converts a urdf-rs [`Joint`] to a Bevy Rapier [`TypedJoint`].
pub fn urdf_joint_to_typed_joint(joint: &urdf_rs::Joint) -> TypedJoint {
    let origin = urdf_pose_to_transform(&joint.origin);
    let local_anchor1 = origin.translation;
    let local_basis1 = origin.rotation;
    let lower = joint.limit.lower as f32;
    let upper = joint.limit.upper as f32;
    let effort = joint.limit.effort as f32;

    // TODO: Handle dynamics, mimic, and safety controller

    match joint.joint_type {
        urdf_rs::JointType::Fixed => {
            let joint = FixedJointBuilder::new()
                .local_anchor1(local_anchor1)
                .local_basis1(local_basis1)
                .build();
            TypedJoint::FixedJoint(joint)
        }
        urdf_rs::JointType::Revolute => {
            let axis: Vec3 = joint.axis.xyz.to_bevy();
            let joint = RevoluteJointBuilder::new(axis)
                .local_anchor1(local_anchor1)
                .limits([lower, upper])
                .motor_position(0.0, 300.0, 20.0)
                .motor_max_force(effort)
                .build();
            TypedJoint::RevoluteJoint(joint)
        }
        _ => {
            todo!()
        }
    }
}
