use std::path::Path;

use super::types::*;
use bevy::prelude::*;
use bevy_rapier3d::dynamics::FixedJointBuilder;
use bevy_rapier3d::dynamics::TypedJoint;
use bevy_rapier3d::prelude::*;
use bevy_rapier3d::rapier::dynamics::MassProperties as RapierMassProperties;
use urdf_rs::Robot as RobotURDF;

pub struct URDF {}

// Bevy uses a Y-up coordinate system, so we need to convert URDF's Z-up to Bevy's Y-up
pub const URDF_TO_BEVY_ROT: Transform =
    Transform::from_rotation(Quat::from_xyzw(-0.5, 0.5, 0.5, 0.5));

impl Parse for URDF {
    fn parse(file_path: &Path, asset_server: &AssetServer) -> RobotBlueprint {
        let robot: RobotURDF = urdf_rs::read_file(file_path).unwrap();
        let mut links: Vec<LinkBlueprint> = Vec::new();
        let mut joints: Vec<JointBlueprint> = Vec::new();
        let mut name_to_joint: std::collections::HashMap<String, usize> =
            std::collections::HashMap::new();
        let mut name_to_link: std::collections::HashMap<String, usize> =
            std::collections::HashMap::new();

        // Create the link blueprints
        for link in robot.links {
            let mut visuals: Vec<Handle<WorldAsset>> = Vec::new();
            for visual in link.visual {
                visuals.push(urdf_geometry_to_bevy_asset(visual.geometry, &asset_server));
            }
            let euler = link.inertial.origin.rpy.to_bevy();
            let rotation = Quat::from_euler(EulerRot::XYZ, euler.x, euler.y, euler.z);
            let local_transform = Transform::from_translation(link.inertial.origin.xyz.to_bevy())
                .with_rotation(rotation);
            let link_blueprint = LinkBlueprint {
                name: link.name.clone(),
                additional_mass_properties: inertial_to_additional_mass_properties(&link.inertial),
                visuals: visuals,
                parent_joint: None,
                children_joints: Vec::new(),
                world_transform: local_transform,
            };
            links.push(link_blueprint);
            name_to_link.insert(link.name, links.len() - 1);
        }

        // Create the joint blueprints
        for joint in robot.joints {
            let parent_link = name_to_link[&joint.parent.link];
            let child_link = name_to_link[&joint.child.link];
            let joint_data = urdf_joint_to_typed_joint(&joint);
            let euler = joint.origin.rpy.to_bevy();
            let rotation = Quat::from_euler(EulerRot::XYZ, euler.x, euler.y, euler.z);
            let local_transform =
                Transform::from_translation(joint.origin.xyz.to_bevy()).with_rotation(rotation);
            let joint_blueprint = JointBlueprint {
                name: joint.name.clone(),
                joint_data,
                parent_link,
                child_link,
                world_transform: local_transform,
            };
            joints.push(joint_blueprint);
            name_to_joint.insert(joint.name, joints.len() - 1);

            // fill parent and child link fields
            links[parent_link].children_joints.push(joints.len() - 1);
            links[child_link].parent_joint = Some(joints.len() - 1);
        }

        // if there is a root link, convert it to bevy coordinates
        // we will assume the root link is the first link and is not transformed
        if let Some(root_link) = links.get_mut(0) {
            root_link.world_transform = URDF_TO_BEVY_ROT;
        }

        // Compute world transforms
        for link in &mut links {
            if let Some(parent_joint) = link.parent_joint {
                link.world_transform = joints[parent_joint].world_transform * link.world_transform;
            }
            for child_joint in &link.children_joints {
                joints[*child_joint].world_transform =
                    link.world_transform * joints[*child_joint].world_transform;
            }
        }

        RobotBlueprint {
            name: robot.name,
            joints,
            links,
        }
    }
}

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
    let r_rpy = Mat3::from_euler(
        EulerRot::XYZ,
        inertial.origin.rpy[0] as f32,
        inertial.origin.rpy[1] as f32,
        inertial.origin.rpy[2] as f32,
    );

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
    let joint_rpy = joint.origin.rpy.to_bevy();
    let local_anchor1 = joint.origin.xyz.to_bevy();
    let local_anchor2 = Vec3::ZERO;
    let local_basis1 = Quat::from_euler(EulerRot::XYZ, joint_rpy[0], joint_rpy[1], joint_rpy[2]);
    let local_basis2 = Quat::IDENTITY;

    match joint.joint_type {
        urdf_rs::JointType::Fixed => {
            let joint = FixedJointBuilder::new()
                .local_anchor1(local_anchor1)
                .local_anchor2(local_anchor2)
                .local_basis1(local_basis1)
                .local_basis2(local_basis2)
                .build();
            TypedJoint::FixedJoint(joint)
        }
        urdf_rs::JointType::Revolute => {
            let axis: Vec3 = joint.axis.xyz.to_bevy();
            let joint = RevoluteJointBuilder::new(axis)
                .local_anchor1(local_anchor1)
                .local_anchor2(local_anchor2)
                .build();
            TypedJoint::RevoluteJoint(joint)
        }
        _ => {
            todo!()
        }
    }
}
