use bevy::prelude::*;
use bevy_rapier3d::dynamics::FixedJointBuilder;
use bevy_rapier3d::dynamics::TypedJoint;
use bevy_rapier3d::prelude::*;
use bevy_rapier3d::rapier::dynamics::MassProperties as RapierMassProperties;
use urdf_rs::Robot as RobotURDF;

// Spot URDF content and mesh resources
pub const URDF_CONTENT: &str = include_str!("../../assets/spot_simple.urdf");
pub const URDF_TO_BEVY_MAT: Mat3 = Mat3::from_cols(
    Vec3::new(0.0, 0.0, -1.0),
    Vec3::new(-1.0, 0.0, 0.0),
    Vec3::new(0.0, 1.0, 0.0),
);

/// Loads a robot from a URDF string and spawns it in the scene.
pub fn load_robot(mut commands: Commands, asset_server: Res<AssetServer>) {
    // parse robot URDF
    let robot: RobotURDF = urdf_rs::read_from_string(URDF_CONTENT).unwrap();

    // load each robot link
    let mut link_map: std::collections::HashMap<String, Entity> = std::collections::HashMap::new();
    for link in robot.links {
        // add a rigid body for this link with its initial world transform
        let mut link_cmd = commands.spawn(RigidBody::Dynamic);

        // Apply inertial properties from the URDF Link
        link_cmd.insert(inertial_to_additional_mass_properties(&link.inertial));

        let link_entity = link_cmd.id();

        // add visual meshes for this link
        for visual in link.visual {
            if let urdf_rs::Geometry::Mesh { filename, .. } = visual.geometry {
                let mesh = asset_server.load(GltfAssetLabel::Scene(0).from_asset(filename));
                let visual_mesh = commands.spawn((WorldAssetRoot(mesh),)).id();
                commands.entity(link_entity).add_child(visual_mesh);
            }
        }

        // TODO: add collision meshes for this link

        link_map.insert(link.name, link_entity);
    }

    // load each robot joint
    for joint in robot.joints {
        // ROS URDF expects a link -> joint -> link hierarchy
        let parent_entity = link_map.get(&joint.parent.link).unwrap();
        let child_entity = link_map.get(&joint.child.link).unwrap();
        let joint_data = urdf_joint_to_typed_joint(&joint);
        let multibody_joint = MultibodyJoint::new(*parent_entity, joint_data);
        commands.entity(*child_entity).insert(multibody_joint);
    }
}

/// Helper trait to convert a urdff-rs [`Vec3`] to a Bevy [`Vec3`].
pub trait UrdfVec3Ext {
    fn to_bevy(&self) -> Vec3;
}

impl UrdfVec3Ext for urdf_rs::Vec3 {
    fn to_bevy(&self) -> Vec3 {
        Vec3::new(self[0] as f32, self[1] as f32, self[2] as f32)
    }
}

/// Converts an urdf-rs [`Inertial`] struct to a Bevy [`AdditionalMassProperties`] struct.
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

/// Converts a [`urdf_rs::Joint`] to a Bevy Rapier [`TypedJoint`].
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
