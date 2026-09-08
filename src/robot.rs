pub mod types;
pub mod urdf_loader;

use bevy::prelude::*;
use bevy_rapier3d::prelude::*;
use types::RobotBlueprint;

/// Spawns a robot from a RobotBlueprint and spawns it in the scene.
pub fn spawn_robot(mut commands: Commands, robot_blueprint: RobotBlueprint) {
    // load each robot link
    let mut link_map: std::collections::HashMap<String, Entity> = std::collections::HashMap::new();
    for link in robot_blueprint.links {
        // add a rigid body for this link with its initial world transform
        let mut link_cmd = commands.spawn(RigidBody::Dynamic);

        // Apply inertial properties from the URDF Link
        link_cmd.insert(link.additional_mass_properties);

        let link_entity = link_cmd.id();

        // add visual meshes for this link
        for visual in link.visuals {
            let mesh = commands.spawn(WorldAssetRoot(visual)).id();
            commands.entity(link_entity).add_child(mesh);
        }

        // TODO: add collision meshes for this link

        link_map.insert(link.name, link_entity);
    }

    // load each robot joint
    for joint in robot_blueprint.joints {
        // ROS URDF expects a link -> joint -> link hierarchy
        let parent_entity = link_map.get(&joint.parent).unwrap();
        let child_entity = link_map.get(&joint.child).unwrap();
        let multibody_joint = MultibodyJoint::new(*parent_entity, joint.joint_data);
        commands.entity(*child_entity).insert(multibody_joint);
    }
}
