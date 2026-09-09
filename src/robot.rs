pub mod types;
pub mod urdf_loader;

use bevy::prelude::*;
use bevy_rapier3d::prelude::*;
use types::RobotBlueprint;

/// Spawns a robot from a RobotBlueprint and spawns it in the scene.
/// Rapier3D joint motors use world transforms so we need to convert local joint transforms to world transforms.
pub fn spawn_robot(
    mut commands: Commands,
    robot_blueprint: RobotBlueprint,
    root_transform: Option<Transform>,
) {
    // load each robot link
    let mut link_entities: Vec<Entity> = Vec::new();
    for (idx, link) in robot_blueprint.links.into_iter().enumerate() {
        // if a root transform is provided, apply it to the robot's root link
        let link_transform = if idx == 0
            && let Some(root_transform) = root_transform
        {
            root_transform * link.world_transform
        } else {
            link.world_transform
        };

        // add a rigid body for this link with its initial world transform
        let mut link_cmd = commands.spawn((RigidBody::Dynamic, link_transform));

        // Apply inertial properties from the URDF Link
        link_cmd.insert(link.additional_mass_properties);

        let link_entity = link_cmd.id();

        // add visual meshes for this link
        for visual in link.visuals {
            let mesh = commands.spawn(WorldAssetRoot(visual)).id();
            commands.entity(link_entity).add_child(mesh);
        }

        // add collision meshes for this link
        for collision in link.collisions {
            let mesh = commands
                .spawn((
                    WorldAssetRoot(collision),
                    AsyncSceneCollider::default(),
                    Visibility::Hidden,
                ))
                .id();
            commands.entity(link_entity).add_child(mesh);
        }

        link_entities.push(link_entity);
    }

    // load each robot joint
    for joint in robot_blueprint.joints {
        // ROS URDF expects a link -> joint -> link hierarchy
        let parent_entity = link_entities.get(joint.parent_link).unwrap();
        let child_entity = link_entities.get(joint.child_link).unwrap();
        let multibody_joint = MultibodyJoint::new(*parent_entity, joint.joint_data);
        commands.entity(*child_entity).insert(multibody_joint);
    }
}
