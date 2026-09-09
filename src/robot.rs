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
    let root_transform = root_transform.unwrap_or(Transform::IDENTITY);
    let RobotBlueprint { links, joints, .. } = robot_blueprint;
    let mut joint_world_transforms = vec![Transform::IDENTITY; joints.len()];

    // Load each robot link and collect entity IDs
    let link_entities: Vec<Entity> = links
        .into_iter()
        .map(|link| {
            // Compute the world transform using forward kinematics
            let world_transform = if let Some(parent_joint) = link.parent_joint {
                joint_world_transforms[parent_joint] * link.local_transform
            } else {
                root_transform * link.local_transform
            };

            // Compute child joint world transforms
            for &child_joint in &link.children_joints {
                joint_world_transforms[child_joint] =
                    world_transform * joints[child_joint].local_transform;
            }

            // spawn the link entity
            commands
                .spawn((
                    RigidBody::Dynamic,
                    world_transform,
                    Visibility::default(),
                    link.additional_mass_properties,
                ))
                .with_children(|parent| {
                    for visual in link.visuals {
                        parent.spawn(WorldAssetRoot(visual));
                    }

                    for collision in link.collisions {
                        parent.spawn((
                            WorldAssetRoot(collision),
                            AsyncSceneCollider::default(),
                            Visibility::Hidden,
                        ));
                    }
                })
                .id()
        })
        .collect();

    // Load each robot joint
    for joint in joints {
        let parent_entity = link_entities[joint.parent_link];
        let child_entity = link_entities[joint.child_link];
        let multibody_joint = MultibodyJoint::new(parent_entity, joint.joint_data);
        commands.entity(child_entity).insert(multibody_joint);
    }
}
