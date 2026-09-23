use bevy::prelude::*;
use bevy_rapier3d::prelude::*;

use crate::robot::types::SpawnPose;

use super::kinematics::compute_link_world_transforms;
use super::types::{
    JointBlueprint, JointRef, LinkBlueprint, Robot, RobotBlueprint, RobotJoints, RobotName,
    RobotPose,
};

/// Spawns a robot from a RobotBlueprint and spawns it in the scene.
/// Rapier3D joint motors use world transforms so we need to convert local joint transforms to world transforms.
pub fn spawn_robot(
    commands: &mut Commands,
    robot_blueprint: RobotBlueprint,
    spawn_pose: Option<SpawnPose>,
) -> Entity {
    let spawn_pose = spawn_pose.unwrap_or_default();
    let root_transform = spawn_pose.root_transform;
    let initial_pose = spawn_pose.positions.clone();
    let RobotBlueprint {
        name,
        root_link,
        links,
        joints,
    } = robot_blueprint;

    let link_transforms = compute_link_world_transforms(&links, &joints, root_link, root_transform);

    let link_entities: Vec<Entity> = links
        .iter()
        .zip(link_transforms)
        .map(|(link, xform)| spawn_link_entity(commands, link, xform))
        .collect();

    let robot_joints: RobotJoints = create_robot_joints(commands, &joints, &link_entities);

    // spawn a Robot and RobotPose component and attach it to our robot entity
    commands
        .spawn((
            Robot {
                root: link_entities[root_link],
            },
            RobotName(name),
            robot_joints,
            RobotPose {
                positions: initial_pose.into(),
            },
            spawn_pose,
        ))
        .id()
}

/// Spawns a single robot link entity with rigid body, visuals, and colliders.
fn spawn_link_entity(
    commands: &mut Commands,
    link: &LinkBlueprint,
    world_transform: Transform,
) -> Entity {
    commands
        .spawn((
            RigidBody::Dynamic,
            Velocity::default(),
            world_transform,
            Visibility::default(),
            link.additional_mass_properties,
            Name::new(link.name.clone()),
            Sleeping::disabled(), // TODO: Properly use sleeping system
        ))
        .with_children(|parent| {
            for visual in &link.visuals {
                parent.spawn(WorldAssetRoot(visual.clone()));
            }

            for collision in &link.collisions {
                parent.spawn((
                    WorldAssetRoot(collision.clone()),
                    AsyncSceneCollider {
                        shape: Some(ComputedColliderShape::ConvexHull),
                        ..default()
                    },
                    Visibility::Hidden,
                    Friction::coefficient(1.0),
                ));
            }
        })
        .id()
}

/// Attaches Rapier `MultibodyJoint` components to child link entities and maps joint DOFs.
fn create_robot_joints(
    commands: &mut Commands,
    joints: &[JointBlueprint],
    link_entities: &[Entity],
) -> RobotJoints {
    let mut joint_refs = Vec::with_capacity(joints.len());
    let mut num_dofs = 0;

    for joint in joints {
        let parent_entity = link_entities[joint.parent_link];
        let child_entity = link_entities[joint.child_link];
        let multibody_joint = MultibodyJoint::new(parent_entity, joint.joint_data);
        commands.entity(child_entity).insert(multibody_joint);

        let joint_ref = create_joint_entity(
            joint.name.clone(),
            &multibody_joint,
            child_entity,
            &mut num_dofs,
        );
        joint_refs.push(joint_ref);
    }

    RobotJoints {
        joints: joint_refs,
        num_dofs,
    }
}

/// Create a [`JointEntity`] from a [`MultibodyJoint`].
pub fn create_joint_entity(
    name: String,
    multibody_joint: &MultibodyJoint,
    child_entity: Entity,
    num_dofs: &mut usize,
) -> JointRef {
    let dof_mask = multibody_joint.data.as_ref().raw.motor_axes;
    let dofs: Vec<JointAxesMask> = dof_mask.iter().collect();
    *num_dofs += dofs.len();
    JointRef {
        name,
        entity: child_entity,
        dofs,
    }
}
