use bevy::prelude::*;
use bevy_rapier3d::prelude::*;

use super::kinematics::compute_link_world_transforms;
use super::types::{
    JointBlueprint, JointEntity, LinkBlueprint, RobotBlueprint, RobotEntity, RobotPose,
};

/// Spawns a robot from a RobotBlueprint and spawns it in the scene.
/// Rapier3D joint motors use world transforms so we need to convert local joint transforms to world transforms.
pub fn spawn_robot(
    mut commands: Commands,
    robot_blueprint: RobotBlueprint,
    root_transform: Option<Transform>,
    initial_pose: Option<&[f32]>,
) {
    let root_transform = root_transform.unwrap_or(Transform::IDENTITY);
    let RobotBlueprint {
        name,
        root_link,
        links,
        joints,
    } = robot_blueprint;

    let link_transforms =
        compute_link_world_transforms(&links, &joints, root_link, root_transform);

    let link_entities: Vec<Entity> = links
        .iter()
        .zip(link_transforms)
        .map(|(link, xform)| spawn_link_entity(&mut commands, link, xform))
        .collect();

    let (joint_entities, num_dofs) = attach_robot_joints(&mut commands, &joints, &link_entities);

    // spawn a Robot and RobotPose component and attach it to our robot entity
    let initial_positions = initial_pose.unwrap_or_default();
    commands.spawn((
        RobotEntity {
            name,
            joints: joint_entities,
            num_dofs,
        },
        RobotPose {
            positions: initial_positions.into(),
        },
    ));
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
            world_transform,
            Visibility::default(),
            link.additional_mass_properties,
            Name::new(link.name.clone()),
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
                ));
            }
        })
        .id()
}

/// Attaches Rapier `MultibodyJoint` components to child link entities and maps joint DOFs.
fn attach_robot_joints(
    commands: &mut Commands,
    joints: &[JointBlueprint],
    link_entities: &[Entity],
) -> (Vec<JointEntity>, usize) {
    let mut joint_entities = Vec::with_capacity(joints.len());
    let mut num_dofs = 0;

    for joint in joints {
        let parent_entity = link_entities[joint.parent_link];
        let child_entity = link_entities[joint.child_link];
        let multibody_joint = MultibodyJoint::new(parent_entity, joint.joint_data);
        commands.entity(child_entity).insert(multibody_joint);

        let joint_entity = create_joint_entity(
            joint.name.clone(),
            &multibody_joint,
            child_entity,
            &mut num_dofs,
        );
        joint_entities.push(joint_entity);
    }

    (joint_entities, num_dofs)
}

/// Create a [`JointEntity`] from a [`MultibodyJoint`].
pub fn create_joint_entity(
    name: String,
    multibody_joint: &MultibodyJoint,
    child_entity: Entity,
    num_dofs: &mut usize,
) -> JointEntity {
    let dof_mask = multibody_joint.data.as_ref().raw.motor_axes;
    let dofs: Vec<usize> = (0..6)
        .filter(|&i| (dof_mask.bits() & (1 << i)) != 0)
        .collect();
    *num_dofs += dofs.len();
    JointEntity {
        name,
        entity: child_entity,
        dofs,
    }
}
