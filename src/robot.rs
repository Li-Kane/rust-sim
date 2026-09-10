pub mod types;
pub mod urdf_loader;

use bevy::prelude::*;
use bevy_rapier3d::prelude::*;
use types::{JointEntity, RobotBlueprint, RobotEntity};

use crate::robot::types::RobotPose;

pub struct RobotPlugin;

impl Plugin for RobotPlugin {
    fn build(&self, app: &mut App) {
        app.add_systems(Update, apply_robot_pose);
    }
}

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
        links,
        joints,
        ..
    } = robot_blueprint;
    let mut link_entities: Vec<Entity> = Vec::new();
    let mut joint_world_transforms = vec![Transform::IDENTITY; joints.len()];
    let mut joint_entities: Vec<JointEntity> = Vec::new();
    let mut num_dofs = 0;

    // Load each robot link and collect entity IDs
    for link in links {
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
        let link_entity = commands
            .spawn((
                RigidBody::Dynamic,
                world_transform,
                Visibility::default(),
                link.additional_mass_properties,
                Name::new(link.name),
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
            .id();
        link_entities.push(link_entity);
    }

    // Load each robot joint
    for joint in joints {
        let parent_entity = link_entities[joint.parent_link];
        let child_entity = link_entities[joint.child_link];
        let multibody_joint = MultibodyJoint::new(parent_entity, joint.joint_data);
        commands.entity(child_entity).insert(multibody_joint);

        let joint_entity =
            create_joint_entity(joint.name, &multibody_joint, child_entity, &mut num_dofs);
        joint_entities.push(joint_entity);
    }

    // spawn a Robot and RobotPose component and attach it to our robot entity
    let initial_positions = initial_pose.unwrap_or_default();
    commands.spawn((
        RobotEntity {
            name,
            joints: joint_entities,
            num_dofs: num_dofs,
        },
        RobotPose {
            positions: initial_positions.into(),
        },
    ));
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

/// Applies the robot pose to the multibody joints.
pub fn apply_robot_pose(
    robot_query: Query<(&RobotEntity, &RobotPose), Changed<RobotPose>>,
    mut joint_query: Query<&mut MultibodyJoint>,
) {
    for (robot, pose) in robot_query.iter() {
        // size check
        if pose.positions.len() != robot.num_dofs {
            warn!(
                "Pose length {} does not match robot num_dofs {} for '{}'",
                pose.positions.len(),
                robot.num_dofs,
                robot.name
            );
            continue;
        }

        // Apply the robot pose to the multibody joints
        let mut dof_idx = 0;
        for joint in &robot.joints {
            if let Ok(mut multibody_joint) = joint_query.get_mut(joint.entity) {
                let generic = &mut multibody_joint.data.as_mut().raw;
                for dof in &joint.dofs {
                    generic.motors[*dof].target_pos = pose.positions[dof_idx];
                    dof_idx += 1;
                }
            }
        }
    }
}
