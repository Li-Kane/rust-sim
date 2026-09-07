pub mod kinematics;
pub mod loader;
pub mod mesh;
pub mod types;

pub use loader::load_spot_model;
pub use mesh::*;
pub use types::*;

use std::collections::HashMap;
use bevy::prelude::*;
use bevy_rapier3d::prelude::*;

/// Spawns the Spot robot into the Bevy ECS scene using Rapier Multibody Joints
pub fn setup_robot(
    mut commands: Commands,
    mut meshes: ResMut<Assets<Mesh>>,
    mut materials: ResMut<Assets<StandardMaterial>>,
) {
    let model = match load_spot_model() {
        Ok(m) => m,
        Err(err) => {
            error!("Failed to load Spot URDF model: {err}");
            return;
        }
    };

    // Load and cache the 4 OBJ visual meshes
    let body_mesh = parse_obj_mesh(BODY_OBJ)
        .map(|m| meshes.add(m))
        .expect("body.obj parse failed");
    let hip_mesh = parse_obj_mesh(HIP_OBJ)
        .map(|m| meshes.add(m))
        .expect("hip.obj parse failed");
    let uleg_mesh = parse_obj_mesh(ULEG_OBJ)
        .map(|m| meshes.add(m))
        .expect("uleg.obj parse failed");
    let lleg_mesh = parse_obj_mesh(LLEG_OBJ)
        .map(|m| meshes.add(m))
        .expect("lleg.obj parse failed");

    let mut link_idx_to_entity: HashMap<usize, Entity> = HashMap::new();

    // 1. Spawn floating base (torso body)
    let body_link_idx = *model
        .link_name_to_idx
        .get("body")
        .unwrap_or(&model.root_link);
    let body_link = &model.links[body_link_idx];

    let body_material = materials.add(StandardMaterial {
        base_color: body_link.color,
        metallic: 0.2,
        perceptual_roughness: 0.4,
        ..default()
    });

    let body_entity = commands
        .spawn((
            LinkEntityIndex(body_link_idx),
            RobotBaseMarker,
            Name::new("body"),
            RigidBody::Dynamic,
            Collider::cuboid(
                BODY_BOX_SIZE.x * 0.5,
                BODY_BOX_SIZE.y * 0.5,
                BODY_BOX_SIZE.z * 0.5,
            ),
            Friction::coefficient(0.8),
            Restitution::coefficient(0.0),
            AdditionalMassProperties::Mass(body_link.mass),
            Velocity::zero(),
            body_link.world_transform,
            Mesh3d(body_mesh),
            MeshMaterial3d(body_material),
        ))
        .id();

    link_idx_to_entity.insert(body_link_idx, body_entity);
    if model.root_link != body_link_idx {
        link_idx_to_entity.insert(model.root_link, body_entity);
    }

    // 2. Spawn each child leg link and its MultibodyJoint
    for (joint_idx, joint) in model.joints.iter().enumerate() {
        if joint.name == "root_joint" || joint.name == "body_inertia_joint" {
            continue;
        }

        let Some(&parent_entity) = link_idx_to_entity.get(&joint.parent_link) else {
            error!(
                "Parent link {} not spawned yet for joint {}",
                joint.parent_link, joint.name
            );
            continue;
        };

        let child_link = &model.links[joint.child_link];

        let (mesh_handle, collider, friction) = match child_link.mesh_name.as_deref() {
            Some("hip.obj") => (
                hip_mesh.clone(),
                Collider::ball(0.04),
                0.5,
            ),
            Some("uleg.obj") => (
                uleg_mesh.clone(),
                Collider::capsule(Vec3::ZERO, Vec3::new(0.025, 0.0, -0.32), 0.025),
                0.5,
            ),
            Some("lleg.obj") => (
                lleg_mesh.clone(),
                Collider::capsule(Vec3::ZERO, Vec3::new(0.0, 0.0, -0.34), FOOT_RADIUS),
                1.0,
            ),
            _ => (
                hip_mesh.clone(),
                Collider::ball(0.04),
                0.5,
            ),
        };

        let material = materials.add(StandardMaterial {
            base_color: child_link.color,
            metallic: 0.2,
            perceptual_roughness: 0.4,
            ..default()
        });

        let rev_joint = RevoluteJointBuilder::new(joint.axis)
            .local_anchor1(joint.origin_transform.translation)
            .local_anchor2(Vec3::ZERO)
            .limits([joint.lower_limit, joint.upper_limit])
            .motor_position(joint.desired_angle, 80.0, 1.5)
            .motor_max_force(joint.max_effort)
            .build();

        let child_entity = commands
            .spawn((
                LinkEntityIndex(joint.child_link),
                RobotJointIndex(joint_idx),
                Name::new(child_link.name.clone()),
                RigidBody::Dynamic,
                collider,
                Friction::coefficient(friction),
                Restitution::coefficient(0.0),
                AdditionalMassProperties::Mass(child_link.mass),
                Velocity::zero(),
                child_link.world_transform,
                Mesh3d(mesh_handle),
                MeshMaterial3d(material),
                MultibodyJoint::new(parent_entity, rev_joint.into()),
            ))
            .id();

        link_idx_to_entity.insert(joint.child_link, child_entity);
    }

    commands.insert_resource(model);
}

pub struct RobotPlugin;

impl Plugin for RobotPlugin {
    fn build(&self, app: &mut App) {
        app.add_systems(Startup, setup_robot);
    }
}
