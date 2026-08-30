use bevy::prelude::*;

use crate::physics::types::{BODY_BOX_SIZE, ContactInfo, PhysicsSettings};
use crate::robot::RobotModel;

/// Returns world contact points for all 4 lower leg feet and 8 body box corners
pub fn get_robot_contact_points(
    robot: &RobotModel,
    settings: &PhysicsSettings,
) -> Vec<ContactInfo> {
    let mut points = Vec::with_capacity(12);

    // 1. Four foot contact points on each lower leg (fl_lleg, fr_lleg, hl_lleg, hr_lleg)
    let foot_links = ["fl_lleg", "fr_lleg", "hl_lleg", "hr_lleg"];
    for link_name in foot_links {
        if let Some(&idx) = robot.link_name_to_idx.get(link_name) {
            let link_transform = robot.links[idx].world_transform;
            let foot_world_pos = link_transform.transform_point(settings.foot_offset);
            points.push(ContactInfo {
                world_pos: foot_world_pos,
                radius: settings.foot_radius,
                link_idx: idx,
            });
        }
    }

    // 2. Eight base body box corner contact points
    let half = BODY_BOX_SIZE * 0.5;
    let body_transform = robot.links[robot.root_link].world_transform;
    for &sx in &[-1.0, 1.0] {
        for &sy in &[-1.0, 1.0] {
            for &sz in &[-1.0, 1.0] {
                let local_corner = Vec3::new(sx * half.x, sy * half.y, sz * half.z);
                let corner_world_pos = body_transform.transform_point(local_corner);
                points.push(ContactInfo {
                    world_pos: corner_world_pos,
                    radius: 0.01,
                    link_idx: robot.root_link,
                });
            }
        }
    }

    points
}

/// Evaluates the Hunt-Crossley normal force and Coulomb friction for a single contact point
pub fn compute_contact_force(
    contact: &ContactInfo,
    pt_vel: Vec3,
    settings: &PhysicsSettings,
) -> Option<Vec3> {
    let lowest_y = contact.world_pos.y - contact.radius;
    let penetration = settings.ground_y - lowest_y;

    if penetration <= 0.0 {
        return None;
    }

    let v_normal = -pt_vel.y; // Positive when penetrating deeper into the ground

    // Hunt-Crossley normal force: Fn = max(0, kp * delta^1.5 + kc * delta^1.5 * v_n)
    let delta_pow = penetration.powf(settings.exponent);
    let fn_raw = settings.kp * delta_pow + settings.kc * delta_pow * v_normal;
    let fn_mag = fn_raw.max(0.0);

    let normal_force = Vec3::new(0.0, fn_mag, 0.0);

    // Tangential friction force (Coulomb regularized)
    let v_tangent = Vec3::new(pt_vel.x, 0.0, pt_vel.z);
    let v_tangent_mag = v_tangent.length();
    let max_friction = settings.mu * fn_mag;
    let friction_mag = max_friction.min(v_tangent_mag * 12_000.0);

    let friction_force = if v_tangent_mag > 1e-4 {
        -v_tangent.normalize() * friction_mag
    } else {
        -v_tangent * 12_000.0
    };

    Some(normal_force + friction_force)
}

/// Transmits contact force up the kinematic chain into joint torques via Jacobian transpose
pub fn propagate_contact_force_to_joints(
    robot: &RobotModel,
    link_idx: usize,
    contact_world_pos: Vec3,
    contact_force: Vec3,
    joint_torques: &mut [f32],
) {
    let mut link_cursor = link_idx;
    while let Some(j_idx) = robot.links[link_cursor].parent_joint {
        let parent_link_idx = robot.joints[j_idx].parent_link;
        let parent_world = robot.links[parent_link_idx].world_transform;
        let joint_axis_world = parent_world.rotation * robot.joints[j_idx].axis;
        let joint_pos_world = parent_world.transform_point(robot.joints[j_idx].origin_transform.translation);

        let arm = contact_world_pos - joint_pos_world;
        let tau_ext = joint_axis_world.dot(arm.cross(contact_force));
        joint_torques[j_idx] += tau_ext;

        link_cursor = parent_link_idx;
        if link_cursor == robot.root_link {
            break;
        }
    }
}
