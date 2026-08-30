use bevy::prelude::*;

use crate::physics::types::{BODY_BOX_SIZE, PhysicsSettings};
use crate::robot::RobotModel;

/// System to draw bright red wireframe colliders (foot spheres + body bounding box)
pub fn debug_draw_colliders_system(
    mut gizmos: Gizmos,
    robot: Res<RobotModel>,
    settings: Res<PhysicsSettings>,
) {
    if !settings.show_colliders {
        return;
    }

    let bright_red = Color::srgb(1.0, 0.0, 0.0);

    // 1. Draw 4 foot contact spheres
    let foot_links = ["fl_lleg", "fr_lleg", "hl_lleg", "hr_lleg"];
    for link_name in foot_links {
        if let Some(&idx) = robot.link_name_to_idx.get(link_name) {
            let link_transform = robot.links[idx].world_transform;
            let foot_world_pos = link_transform.transform_point(settings.foot_offset);
            gizmos.sphere(
                Isometry3d::from_translation(foot_world_pos),
                settings.foot_radius,
                bright_red,
            );
        }
    }

    // 2. Draw base body oriented bounding box
    let body_transform = robot.links[robot.root_link].world_transform;
    let body_box_world = Transform {
        translation: body_transform.translation,
        rotation: body_transform.rotation,
        scale: BODY_BOX_SIZE,
    };
    gizmos.cube(body_box_world, bright_red);
}
