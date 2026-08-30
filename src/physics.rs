#![allow(dead_code)]
use bevy::prelude::*;

use crate::SimState;
use crate::robot::RobotModel;

/// Physics configuration parameters matching kdFlex / personal_website
#[derive(Resource, Debug, Clone)]
pub struct PhysicsSettings {
    pub gravity: Vec3,
    pub kp: f32,           // Normal spring stiffness (N/m^1.5)
    pub kc: f32,           // Normal damping (N*s/m^2.5)
    pub exponent: f32,     // Hunt-Crossley non-linear power n (1.5)
    pub mu: f32,           // Coulomb friction coefficient
    pub substeps: usize,   // Numerical integration sub-steps per frame
    pub ground_y: f32,     // Ground floor plane Y level
    pub foot_radius: f32,  // Foot collision sphere radius (m)
    pub foot_offset: Vec3, // Offset in URDF lower leg local frame
}

impl Default for PhysicsSettings {
    fn default() -> Self {
        Self {
            gravity: Vec3::new(0.0, -9.81, 0.0),
            kp: 500_000.0,
            kc: 200_000.0,
            exponent: 1.5,
            mu: 1.0,
            substeps: 30,
            ground_y: 0.0,
            foot_radius: 0.02,
            foot_offset: Vec3::new(0.0, 0.0, -0.34),
        }
    }
}

/// Floating base rigid body physics state for Spot
#[derive(Resource, Debug, Clone)]
pub struct PhysicsRigidBody {
    pub mass: f32,
    pub inertia: Vec3, // Diagonal inertia tensor in principal axes (Ixx, Iyy, Izz)
    pub linear_velocity: Vec3,
    pub angular_velocity: Vec3, // in world coordinates (rad/s)
}

impl Default for PhysicsRigidBody {
    fn default() -> Self {
        Self {
            mass: 45.0,
            inertia: Vec3::new(1.8, 3.2, 2.5),
            linear_velocity: Vec3::ZERO,
            angular_velocity: Vec3::ZERO,
        }
    }
}

impl PhysicsRigidBody {
    pub fn reset_velocities(&mut self) {
        self.linear_velocity = Vec3::ZERO;
        self.angular_velocity = Vec3::ZERO;
    }
}

/// Helper function to return world contact points for all 4 lower leg feet and 4 body corners
pub fn get_robot_contact_points(
    robot: &RobotModel,
    settings: &PhysicsSettings,
) -> Vec<(Vec3, f32)> {
    let mut points = Vec::with_capacity(8);

    // 1. Four foot contact points on each lower leg (fl_lleg, fr_lleg, hl_lleg, hr_lleg)
    let foot_links = ["fl_lleg", "fr_lleg", "hl_lleg", "hr_lleg"];
    for link_name in foot_links {
        if let Some(&idx) = robot.link_name_to_idx.get(link_name) {
            let link_transform = robot.links[idx].world_transform;
            let foot_world_pos = link_transform.transform_point(settings.foot_offset);
            points.push((foot_world_pos, settings.foot_radius));
        }
    }

    // 2. Four base body corner contact points (protective collision if base hits ground)
    let body_half_extents = Vec3::new(0.28, 0.08, 0.12);
    let body_transform = robot.links[robot.root_link].world_transform;
    for &sx in &[-1.0, 1.0] {
        for &sz in &[-1.0, 1.0] {
            let local_corner = Vec3::new(
                sx * body_half_extents.x,
                -body_half_extents.y,
                sz * body_half_extents.z,
            );
            let corner_world_pos = body_transform.transform_point(local_corner);
            points.push((corner_world_pos, 0.04));
        }
    }

    points
}

/// Physics stepping system using Hunt-Crossley contact model and Semi-Implicit Euler integration
pub fn physics_step_system(
    time: Res<Time>,
    mut robot: ResMut<RobotModel>,
    mut rb: ResMut<PhysicsRigidBody>,
    settings: Res<PhysicsSettings>,
) {
    let dt = time.delta_secs().min(0.033);
    if dt <= 0.0 {
        return;
    }

    let substeps = settings.substeps.max(1);
    let dt_sub = dt / substeps as f32;

    let mass = rb.mass.max(1.0);
    let inertia = rb.inertia;

    for _ in 0..substeps {
        // 1. Update forward kinematics to get updated link and contact transforms
        robot.update_kinematics();

        let com_pos = robot.root_transform.translation;
        let mut total_force = mass * settings.gravity;
        let mut total_torque = Vec3::ZERO;

        // 2. Evaluate ground contact forces for each contact point
        let contact_points = get_robot_contact_points(&robot, &settings);

        for (pt_center, radius) in contact_points {
            let lowest_y = pt_center.y - radius;
            let penetration = settings.ground_y - lowest_y;

            if penetration > 0.0 {
                let r_arm = pt_center - com_pos;
                // Velocity of contact point = v_com + w x r
                let pt_vel = rb.linear_velocity + rb.angular_velocity.cross(r_arm);
                let v_normal = -pt_vel.y; // positive when penetrating deeper

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

                let contact_force = normal_force + friction_force;
                total_force += contact_force;
                total_torque += r_arm.cross(contact_force);
            }
        }

        // 3. Add stabilizing aerodynamic/numerical drag
        let linear_damping = -0.3 * rb.linear_velocity;
        let angular_damping = -0.8 * rb.angular_velocity;
        total_force += linear_damping;
        total_torque += angular_damping;

        // 4. Semi-Implicit Euler integration for linear motion
        let linear_accel = total_force / mass;
        rb.linear_velocity += linear_accel * dt_sub;
        robot.root_transform.translation += rb.linear_velocity * dt_sub;

        // 5. Angular motion integration
        // Compute world frame inertia: I_world = R * I_local * R^T
        let rot_mat = Mat3::from_quat(robot.root_transform.rotation);
        let inv_i_local = Vec3::new(1.0 / inertia.x, 1.0 / inertia.y, 1.0 / inertia.z);
        let inv_i_world = rot_mat * Mat3::from_diagonal(inv_i_local) * rot_mat.transpose();

        let gyro_torque = rb.angular_velocity.cross(
            (rot_mat * Mat3::from_diagonal(inertia) * rot_mat.transpose()) * rb.angular_velocity,
        );
        let net_rot_torque = total_torque - gyro_torque;
        let angular_accel = inv_i_world * net_rot_torque;

        rb.angular_velocity += angular_accel * dt_sub;

        // Quaternion integration
        let w_quat = Quat::from_xyzw(
            rb.angular_velocity.x,
            rb.angular_velocity.y,
            rb.angular_velocity.z,
            0.0,
        );
        let q_curr = robot.root_transform.rotation;
        let q_dot = w_quat * q_curr * 0.5;
        let q_new = Quat::from_xyzw(
            q_curr.x + q_dot.x * dt_sub,
            q_curr.y + q_dot.y * dt_sub,
            q_curr.z + q_dot.z * dt_sub,
            q_curr.w + q_dot.w * dt_sub,
        );
        robot.root_transform.rotation = q_new.normalize();
    }

    // Keep kinematics synchronized after substeps
    robot.update_kinematics();
}

pub struct PhysicsPlugin;

impl Plugin for PhysicsPlugin {
    fn build(&self, app: &mut App) {
        app.init_resource::<PhysicsSettings>()
            .init_resource::<PhysicsRigidBody>()
            .add_systems(
                Update,
                physics_step_system.run_if(in_state(SimState::InGame)),
            );
    }
}
