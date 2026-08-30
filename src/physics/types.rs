use bevy::prelude::*;

/// Body collision box dimensions in URDF frame (Length: 0.58m, Width: 0.26m, Height: 0.16m)
pub const BODY_BOX_SIZE: Vec3 = Vec3::new(0.58, 0.26, 0.16);

/// Physics configuration parameters matching kdFlex / personal_website
#[derive(Resource, Debug, Clone)]
pub struct PhysicsSettings {
    pub gravity: Vec3,
    pub kp: f32,              // Normal spring stiffness (N/m^1.5)
    pub kc: f32,              // Normal damping (N*s/m^2.5)
    pub exponent: f32,        // Hunt-Crossley non-linear power n (1.5)
    pub mu: f32,              // Coulomb friction coefficient
    pub substeps: usize,      // Numerical integration sub-steps per frame
    pub ground_y: f32,        // Ground floor plane Y level
    pub foot_radius: f32,     // Foot collision sphere radius (m)
    pub foot_offset: Vec3,    // Offset in URDF lower leg local frame
    pub joint_kp: f32,        // Joint actuator PD stiffness (N*m/rad)
    pub joint_kd: f32,        // Joint actuator PD damping (N*m*s/rad)
    pub show_colliders: bool, // Debug visualization toggle
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
            joint_kp: 80.0,
            joint_kd: 1.5,
            show_colliders: false,
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

/// Contact point metadata with associated link index
#[derive(Debug, Clone, Copy)]
pub struct ContactInfo {
    pub world_pos: Vec3,
    pub radius: f32,
    pub link_idx: usize,
}
