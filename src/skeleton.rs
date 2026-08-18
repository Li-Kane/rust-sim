#![allow(dead_code)]
use bevy::prelude::*;

/// A single degree of freedom (1D parameter, e.g. rotation angle in radians).
#[derive(Debug, Clone, Copy, Default)]
pub struct Dof {
    pub value: f32,
}

impl Dof {
    pub fn new(value: f32) -> Self {
        Self { value }
    }
}

/// A 3-DOF Joint for a bone
/// Contains 3 rotational degrees of freedom (X, Y, Z / Roll, Pitch, Yaw).
#[derive(Debug, Clone)]
pub struct Joint {
    pub name: String,
    pub dofs: [Dof; 3],
    pub world_transform: Mat4,
    pub local_transform: Mat4,
    pub parent: Option<usize>,
    pub children: Vec<usize>, // idx in skeleton.joints of its children
}

impl Joint {
    pub fn new(name: impl Into<String>, transform: Mat4, parent: Option<usize>) -> Self {
        Self {
            name: name.into(),
            dofs: [Dof::default(); 3],
            world_transform: transform.clone(),
            local_transform: transform,
            parent,
            children: Vec::new(),
        }
    }

    pub fn with_dofs(mut self, rx: f32, ry: f32, rz: f32) -> Self {
        self.dofs = [Dof::new(rx), Dof::new(ry), Dof::new(rz)];
        self
    }

    /// Computes the Rodrigues rotation matrix assuming DOF values are in degrees.
    /// R = I + sin(θ) K + (1 - cos(θ)) K²
    /// where ω = (dof_x, dof_y, dof_z), θ = ||ω|| (in radians), and K is the skew-symmetric matrix of ω / θ.
    pub fn rodrigues_rotation(&self) -> Mat4 {
        let omega = Vec3::new(
            self.dofs[0].value.to_radians(),
            self.dofs[1].value.to_radians(),
            self.dofs[2].value.to_radians(),
        );
        let theta = omega.length();
        if theta < 1e-6 {
            return Mat4::IDENTITY;
        }
        let k = omega / theta;
        let k_cross = Mat3::from_cols(
            Vec3::new(0.0, k.z, -k.y),
            Vec3::new(-k.z, 0.0, k.x),
            Vec3::new(k.y, -k.x, 0.0),
        );
        Mat4::from_mat3(
            Mat3::IDENTITY + k_cross * theta.sin() + (k_cross * k_cross) * (1.0 - theta.cos()),
        )
    }
}

/// A rigid body segment (bone) represented as a simple 3D box.
#[derive(Debug, Clone)]
pub struct Bone {
    pub name: String,
    pub size: Vec3,
    pub world_transform: Mat4,
    pub local_transform: Mat4,
    pub parent: usize,
}

impl Bone {
    pub fn new(name: impl Into<String>, size: Vec3, transform: Mat4, parent: usize) -> Self {
        Self {
            name: name.into(),
            size,
            world_transform: transform.clone(),
            local_transform: transform,
            parent,
        }
    }
}

/// Marker component storing the index into `skeleton.joints`
#[derive(Component, Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub struct JointIndex(pub usize);

/// Marker component storing the index into `skeleton.bones`
#[derive(Component, Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub struct BoneIndex(pub usize);

/// Complete skeleton tree structure.
#[derive(Debug, Clone, Resource)]
pub struct Skeleton {
    pub name: String,
    pub joints: Vec<Joint>,
    pub bones: Vec<Bone>,
}

impl Skeleton {
    pub fn new(name: impl Into<String>) -> Self {
        Self {
            name: name.into(),
            joints: Vec::new(),
            bones: Vec::new(),
        }
    }

    /// Helper function to add a joint to the skeleton
    pub fn add_joint(&mut self, name: &str, transform: Mat4, parent: Option<usize>) -> usize {
        let id = self.joints.len();
        let joint = Joint::new(name, transform, parent);
        self.joints.push(joint);

        if let Some(parent_idx) = parent {
            self.joints[parent_idx].children.push(id);
        }
        id
    }

    /// Helper function to add a bone to the skeleton
    pub fn add_bone(&mut self, name: &str, size: Vec3, transform: Mat4, parent: usize) {
        let bone = Bone::new(name, size, transform, parent);
        self.bones.push(bone);
    }

    /// Builds a 2-link skeleton with 2 joints:
    /// - Joint 1 at the world origin (0, 0, 0)
    /// - Link 1 (bone 1)
    /// - Joint 2 between both links at (0, 1, 0)
    /// - Link 2 (bone 2)
    pub fn build_two_link() -> Self {
        let mut two_link = Self::new("two_link");
        // Joint 1 at the world origin (0, 0, 0)
        let joint_1 = two_link.add_joint("joint 1", Mat4::IDENTITY, None);

        // Link 1 (bone 1)
        two_link.add_bone(
            "bone1",
            Vec3::new(0.15, 1.0, 0.15),
            Mat4::from_cols(
                vec4(1.0, 0.0, 0.0, 0.0),
                vec4(0.0, 1.0, 0.0, 0.0),
                vec4(0.0, 0.0, 1.0, 0.0),
                vec4(0.0, 0.0, 0.0, 1.0),
            ),
            joint_1,
        );

        // Joint 2 between both links at (0, 1, 0)
        let joint_2 = two_link.add_joint(
            "joint 2",
            Mat4::from_cols(
                vec4(1.0, 0.0, 0.0, 0.0),
                vec4(0.0, 1.0, 0.0, 0.0),
                vec4(0.0, 0.0, 1.0, 0.0),
                vec4(0.0, 1.0, 0.0, 1.0),
            ),
            Some(joint_1),
        );

        // Link 2 (bone 2)
        two_link.add_bone(
            "bone 2",
            Vec3::new(0.1, 1.0, 0.1),
            Mat4::from_cols(
                vec4(1.0, 0.0, 0.0, 0.0),
                vec4(0.0, 1.0, 0.0, 0.0),
                vec4(0.0, 0.0, 1.0, 0.0),
                vec4(0.0, 0.5, 0.0, 1.0),
            ),
            joint_2,
        );

        two_link
    }

    /// Spawns the skeleton into the Bevy world by iterating through bones and joints
    /// Spawns each bone as a box mesh and each joint as a sphere mesh at its pivot position.
    pub fn spawn(
        &mut self,
        commands: &mut Commands,
        meshes: &mut ResMut<Assets<Mesh>>,
        bone_material: Handle<StandardMaterial>,
        joint_material: Handle<StandardMaterial>,
    ) {
        let sphere_mesh = meshes.add(Sphere::new(0.15));
        // spawn spheres for joints
        for (i, joint) in self.joints.iter().enumerate() {
            commands.spawn((
                JointIndex(i),
                Name::new(joint.name.clone()),
                Mesh3d(sphere_mesh.clone()),
                MeshMaterial3d(joint_material.clone()),
                Transform::from_matrix(joint.world_transform),
            ));
        }
        // spawn boxes for bones
        for (i, bone) in self.bones.iter().enumerate() {
            commands.spawn((
                BoneIndex(i),
                Name::new(bone.name.clone()),
                Mesh3d(meshes.add(Cuboid::from_size(bone.size))),
                MeshMaterial3d(bone_material.clone()),
                Transform::from_matrix(bone.world_transform),
            ));
        }
    }
}

/// Update joint and link transforms
pub fn update_skeleton_transform(mut skeleton: ResMut<Skeleton>) {
    let skeleton = &mut *skeleton;
    for i in 0..skeleton.joints.len() {
        let joint = &skeleton.joints[i];
        let mut world_transform = joint.local_transform * joint.rodrigues_rotation();
        if let Some(parent) = joint.parent {
            world_transform = &skeleton.joints[parent].world_transform * world_transform;
        }
        let joint = &mut skeleton.joints[i];
        joint.world_transform = world_transform;
    }

    for bone in &mut skeleton.bones {
        bone.world_transform = &skeleton.joints[bone.parent].world_transform * bone.local_transform;
    }
}

/// Redraws the skeleton entities in the scene by syncing their Transforms via O(1) index lookups
pub fn redraw_skeleton(
    skeleton: Option<Res<Skeleton>>,
    mut joints_query: Query<(&JointIndex, &mut Transform)>,
    mut bones_query: Query<(&BoneIndex, &mut Transform), Without<JointIndex>>,
) {
    let Some(skeleton) = skeleton else { return };

    for (joint_idx, mut transform) in &mut joints_query {
        *transform = Transform::from_matrix(skeleton.joints[joint_idx.0].world_transform);
    }

    for (bone_idx, mut transform) in &mut bones_query {
        *transform = Transform::from_matrix(skeleton.bones[bone_idx.0].world_transform);
    }
}

/// Bevy system to draw coordinate axes for the skeleton resource every frame.
pub fn draw_skeleton_axes(skeleton: Option<Res<Skeleton>>, mut gizmos: Gizmos) {
    if let Some(skeleton) = skeleton {
        let length = 0.4;
        for joint in &skeleton.joints {
            let origin = joint.world_transform.transform_point3(Vec3::ZERO);
            let x_dir = joint
                .world_transform
                .transform_vector3(Vec3::X)
                .normalize_or_zero()
                * length;
            let y_dir = joint
                .world_transform
                .transform_vector3(Vec3::Y)
                .normalize_or_zero()
                * length;
            let z_dir = joint
                .world_transform
                .transform_vector3(Vec3::Z)
                .normalize_or_zero()
                * length;

            // X axis -> Red
            gizmos.arrow(origin, origin + x_dir, Color::srgb(1.0, 0.0, 0.0));
            // Y axis -> Green
            gizmos.arrow(origin, origin + y_dir, Color::srgb(0.0, 1.0, 0.0));
            // Z axis -> Blue
            gizmos.arrow(origin, origin + z_dir, Color::srgb(0.0, 0.0, 1.0));
        }
    }
}
