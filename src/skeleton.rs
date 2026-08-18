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
    pub transform: Mat4,
    pub children: Vec<usize>,               // idx in skeleton.joints of its children
}

impl Joint {
    pub fn new(name: impl Into<String>, transform: Mat4) -> Self {
        Self {
            name: name.into(),
            dofs: [Dof::default(); 3],
            transform,
            children: Vec::new(),
        }
    }

    pub fn with_dofs(mut self, rx: f32, ry: f32, rz: f32) -> Self {
        self.dofs = [Dof::new(rx), Dof::new(ry), Dof::new(rz)];
        self
    }
}

/// A rigid body segment (bone) represented as a simple 3D box.
#[derive(Debug, Clone)]
pub struct Bone {
    pub name: String,
    pub size: Vec3,
    pub transform: Mat4,
    pub parent: usize,
}

impl Bone {
    pub fn new(name: impl Into<String>, size: Vec3, transform: Mat4, parent: usize) -> Self {
        Self {
            name: name.into(),
            size,
            transform,
            parent,
        }
    }
}

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
        let joint = Joint::new(
            name,
            transform,
        );
        self.joints.push(joint);

        if let Some(parent_idx) = parent {
            self.joints[parent_idx].children.push(id);
        }
        id
    }

    /// Helper function to add a bone to the skeleton
    pub fn add_bone(&mut self, name: &str, size: Vec3, transform: Mat4, parent: usize) {
        let bone = Bone::new(
            name,
            size,
            transform,
            parent,
        );
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
                vec4(0.0, 0.5, 0.0, 1.0),
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
                vec4(0.0, 1.5, 0.0, 1.0),
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
        for joint in &self.joints {
            commands.spawn((
                Name::new(joint.name.clone()),
                Mesh3d(sphere_mesh.clone()),
                MeshMaterial3d(joint_material.clone()),
                Transform::from_matrix(joint.transform),
            ));
        }
        // spawn boxes for joints
        for bone in &self.bones {
            commands.spawn((
                Name::new(bone.name.clone()),
                Mesh3d(meshes.add(Cuboid::from_size(bone.size))),
                MeshMaterial3d(bone_material.clone()),
                Transform::from_matrix(bone.transform),
            ));
        }
    }
}

/// Update skeleton transform
pub fn update_skeleton_transform(skeleton: Option<Res<Skeleton>>) {
    todo!()
}

pub fn redraw_skeleton(skeleton: Option<Res<Skeleton>>) {
    todo!()
}    

/// Bevy system to draw coordinate axes for the skeleton resource every frame.
pub fn draw_skeleton_axes(skeleton: Option<Res<Skeleton>>, mut gizmos: Gizmos) {
    if let Some(skeleton) = skeleton {
        let length = 0.4;
        for joint in &skeleton.joints {
            let origin = joint.transform.transform_point3(Vec3::ZERO);
            let x_dir = joint.transform.transform_vector3(Vec3::X).normalize_or_zero() * length;
            let y_dir = joint.transform.transform_vector3(Vec3::Y).normalize_or_zero() * length;
            let z_dir = joint.transform.transform_vector3(Vec3::Z).normalize_or_zero() * length;

            // X axis -> Red
            gizmos.arrow(origin, origin + x_dir, Color::srgb(1.0, 0.0, 0.0));
            // Y axis -> Green
            gizmos.arrow(origin, origin + y_dir, Color::srgb(0.0, 1.0, 0.0));
            // Z axis -> Blue
            gizmos.arrow(origin, origin + z_dir, Color::srgb(0.0, 0.0, 1.0));
        }
    }
}