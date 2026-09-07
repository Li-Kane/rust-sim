# Run
'''
# webapp
trunk serve
# native app window
cargo run
'''

/// Default standing joint positions for Spot (in radians)
pub const DEFAULT_JOINT_ANGLES: [(&str, f32); 12] = [
    ("fl_hx", 0.1),
    ("fl_hy", 0.9),
    ("fl_kn", -1.5),
    ("fr_hx", -0.1),
    ("fr_hy", 0.9),
    ("fr_kn", -1.5),
    ("hl_hx", 0.1),
    ("hl_hy", 1.1),
    ("hl_kn", -1.5),
    ("hr_hx", -0.1),
    ("hr_hy", 1.1),
    ("hr_kn", -1.5),
];

/// Returns the coordinate conversion rotation from URDF (Z-up, X-fwd, Y-left) to Bevy (Y-up, -Z-fwd, +X-right)
pub fn urdf_to_bevy_orientation() -> Quat {
    Quat::from_mat3(&Mat3::from_cols(
        Vec3::new(0.0, 0.0, -1.0),
        Vec3::new(-1.0, 0.0, 0.0),
        Vec3::new(0.0, 1.0, 0.0),
    ))
}
