use bevy::prelude::*;
use bevy_rapier3d::prelude::*;
use bevy_rapier3d::rapier::dynamics::{FrictionModel, SpringCoefficients};

use crate::SimState;

pub struct PhysicsPlugin;

impl Plugin for PhysicsPlugin {
    fn build(&self, app: &mut App) {
        app.add_plugins((
            RapierPhysicsPlugin::<NoUserData>::default(),
            RapierDebugRenderPlugin::default().disabled(),
        ))
        .insert_resource(TimestepMode::Interpolated {
            dt: 1.0 / 60.0,
            time_scale: 1.0,
            substeps: 12,
        })
        .add_systems(OnEnter(SimState::Config), pause_physics)
        .add_systems(OnEnter(SimState::InGame), resume_physics);
    }
}

fn pause_physics(mut config: Query<&mut RapierConfiguration, With<DefaultRapierContext>>) {
    for mut cfg in &mut config {
        cfg.physics_pipeline_active = false;
    }
}

fn resume_physics(mut config: Query<&mut RapierConfiguration, With<DefaultRapierContext>>) {
    for mut cfg in &mut config {
        cfg.physics_pipeline_active = true;
    }
}
