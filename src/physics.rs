use bevy::prelude::*;
use bevy_rapier3d::prelude::*;

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
        .add_systems(Startup, (pause_physics, configure_physics_integrator))
        .add_systems(OnEnter(SimState::Loading), pause_physics)
        .add_systems(OnEnter(SimState::Config), resume_physics)
        .add_systems(OnEnter(SimState::InGame), resume_physics);
    }
}

fn configure_physics_integrator(
    mut context_query: Query<&mut RapierContextSimulation, With<DefaultRapierContext>>,
) {
    for mut context in &mut context_query {
        let params = &mut context.integration_parameters;
        // Solve friction in position-bias correction passes (default is false)
        // Prevents contact penetration correction from inducing sideways slip
        params.friction_in_bias_pass = true;
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
