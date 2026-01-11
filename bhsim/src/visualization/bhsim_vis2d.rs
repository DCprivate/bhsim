use bevy::prelude::*;
use bevy::sprite::{MaterialMesh2dBundle, Mesh2dHandle};
use bevy::math::primitives::Circle;

use crate::simulation::scenario::Scenario;
use crate::simulation::integrator::verlet_integrator;

#[derive(Component)]
struct BodyIndex(pub usize);

//#[derive(Component)]
//struct StatsText;

const SCALE: f32 = 50.0;
//const LABEL_OFFSET: f32 = 20.0; // For real time data later

pub fn run_2d(scenario: Scenario) {
    println!("run_2d: starting Bevy 2D viewer with {} bodies", scenario.system.bodies.len());

    App::new()
        .insert_resource(scenario)
        .add_plugins(DefaultPlugins)
        .add_systems(Startup, setup_bodies_system)
        .add_systems(Update, (physics_step_system, sync_transforms_system))
        .run();
}

fn setup_bodies_system(mut commands: Commands, scenario: Res<Scenario>, mut meshes: ResMut<Assets<Mesh>>, mut materials: ResMut<Assets<ColorMaterial>>) {
    // 2D camera
    commands.spawn(Camera2dBundle::default());

    for (i, body) in scenario.system.bodies.iter().enumerate() {
        let radius_screen = (body.radius as f32).max(0.02) * SCALE;
        let x = body.x.x as f32 * SCALE;
        let y = body.x.y as f32 * SCALE;

        commands.spawn((
            MaterialMesh2dBundle {
                mesh: Mesh2dHandle(meshes.add(Circle::new(radius_screen))),
                material: materials.add(ColorMaterial::from(Color::WHITE)),
                transform: Transform::from_xyz(x, y, 0.0),
                ..Default::default()
            },
            BodyIndex(i),
        ));
    }
}

fn physics_step_system(mut scenario: ResMut<Scenario>) {
    // Split &mut Scenario into &mut fields in one destructuring step
    let Scenario {
        system,
        parameters,
        forces,
        ..
    } = &mut *scenario;

    // Match signature: (system, forces, parameters)
    verlet_integrator(system, forces, parameters);
}

fn sync_transforms_system(scenario: Res<Scenario>, mut query: Query<(&BodyIndex, &mut Transform)>) {
    for (BodyIndex(i), mut transform) in &mut query {
        if let Some(b) = scenario.system.bodies.get(*i) {
            transform.translation.x = (b.x.x as f32) * SCALE;
            transform.translation.y = (b.x.y as f32) * SCALE;
        }
    }
}




