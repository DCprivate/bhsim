use bevy::prelude::*;
use bevy::math::primitives::{Sphere, Cuboid};
use bevy::ecs::system::Local;

use crate::simulation::scenario::Scenario3D;
use crate::simulation::integrator::{verlet_integrator_3d, verlet_single_3d};
use crate::simulation::states::Body3;

//use std::time::Instant; // for debugging

/// Component tagging each sphere with its body index into Scenario3D.system.bodies
#[derive(Component)]
struct BodyIndex3(pub usize);

/// World-space → screen-space scaling factor for positions and radii
const SCALE3D: f32 = 50.0;

/// Distance of the camera from the origin along +Z
const CAMERA_DISTANCE: f32 = 1500.0;

/// Convenience entrypoint, mirroring your run_2d(scenario: Scenario)
pub fn run_3d(scenario: Scenario3D) {
    println!("run_3d: starting Bevy 3D viewer with {} bodies", scenario.system.bodies.len());

    App::new()
        .insert_resource(scenario)
        .add_plugins(DefaultPlugins)
        .add_systems(Startup, setup_3d)
        .add_systems(Update, (physics_step_3d, sync_transforms_3d))
        .run();
}

/// Startup system: spawn camera, light, and one sphere per body
fn setup_3d(mut commands: Commands,
    mut meshes: ResMut<Assets<Mesh>>,
    mut materials: ResMut<Assets<StandardMaterial>>,
    scenario: Res<Scenario3D>,
) {
    // Simple 3D camera looking at the origin
    commands.spawn(Camera3dBundle {

        camera: Camera {
            clear_color: ClearColorConfig::Custom(Color::srgb(0.0, 0.0, 0.0)), // pure black
            ..Default::default()
        },
        transform: Transform::from_xyz(200.0, 150.0, CAMERA_DISTANCE)
            .looking_at(Vec3::ZERO, Vec3::Y),
        ..Default::default()
    });

    // Basic point light
    commands.spawn(PointLightBundle {
        point_light: PointLight {
            intensity: 1500.0,
            range: 1000.0,
            ..Default::default()
        },
        transform: Transform::from_xyz(100.0, 100.0, CAMERA_DISTANCE),
        ..Default::default()
    });

    // =====================================================================
    // 3D AXES: three thin boxes along X, Y, Z
    spawn_axes(&mut commands, &mut meshes, &mut materials);
    // =====================================================================

    // Spawn one sphere per body
    for (i, b) in scenario.system.bodies.iter().enumerate() {
        // Ensure a minimum visual radius so tiny bodies are still visible
        let radius_screen = (b.radius as f32).max(0.02) * SCALE3D;

        commands.spawn((
            PbrBundle {
                mesh: meshes.add(Sphere::new(radius_screen).mesh()),
                material: materials.add(StandardMaterial {
                    base_color: Color::srgb(1.0, 1.0, 1.0), // white
                    unlit: true,
                    ..Default::default()
                }),
                transform: Transform::from_xyz(
                    (b.x.x as f32) * SCALE3D,
                    (b.x.y as f32) * SCALE3D,
                    (b.x.z as f32) * SCALE3D,
                ),
                ..Default::default()
            },
            BodyIndex3(i),
        ));
    }
}

/// Per-frame physics integration for the 3D scenario
fn physics_step_3d(mut scenario: ResMut<Scenario3D>) {
    let Scenario3D {
        system,
        parameters,
        forces,
        ..
    } = &mut *scenario;

    // Single verlet
    //verlet_single_3d(system, forces, parameters);

    // Integrate one timestep using your 3D Verlet
    //verlet_integrator_3d(system, forces, parameters);

    //let t0 = Instant::now();
    verlet_integrator_3d(system, forces, parameters);
    //let dt = t0.elapsed().as_secs_f64() * 1000.0;
    //println!("physics_step_3d: ~{dt:.3} ms");
}

// ========================================================================================
// Body and velocity color stuff
// ========================================================================================

// Old, non-colored
/*fn sync_transforms_3d(
    scenario: Res<Scenario3D>,
    mut query: Query<(&BodyIndex3, &mut Transform)>,
) {
    for (BodyIndex3(i), mut transform) in &mut query {
        if let Some(b) = scenario.system.bodies.get(*i) {
            transform.translation = Vec3::new(
                (b.x.x as f32) * SCALE3D,
                (b.x.y as f32) * SCALE3D,
                (b.x.z as f32) * SCALE3D,
            );
        }
    }
}*/

// ========================================================================================
// compute_heavy_mass_threadshold(), color_for_body(), speed_to_color() are AI generated

// Tune these:
const HEAVY_MASS_FRACTION: f64 = 1.0; // fraction of max mass
const HEAVY_MASS_FLOOR: f64 = 0.0;    // absolute floor in your mass units (optional)

fn compute_heavy_mass_threshold(bodies: &[Body3]) -> f64 {
    let mut max_mass = 0.0_f64;
    for b in bodies {
        if b.m > max_mass {
            max_mass = b.m;
        }
    }

    if max_mass <= 0.0 {
        return f64::INFINITY; // no heavy bodies if masses are degenerate
    }

    let thresh = HEAVY_MASS_FRACTION * max_mass;
    thresh.max(HEAVY_MASS_FLOOR)
}

fn color_for_body(b: &Body3, speed: f32, v_norm: f32, heavy_mass_threshold: f64) -> Color {
    if b.m >= heavy_mass_threshold {
        // "Heavy" body -> keep white
        Color::srgb(1.0, 1.0, 1.0)
    } else {
        // Regular body -> velocity-based color
        speed_to_color(speed, v_norm)
    }
}

fn speed_to_color(speed: f32, max_speed: f32) -> Color {
    if max_speed <= 0.0 {
        // all stationary -> pick a default
        return Color::srgb(1.0, 1.0, 1.0);
    }

    let raw = speed / max_speed;
    let gamma = 1.0;          // >1.0 -> slower gradient; tweak this
    let t = raw.powf(gamma).clamp(0.0, 1.0);

    // Simple blue -> red gradient
    let r = t;
    let g = 0.0;
    let b = 1.0 - t;

    Color::srgb(r, g, b)
}
// ========================================================================================

#[derive(Default)]
struct VelocityColorState {
    smoothed_max: f32,
}

fn sync_transforms_3d(
    scenario: Res<Scenario3D>,
    mut materials: ResMut<Assets<StandardMaterial>>,
    mut query: Query<(&BodyIndex3, &mut Transform, &Handle<StandardMaterial>)>,
    mut vel_state: Local<VelocityColorState>,
) {
    let bodies = &scenario.system.bodies;

    // ========================================================================================
    // The body speed calc and coloring stuff inside the "="'s here are AI generated as well

    // Compute raw max speed for this frame
    let mut max_speed_frame: f32 = 0.0;
    for b in bodies {
        let s = b.v.norm() as f32; // assuming NVec3::norm()
        if s > max_speed_frame {
            max_speed_frame = s;
        }
    }

    // Initialize smoothed_max on first run
    if vel_state.smoothed_max == 0.0 {
        vel_state.smoothed_max = max_speed_frame.max(1e-6);
    }

    // Cap outlier influence: don't let this frame's max
    // be more than X times the current smoothed value.
    let v_cap_factor = 3.0; // tweak: 1.5–3.0 are reasonable
    let capped_max = if max_speed_frame > 0.0 {
        max_speed_frame.min(vel_state.smoothed_max * v_cap_factor)
    } else {
        vel_state.smoothed_max
    };

    // Smooth over time (EMA)
    let alpha = 0.01; // smaller = slower changes, less pulsing
    vel_state.smoothed_max =
        (1.0 - alpha) * vel_state.smoothed_max + alpha * capped_max;

    let v_norm = vel_state.smoothed_max.max(1e-6);

    // Compute heavy mass threshold once per frame
    let heavy_mass_threshold = compute_heavy_mass_threshold(bodies);
    // ========================================================================================

    // Update transforms + colors
    for (BodyIndex3(i), mut transform, mat_handle) in &mut query {
        if let Some(b) = bodies.get(*i) {
            // position
            transform.translation = Vec3::new(
                (b.x.x as f32) * SCALE3D,
                (b.x.y as f32) * SCALE3D,
                (b.x.z as f32) * SCALE3D,
            );

            // speed and final color
            let speed = b.v.norm() as f32;
            let color = color_for_body(b, speed, v_norm, heavy_mass_threshold);

            if let Some(mat) = materials.get_mut(mat_handle) {
                mat.base_color = color;
            }
        }
    }
}

// =========================================================================================
// Draw 3D axes for visual reference
// =========================================================================================

fn spawn_axes(commands: &mut Commands, meshes: &mut Assets<Mesh>, materials: &mut Assets<StandardMaterial>) {
    // Axis length and thickness, in *world* units
    let axis_len = 10.0 * SCALE3D;
    let axis_thickness = 0.009 * SCALE3D;

    // X axis: red, along +X/-X
    commands.spawn(PbrBundle {
        mesh: meshes.add(Cuboid::new(axis_len, axis_thickness, axis_thickness).mesh()),
        material: materials.add(StandardMaterial {
            base_color: Color::srgb(1.0, 0.0, 0.0), // red
            unlit: true,
            ..Default::default()
        }),
        // Cuboid is centered at its transform origin, so this puts it crossing the world origin
        transform: Transform::from_xyz(0.0, 0.0, 0.0),
        ..Default::default()
    });

    // Y axis: green, along +Y/-Y
    commands.spawn(PbrBundle {
        mesh: meshes.add(Cuboid::new(axis_thickness, axis_len, axis_thickness).mesh()),
        material: materials.add(StandardMaterial {
            base_color: Color::srgb(0.0, 1.0, 0.0), // green
            unlit: true,
            ..Default::default()
        }),
        transform: Transform::from_xyz(0.0, 0.0, 0.0),
        ..Default::default()
    });

    // Z axis: blue, along +Z/-Z
    commands.spawn(PbrBundle {
        mesh: meshes.add(Cuboid::new(axis_thickness, axis_thickness, axis_len).mesh()),
        material: materials.add(StandardMaterial {
            base_color: Color::srgb(0.0, 0.0, 1.0), // blue
            unlit: true,
            ..Default::default()
        }),
        transform: Transform::from_xyz(0.0, 0.0, 0.0),
        ..Default::default()
    });
}