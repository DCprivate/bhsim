//! Fixed-step time integrators for the N-body system
//!
//! Provides 2D and 3D double-verlet and a single–verlet
//! leapfrog integrator, all driven by `AccelSet`/`AccelSet3` and `Parameters`

use super::states::{System, NVec2, System3, NVec3};
use super::forces::{AccelSet, AccelSet3};
use super::params::Parameters;

/// Advance the 2D system by one step using velocity–Verlet
/// Uses two force evaluations per step and updates positions, velocities,
/// and `sys.t` in-place based on `params.h0`
pub fn verlet_integrator(sys: &mut System, forces: &mut AccelSet, params: &mut Parameters) {
    let n = sys.bodies.len();
    if n == 0 { // no bodies, return
        return;
    }

    let dt = params.h0; // time step dt
    let half_dt = 0.5 * dt; // half step dt/2, half update for verlet

    // Allocate a vector of accelerations, one per body, initialized to zero
    // a_old[i] will hold a_n for body i at the current time t = sys.t
    let mut a_old = vec![NVec2::zeros(); n];

    // Ask the force set to accumulate accelerations at time t_n into a_old,
    // based on the current system state sys
    forces.accumulate_accels(sys.t, &*sys, &mut a_old);

    // For each body and its corresponding acceleration a^n:
    // v_n+1/2 = v_n + (1/2 * dt) * a_n
    for (b, a) in sys.bodies.iter_mut().zip(a_old.iter()) {
        b.v += half_dt * *a;
    }

    // Now that velocities are at the half-step, advance positions by a full step.
    // x_n+1 = x_n + dt v_n+1/2
    for b in sys.bodies.iter_mut() {
        b.x += dt * b.v;
    }

    // Increment the system time by one full step
    sys.t += dt;

    // Allocate another acceleration buffer for the new time level
    // a_new[i] will hold a_n+1 for body i at the updated time t = sys.t
    let mut a_new = vec![NVec2::zeros(); n];

    // Recompute accelerations at the new time and positions x_n+1
    forces.accumulate_accels(sys.t, &*sys, &mut a_new);

    // // Finish the velocity update: v_n+1 = v_n+1/2 + 0.5 dt a_n+1
    for (b, a) in sys.bodies.iter_mut().zip(a_new.iter()) {
        b.v += half_dt * *a;
    }
}



// =========================================================================================
// 3d stuff below
// =========================================================================================

/// Advance the 3D system by one time step using velocity–Verlet.
/// Uses two force evaluations per step and updates positions, velocities,
/// and `sys.t` in-place with fixed step `dt = params.h0`.
pub fn verlet_integrator_3d(sys: &mut System3, forces: &mut AccelSet3, params: &mut Parameters) {
    let n = sys.bodies.len();
    if n == 0 { // no bodies, return
        return;
    }
    let dt = params.h0; // time step dt
    let half_dt = 0.5 * dt; // half step dt/2

    // a_n from x_n at time t_n
    let mut a_old = vec![NVec3::zeros(); n];
    forces.accumulate_accels(sys.t, &*sys, &mut a_old);

    // Kick: v_n+1/2 = v_n + (1/2 * dt) * a_n
    for (b, a) in sys.bodies.iter_mut().zip(a_old.iter()) {
        b.v += half_dt * *a;
    }

    // Drift: full-step position: x_n+1 = x_n + dt v_n+1/2
    for b in sys.bodies.iter_mut() {
        b.x += dt * b.v;
    }

    // advance time: t_n+1 = t_n + dt
    sys.t += dt;

    // a_n+1 from x_n+1 at time t_n+1
    let mut a_new = vec![NVec3::zeros(); n];
    forces.accumulate_accels(sys.t, &*sys, &mut a_new);

    // Second kick: v_{n+1} = v_half + (dt/2) * a_{n+1}
    for (b, a) in sys.bodies.iter_mut().zip(a_new.iter()) {
        b.v += half_dt * *a;
    }
}

/// Advance the 3D system by one step using a single-force-eval leapfrog.
/// Uses one force evaluation per step and updates positions, velocities,
/// and `sys.t` in-place with fixed step `dt = params.h0`.
pub fn verlet_single_3d(sys: &mut System3, forces: &mut AccelSet3, params: &mut Parameters) {
    let n = sys.bodies.len();
    if n == 0 { // No bodies, return
        return;
    }
    let dt = params.h0; // time step dt
    let half_dt = 0.5 * dt; // half step dt/2

    // Drift: x_n+1 = x_n + (dt/2) * v_n
    for b in sys.bodies.iter_mut() {
        b.x += half_dt * b.v;
    }

    // advance half time (t_n + dt/2)
    let t_mid = sys.t + half_dt;

    
    // compute a_mid from x_half
    let mut a_mid = vec![NVec3::zeros(); n];

    // Kick: v_n+1 = v_n + dt * a_mid
    forces.accumulate_accels(t_mid, &*sys, &mut a_mid);
    for (b, a) in sys.bodies.iter_mut().zip(a_mid.iter()) {
        b.v += dt * *a;
    }

    // Second drift: x_n+1 = x_half + dt/2 * v_n+1
    for b in sys.bodies.iter_mut() {
        b.x += half_dt * b.v;
    }

    // finish advancing time: t_n+1 = t_n + dt
    sys.t += dt;
}

// =========================================================================================
// rk4 integrator will be implemented here
// =========================================================================================