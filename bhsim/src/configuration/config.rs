//! Configuration types for loading simulation scenarios from YAML.
//!
//! This module defines a thin, `serde`-deserializable representation of a
//! simulation scenario. A scenario consists of:
//!
//! - [`EngineConfig`]     – global engine options (dimension, integrator, Barnes–Hut)
//! - [`ParametersConfig`] – numerical parameters and physical constants
//! - [`BodyConfig`]       – initial state for each body
//! - [`ScenarioConfig`]   – top-level wrapper used to load a scenario from YAML
//!
//! # YAML format
//! An example 2D scenario YAML matching these types:
//!
//! ```yaml
//! engine:
//!   dimension: false        # false -> 2D, true -> 3D
//!   integrator: "verlet"    # or "rk4"
//!   barnes_hut: true
//!   theta: 0.5
//!
//! parameters:
//!   t_end: 10.0             # total simulation time
//!   h0: 0.01                # initial/fixed step size
//!   atol: 1.0e-6            # absolute error tolerance
//!   rtol: 1.0e-6            # relative error tolerance
//!   merge_t: 1.0e-3         # merge distance threshold
//!   seed: 42.0              # deterministic seed
//!   eps2: 1.0e-4            # softening epsilon^2
//!   G: 1.0                  # gravitational constant
//!
//! bodies:
//!   - x: [ -0.5, 0.0 ]
//!     v: [  0.0, 1.0 ]
//!     m: 1.0
//!     radius: 0.02
//!   - x: [  0.5, 0.0 ]
//!     v: [  0.0, -1.0 ]
//!     m: 1.0
//!     radius: 0.02
//! ```
//!
//! The engine then maps this configuration into its internal runtime scenario
//! representation, which may use different structs optimized for performance.

use serde::Deserialize;

/// Which integrator method used by the engine
/// integrator: "verlet"` or `integrator: "rk4"
#[derive(Deserialize, Debug, Clone)]
pub enum IntegratorConfig {
    #[serde(rename = "verlet")] // Velocity/position Verlet integrator. Symplectic, long-term energy behavior, fixed step size
    Verlet,

    #[serde(rename = "rk4")] // Classical 4th-order Runge–Kutta integrator (RK4), higher local accuracy per step but not symplectic
    Rk4,
}

/// High-level engine configuration
/// Controls the structure of the simulation
#[derive(Deserialize, Debug)]
pub struct EngineConfig {
    pub dimension: bool, // `false` - 2D simulation, `true` - 3D simulation  
    pub integrator: IntegratorConfig, // Time integrator used for advancing the system state
    pub barnes_hut: bool, // `true` - gravitational forces are approximated using a hierarchical tree, `false` - use direct N^2 summation
    pub theta: Option<f64> // Determine if a node will be pruned and com is taken instead of going deeper in to tree
}

/// Global numerical and physical parameters for a scenario
#[derive(Deserialize, Debug, Clone)]
pub struct ParametersConfig {
    pub t_end: f64,   // time end
    pub h0: f64,      // time step size
    pub atol: f64,    // absolute error tolerance
    pub rtol: f64,    // relative error tolerance
    pub merge_t: f64, // merge threshold
    pub seed: f64,    // deterministic seed to make runs reproducable
    pub eps2: f64,    // softening - prevent singular forces at very small separations
    pub G: f64,       // gravitational constant
}

/// Configuration for a single body’s initial state
#[derive(Deserialize, Debug)]
pub struct BodyConfig {
    pub x: Vec<f64>, // Initial position vector `x` in simulation units
    pub v: Vec<f64>, // Initial velocity vector `v` in simulation units per time unit
    pub m: f64,      // Mass of the body
    pub radius: f64, // Radius of the body, used for collision/merge criteria and for visualization scaling
}

/// Top-level scenario configuration loaded from YAML.
#[derive(Deserialize, Debug)]
pub struct ScenarioConfig {
    pub engine: EngineConfig, // Engine-level configuration (dimension, integrator, Barnes–Hut)
    pub parameters: ParametersConfig, // Global numerical and physical parameters
    pub bodies: Vec<BodyConfig>, // List of bodies that define the initial state of the system
}