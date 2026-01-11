//! Build fully-initialized simulation scenarios from configuration
//!
//! Takes a `ScenarioConfig` (YAML-facing) and produces runtime bundles
//! (`Scenario` for 2D, `Scenario3D` for 3D) containing:
//! - engine settings (`Engine`)
//! - numerical parameters (`Parameters`)
//! - system state (`System`/`System3` with bodies at t = 0)
//! - active force set (`AccelSet`/`AccelSet3`)
//!
//! These scenarios are inserted into Bevy as `Resource`s and consumed by
//! integration and visualization systems

use bevy::prelude::Resource;

use crate::configuration::config::{ScenarioConfig, BodyConfig};
use crate::simulation::engine::Engine;
use crate::simulation::params::Parameters;
use crate::simulation::states::{System, Body, NVec2, System3, Body3, NVec3};
use crate::simulation::forces::{AccelSet, NewtonianGravity, AccelSet3, NewtonianGravity3, NewtonianGravityBarnesHut3};

/// Bevy resource representing a fully-initialized simulation 2D scenario
///
/// This is the main “runtime bundle” constructed from a [`ScenarioConfig`]:
/// it contains the engine settings, parameters, current system
/// state, and the set of active force laws (accelerations)
///
/// In Bevy terms, this is inserted as a `Resource` and then read by systems
/// responsible for integration, visualization, diagnostics, etc
#[derive(Resource)]
pub struct Scenario {
    pub engine: Engine,
    pub parameters: Parameters,
    pub system: System,
    pub forces: AccelSet,
}

impl Scenario {
    pub fn build_scenario(cfg: ScenarioConfig) -> Self {
        // Bodies: map `BodyConfig` -> runtime `Body` using nalgebra vectors
        let bodies: Vec<Body> = cfg.bodies.iter().map(|bc: &BodyConfig| Body {
            x: NVec2::new(bc.x[0], bc.x[1]),
            v: NVec2::new(bc.v[0], bc.v[1]),
            m: bc.m,
            radius: bc.radius,
        }).collect();

        // Initial system state: bodies at t = 0
        let system = System { 
            bodies, 
            t: 0.0 
        };

        // Parameters (runtime) from ParametersConfig
        let p_cfg = cfg.parameters;
        let parameters = Parameters {
            t_end: p_cfg.t_end,
            h0: p_cfg.h0,
            atol: p_cfg.atol,
            rtol: p_cfg.rtol,
            merge_t: p_cfg.merge_t,
            seed: p_cfg.seed,
            eps2: p_cfg.eps2,
            G: p_cfg.G,
        };

        // Engine (runtime) from EngineConfig
        let e_cfg = cfg.engine;
        let engine = Engine {
            dimension: e_cfg.dimension,
            integrator: e_cfg.integrator,
            barnes_hut: e_cfg.barnes_hut,
            theta: e_cfg.theta.unwrap_or(0.7),
        };

        // Forces: construct an AccelSet and register Newtonian gravity
        let mut forces = AccelSet::new();
        forces = forces.with(NewtonianGravity {
            G: parameters.G,
            eps2: parameters.eps2,
        });

        Self {
            engine,
            parameters,
            system,
            forces,
        }
    }
}

// =========================================================================================
// 3d stuff below
// =========================================================================================

/// Bevy resource representing a fully-initialized 3D simulation scenario
///
/// This is the main "runtime bundle" constructed from a [`ScenarioConfig`]:
/// it contains the engine settings, parameters, current system
/// state, and the set of active force laws (accelerations)
///
/// In Bevy terms, this is inserted as a `Resource` and then read by systems
/// responsible for integration, visualization, diagnostics, etc
#[derive(Resource)]
pub struct Scenario3D {
    pub engine: Engine,
    pub parameters: Parameters,
    pub system: System3,
    pub forces: AccelSet3,
}

impl Scenario3D {
    pub fn build_scenario_3d(cfg: ScenarioConfig) -> Self {
        // Bodies: map `BodyConfig` -> runtime `Body` using nalgebra vectors
        let bodies: Vec<Body3> = cfg.bodies.iter().map(|bc: &BodyConfig| Body3 {
            x: NVec3::new(bc.x[0], bc.x[1], bc.x[2]),
            v: NVec3::new(bc.v[0], bc.v[1], bc.v[2]),
            m: bc.m,
            radius: bc.radius,
        }).collect();

        // Initial system state: bodies at t = 0
        let system = System3 {
            bodies,
            t: 0.0,
        };

        // Parameters (runtime) from ParametersConfig
        let parameters = Parameters {
            t_end: cfg.parameters.t_end,
            h0: cfg.parameters.h0,
            atol: cfg.parameters.atol,
            rtol: cfg.parameters.rtol,
            merge_t: cfg.parameters.merge_t,
            seed: cfg.parameters.seed,
            eps2: cfg.parameters.eps2,
            G: cfg.parameters.G,
        };

        // Engine (runtime) from EngineConfig
        let engine = Engine {
            dimension: cfg.engine.dimension,
            integrator: cfg.engine.integrator,
            barnes_hut: cfg.engine.barnes_hut,
            theta: cfg.engine.theta.unwrap_or(0.7),
        };

        // Forces: construct an AccelSet and register Newtonian gravity
        let mut forces = AccelSet3::new();
        /*forces = forces.with(NewtonianGravity3 {
            G: parameters.G,
            eps2: parameters.eps2,
        });*/
        if cfg.engine.barnes_hut && cfg.engine.dimension {
            forces = forces.with(NewtonianGravityBarnesHut3 {
                G: parameters.G,
                eps2: parameters.eps2,
                theta: cfg.engine.theta.unwrap_or(0.7),
            });
        } else {
            forces = forces.with(NewtonianGravity3 {
                G: parameters.G,
                eps2: parameters.eps2,
            });
        }

        Self {
            engine,
            parameters,
            system,
            forces,
        }
    }
}