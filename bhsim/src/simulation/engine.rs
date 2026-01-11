//! High-level runtime engine settings
//!
//! Selects dimension (2D/3D), integrator, and Barnes–Hut options
//! used when building and running a `Scenario`

use crate::configuration::config::IntegratorConfig;

#[derive(Debug, Clone)]
pub struct Engine {
    pub dimension: bool, // false = 2D, true = 3D
    pub integrator: IntegratorConfig, // verlet or rk4
    pub barnes_hut: bool, // false = direct, true = barnes-hut
    pub theta: f64, // parameter to determine if use center of mass
}