pub mod simulation;
pub mod configuration;
pub mod visualization;
pub mod benchmark;

pub use simulation::states::{Body, System, NVec2, Body3, System3, NVec3};
pub use simulation::forces::{Acceleration, AccelSet, NewtonianGravity, NewtonianGravityBarnesHut3};
pub use simulation::integrator::verlet_integrator;
pub use simulation::scenario::{Scenario, Scenario3D};

pub use configuration::config::{IntegratorConfig, EngineConfig, ParametersConfig, BodyConfig, ScenarioConfig};

pub use visualization::{bhsim_vis2d::run_2d, bhsim_vis3d::run_3d};

pub use benchmark::benchmark::{bench_gravity, bench_verlet, bench_verlet_curve};