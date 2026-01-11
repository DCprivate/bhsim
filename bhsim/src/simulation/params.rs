//! Numerical and physical parameters for the simulation
//!
//! `Parameters` holds runtime settings:
//! - integration step size and end time,
//! - error tolerances for rk4,
//! - softening and gravitational constant (`eps2`, `G`),
//! - merge threshold and random seed

#[derive(Debug, Clone)]
pub struct Parameters {
    pub t_end: f64, // time end
    pub h0: f64, // step size
    pub atol: f64, // absolute error tolerance
    pub rtol: f64, // relative error tolerance
    pub merge_t: f64, // merge threshold
    pub seed: f64, // deterministic seed
    pub eps2: f64, // softening
    pub G: f64, // gravitational constant
}