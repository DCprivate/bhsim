//! Core state types for the N-body simulation.
//!
//! Defines 2D and 3D body/system structs:
//! - `Body` / `System`  using `NVec2` (2d)
//! - `Body3` / `System3` using `NVec3` (3d)
//!
//! Each system holds the list of bodies and the current simulation time `t`.

use nalgebra::{Vector2, Vector3};
pub type NVec2 = Vector2<f64>;
pub type NVec3 = Vector3<f64>;

#[derive(Debug, Clone)]
pub struct Body {
    pub x: NVec2, // position
    pub v: NVec2, // velocity
    pub m: f64, // mass
    pub radius: f64, // radius (softening)
}

#[derive(Debug, Clone)]
pub struct System {
    pub bodies: Vec<Body>, // 2d collection of bodies
    pub t: f64, // time
}

#[derive(Debug, Clone)]
pub struct Body3 {
    pub x: NVec3, // 3d position
    pub v: NVec3, // 3d velocity
    pub m: f64, // mass
    pub radius: f64, // radius (softening)
}

#[derive(Debug, Clone)]
pub struct System3 {
    pub bodies: Vec<Body3>, // 3d collection of bodies
    pub t: f64, // time
}