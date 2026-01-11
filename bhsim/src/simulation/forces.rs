//! Force / acceleration contributors for the n-body engine
//!
//! Defines 2D and 3D acceleration traits, including direct
//! Newtonian gravity and a Barnes–Hut–based variant

use crate::simulation::states::{System, System3, NVec2, NVec3};
use crate::simulation::barnes_hut::BarnesHutTree3;

/// Collection of 2D acceleration terms (gravity, drag, etc.)
/// Each term implements [`Acceleration`] and their contributions are summed
/// into a single acceleration vector per body
pub struct AccelSet {
    terms: Vec<Box<dyn Acceleration + Send + Sync>>,
}

impl AccelSet {
    /// Create an empty acceleration set
    pub fn new() -> Self {
        Self {
            terms: Vec::new()
        }
    }

    /// Add an acceleration term
    pub fn with<T>(mut self, term: T) -> Self
    where
        T: Acceleration + Send + Sync + 'static,
    {
        self.terms.push(Box::new(term));
        self
    }

    /// Compute total accelerations at time `t` for all bodies in `sys`
    /// - `out[i]` will be set to the sum of contributions from all terms
    pub fn accumulate_accels(&self, t: f64, sys: &System, out: &mut [NVec2]) {
        // Zero buffer
        for a in out.iter_mut() {
            *a = NVec2::zeros();
        }
        // Iterate over all acceration contributors
        for term in &self.terms {
            term.acceleration(t, sys, out);
        }
    }
}

/// Trait for 2D acceleration sources operating on [`System`]
/// Implementations add their contribution into `out[i]` for each body
pub trait Acceleration {
    fn acceleration(&self, t: f64, sys: &System, out: &mut [NVec2]);
}

/// 2D Newtonian gravity with softening
/// Uses body radius and eps2 to smooth close encounters and avoid
/// singularities at small separations
pub struct NewtonianGravity {
    pub G: f64, // gravitional constant
    pub eps2: f64, // softening
}

impl Acceleration for NewtonianGravity {
    fn acceleration(&self, _t: f64, sys: &System, out: &mut [NVec2]) {
        let n = sys.bodies.len();
        if n == 0 { // No bodies, return
            return;
        }

        // Loop over each unordered pair (i, j) with i < j
        for i in 0..n {
            // bi: body i (left side of the pair)
            let bi = &sys.bodies[i];
            let xi = bi.x;      // position of body i
            let mi = bi.m;      // mass of body i

            for j in (i + 1)..n {
                // bj: body j (right side of the pair)
                let bj = &sys.bodies[j];
                let xj = bj.x;  // position of body j
                let mj = bj.m;  // mass of body j

                // r is the displacement vector from i to j
                // If r points from i to j, then i feels a pull along +r,
                // j feels a pull along -r
                let r = xj - xi;

                // Squared separation distance |r|^2 (no softening yet)
                let r2 = r.dot(&r);

                // Per-pair softening:
                // - bi.radius and bj.radius are the "core" sizes of each body
                // - Wtake the average of their squared radii:
                //     0.5 * (ri^2 + rj^2)
                // - Then add self.eps2 as a global numerical safety floor
                //
                // This means close interactions are smoothed based on the
                // sizes of both bodies + a minimum softening
                let soft2 = 0.5 * (bi.radius * bi.radius + bj.radius * bj.radius) + self.eps2;

                // Total softened squared distance:
                // d2 = |r|^2 + softening^2
                let d2 = r2 + soft2;

                // 1 / |r_soft|
                let inv_r = d2.sqrt().recip();

                // 1 / |r_soft|^3
                // (this is what appears in the Newtonian acceleration formula:
                //   a = r / |r|^3
                //   => a = r * (1 / |r|^3) )
                let inv_r3 = inv_r * inv_r * inv_r;

                // Combine G and the distance factor:
                // coef = G / |r_soft|^3
                let coef = self.G * inv_r3;

                // -------------------------
                // Apply Newton's law:
                // a_i +=  G * m_j * r / |r_soft|^3
                // a_j += -G * m_i * r / |r_soft|^3
                // (equal and opposite)
                // -------------------------

                // Acceleration on body i due to body j:
                // direction: along +r (toward j)
                // magnitude scaled by mass of j
                out[i] += coef * mj * r;

                // Acceleration on body j due to body i:
                // direction: along -r (toward i)
                // magnitude scaled by mass of i
                out[j] -= coef * mi * r;
            }
        }
    }
}

// =========================================================================================
// 2D stuff above
// 3D stuff below
// =========================================================================================

/// Collection of 3D acceleration terms (gravity, drag, etc)
/// Each term implements [`Acceleration`] and their contributions are summed
/// into a single acceleration vector per body
pub struct AccelSet3 {
    terms: Vec<Box<dyn Acceleration3 + Send + Sync>>,
}

impl AccelSet3 {
    /// Constructor
    pub fn new() -> Self {
        Self {
            terms: Vec::new(),
        }
    }

    /// Add an acceleration term
    pub fn with(mut self, term: impl Acceleration3 + Send + Sync + 'static) -> Self {
        self.terms.push(Box::new(term));
        self
    }

    /// Compute total accelerations at time `t` for all bodies in `sys`
    /// - `out[i]` will be set to the sum of contributions from all terms
    pub fn accumulate_accels(&self, t: f64, sys: &System3, out: &mut [NVec3]) {
        // Zero buffer
        for a in out.iter_mut() {
            *a = NVec3::zeros();
        }
        // Iterate over all acceration contributors
        for term in &self.terms {
            term.acceleration(t, sys, out);
        }
    }
}

/// Trait for 3D acceleration sources operating on [`System3`]
pub trait Acceleration3 {
    fn acceleration(&self, t: f64, sys: &System3, out: &mut [NVec3]);
}

/// 3D Newtonian gravity with softening (direct n^2 sum)
pub struct NewtonianGravity3 {
    pub G: f64,
    pub eps2: f64,
}

impl Acceleration3 for NewtonianGravity3 {
    fn acceleration(&self, t: f64, sys: &System3, out: &mut [NVec3]) {
        // Number of bodies in the system.
        let n = sys.bodies.len();
        if n == 0 { // No bodies, return
            return;
        }

        // Loop over each unordered pair (i, j) with i < j
        for i in 0..n {
            // bi: body i (left side of the pair)
            let bi = &sys.bodies[i];
            let xi = bi.x;      // position of body i
            let mi = bi.m;      // mass of body i

            for j in (i + 1)..n {
                // bj: body j (right side of the pair)
                let bj = &sys.bodies[j];
                let xj = bj.x;  // position of body j
                let mj = bj.m;  // mass of body j

                // r is the displacement vector from i to j.
                // If r points from i to j, then i feels a pull along +r,
                // j feels a pull along -r.
                let r = xj - xi;

                // Squared separation distance |r|^2 (no softening yet).
                let r2 = r.dot(&r);

                // Per-pair softening:
                // - bi.radius and bj.radius are the "core" sizes of each body.
                // - Take the average of their squared radii:
                //     0.5 * (ri^2 + rj^2)
                // - Then add self.eps2 as a global numerical safety floor.
                //
                // This means close interactions are smoothed based on the
                // sizes of both bodies + a minimum softening.
                let soft2 = 0.5 * (bi.radius * bi.radius + bj.radius * bj.radius) + self.eps2;

                // Total softened squared distance:
                // d2 = |r|^2 + softening^2
                let d2 = r2 + soft2;

                // 1 / |r_soft|
                let inv_r = d2.sqrt().recip();

                // 1 / |r_soft|^3
                // (this is what appears in the Newtonian acceleration formula:
                //   a = r / |r|^3
                //   => a = r * (1 / |r|^3) )
                let inv_r3 = inv_r * inv_r * inv_r;

                // Combine G and the distance factor:
                // coef = G / |r_soft|^3
                let coef = self.G * inv_r3;

                // -------------------------
                // Apply Newton's law:
                // a_i +=  G * m_j * r / |r_soft|^3
                // a_j += -G * m_i * r / |r_soft|^3
                // (equal and opposite)
                // -------------------------

                // Acceleration on body i due to body j:
                // direction: along +r (toward j)
                // magnitude scaled by mass of j.
                out[i] += coef * mj * r;

                // Acceleration on body j due to body i:
                // direction: along -r (toward i)
                // magnitude scaled by mass of i.
                out[j] -= coef * mi * r;
            }
        }
    }
}

// =========================================================================================
// 3D Barnes-hut implementation 
// =========================================================================================

/// 3D Newtonian gravity evaluated via a Barnes–Hut octree
/// Wraps [`BarnesHutTree3`] to get approximate O(N log N) accelerations
/// controlled by `theta` (opening angle) and `eps2` (softening)
pub struct NewtonianGravityBarnesHut3 {
    pub G: f64,
    pub eps2: f64,
    pub theta: f64,
}

impl Acceleration3 for NewtonianGravityBarnesHut3 {
    /// Compute 3D accelerations using a Barnes–Hut tree built from `sys`
    fn acceleration(&self, t: f64, sys: &System3, out: &mut [NVec3]) {
        let tree = BarnesHutTree3::build(sys);
        for i in 0..sys.bodies.len() {
            out[i] = tree.force_on_body(i, sys, self.G, self.eps2, self.theta);
        }
    }
}