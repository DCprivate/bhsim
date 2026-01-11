use bhsim::simulation::states::{Body3, System3};
use bhsim::simulation::params::Parameters;
use bhsim::simulation::forces::{NewtonianGravity3, AccelSet3};

/// Build a simple 2-body System3 separated along x-axis
pub fn two_body_system(dist: f64, m1: f64, m2: f64) -> System3 {
    let b1 = Body3 {
        x: [-dist / 2.0, 0.0, 0.0].into(),
        v: [0.0, 0.0, 0.0].into(),
        m: m1,
        radius: 0.0,
    };
    let b2 = Body3 {
        x: [dist / 2.0, 0.0, 0.0].into(),
        v: [0.0, 0.0, 0.0].into(),
        m: m2,
        radius: 0.0,
    };
    System3 {
        bodies: vec![b1, b2],
        t: 0.0,
    }
}

/// Default physics parameters for tests
pub fn test_params() -> Parameters {
    Parameters {
        t_end: 1.0,
        h0: 0.001,
        atol: 1e-9,
        rtol: 1e-6,
        merge_t: 0.0,
        seed: 42.0,
        eps2: 0.0,
        G: 0.1,
    }
}

/// Build a gravity term + AccelSet
pub fn gravity_set(p: &Parameters) -> AccelSet3 {
    AccelSet3::new().with(NewtonianGravity3 {
        G: p.G,
        eps2: p.eps2,
    })
}

// ==================================================================================
// Gravity tests
// ==================================================================================

#[test]
fn gravity_newton_third_law() {
    let sys = two_body_system(1.0, 2.0, 3.0);
    let p = test_params();
    let forces = gravity_set(&p);

    let mut acc = vec![Default::default(); 2];
    forces.accumulate_accels(sys.t, &sys, &mut acc);

    let a1 = acc[0];
    let a2 = acc[1];

    let net = a1 * sys.bodies[0].m + a2 * sys.bodies[1].m;

    assert!(net.norm() < 1e-12, "Net momentum not zero: {:?}", net);
}

#[test]
fn gravity_points_toward_other_body() {
    let sys = two_body_system(2.0, 1.0, 1.0);
    let p = test_params();
    let forces = gravity_set(&p);

    let mut acc = vec![Default::default(); 2];
    forces.accumulate_accels(sys.t, &sys, &mut acc);

    let dx = sys.bodies[1].x - sys.bodies[0].x;
    let a1 = acc[0];

    // Should point in same direction as +dx (negative sign for attraction)
    assert!(dx.norm() > 0.0);
    assert!(a1.dot(&dx) > 0.0, "Acceleration is not toward second body");
}

#[test]
fn gravity_inverse_square_law() {
    let sys_r = two_body_system(1.0, 1.0, 1.0);
    let sys_2r = two_body_system(2.0, 1.0, 1.0);
    let p = test_params();
    let forces = gravity_set(&p);

    let mut acc_r = vec![Default::default(); 2];
    let mut acc_2r = vec![Default::default(); 2];

    forces.accumulate_accels(sys_r.t, &sys_r, &mut acc_r);
    forces.accumulate_accels(sys_2r.t, &sys_2r, &mut acc_2r);

    let ratio = acc_r[0].norm() / acc_2r[0].norm();

    assert!((ratio - 4.0).abs() < 1e-3, "Expected ~4x, got {}", ratio);
}

#[test]
fn gravity_softening_prevents_blowup() {
    let mut p = test_params();
    p.eps2 = 0.1;

    let sys = two_body_system(1e-9, 1.0, 1.0);
    let forces = gravity_set(&p);

    let mut acc = vec![Default::default(); 2];
    forces.accumulate_accels(sys.t, &sys, &mut acc);

    assert!(acc[0].norm() < 1e9, "Softening failed; acceleration too large");
}

// ==================================================================================
// Integrator tests
// ==================================================================================

// ==================================================================================
// Barnes-Hut tests
// ==================================================================================