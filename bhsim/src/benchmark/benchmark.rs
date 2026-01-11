use std::time::Instant;
use crate::simulation::states::{Body3, System3, NVec3};
use crate::simulation::params::Parameters;
use crate::simulation::forces::{AccelSet3, Acceleration3, NewtonianGravity3, NewtonianGravityBarnesHut3};
use crate::simulation::integrator::{verlet_integrator_3d};

pub fn bench_gravity() {
    // Different system sizes to test
    let ns = [200, 400, 800, 1600, 3200, 6400]; //, 12800, 25600, 51200];

    for n in ns {
        // Build a manual System3
        let mut bodies = Vec::with_capacity(n);

        for i in 0..n {
            let i_f = i as f64;
            // deterministic positions, no rand needed
            let x = NVec3::new(
                (i_f * 0.37).sin() * 5.0,
                (i_f * 0.13).cos() * 5.0,
                (i_f * 0.07).sin() * 5.0,
            );
            let v = NVec3::zeros();
            let m = 1.0;

            bodies.push(Body3 {
                x,
                v,
                m,
                radius: 0.01,
            });
        }

        let parameters = Parameters {
            t_end: 100.0,
            h0: 0.001,
            atol: 1.0e-9,
            rtol: 1.0e-6,
            merge_t: 1.0e-3,
            seed: 42.0,
            eps2: 1e-4,
            G: 0.1,
        };

        let sys = System3 { bodies, t: 0.0};

        let mut out = vec![NVec3::zeros(); n];

        // Set up gravity models
        let direct = NewtonianGravity3 {
            G: parameters.G,
            eps2: parameters.eps2,
        };

        let bh = NewtonianGravityBarnesHut3 {
            G: parameters.G,
            eps2: parameters.eps2,
            theta: 0.7,
        };

        // Warm up
        direct.acceleration(0.0, &sys, &mut out);
        bh.acceleration(0.0, &sys, &mut out);

        // Time direct
        let t0 = Instant::now();
        direct.acceleration(0.0, &sys, &mut out);
        let dt_direct = t0.elapsed().as_secs_f64();

        // Time barnes-hut
        let t1 = Instant::now();
        bh.acceleration(0.0, &sys, &mut out);
        let dt_bh = t1.elapsed().as_secs_f64();

        println!("N = {n:5}, direct = {:8.6} s, BH = {:8.6} s", dt_direct, dt_bh);
    }
}

pub fn bench_verlet() {
    // Test different N values
    let ns = [200, 400, 800, 1600, 3200, 6400, 12800];
    let steps = 2; // number of integrator steps per model (tune as needed)

    for n in ns {
        // =======================================================
        // Build bodies
        // =======================================================
        let mut bodies = Vec::with_capacity(n);

        for i in 0..n {
            let i_f = i as f64;

            let x = NVec3::new(
                (i_f * 0.37).sin() * 5.0,
                (i_f * 0.13).cos() * 5.0,
                (i_f * 0.07).sin() * 5.0,
            );

            bodies.push(Body3 {
                x,
                v: NVec3::zeros(),
                m: 1.0,
                radius: 0.01,
            });
        }

        // Mutable system between steps
        let sys_template = System3 { bodies, t: 0.0 };

        // Parameters
        let mut params = Parameters {
            t_end: 100.0,
            h0: 0.001,
            atol: 1.0e-9,
            rtol: 1.0e-6,
            merge_t: 1.0e-3,
            seed: 42.0,
            eps2: 1e-4,
            G: 0.1,
        };

        // Direct Gravity benchmark
        let mut sys_direct = sys_template.clone();

        // Build AccelSet3 and attach direct gravity
        let mut forces_direct = AccelSet3::new().with(NewtonianGravity3 {
            G: params.G,
            eps2: params.eps2,
        });

        // Warm-up
        verlet_integrator_3d(&mut sys_direct, &mut forces_direct, &mut params);

        let t0 = Instant::now();
        for _ in 0..steps {
            verlet_integrator_3d(&mut sys_direct, &mut forces_direct, &mut params);
        }
        let direct_per_step = t0.elapsed().as_secs_f64() / steps as f64;


        // Barnes–Hut benchmark
        let mut sys_bh = sys_template.clone();

        let mut forces_bh = AccelSet3::new().with(NewtonianGravityBarnesHut3 {
            G: params.G,
            eps2: params.eps2,
            theta: 0.7,
        });


        // Warm-up
        verlet_integrator_3d(&mut sys_bh, &mut forces_bh, &mut params);

        let t1 = Instant::now();
        for _ in 0..steps {
            verlet_integrator_3d(&mut sys_bh, &mut forces_bh, &mut params);
        }
        let bh_per_step = t1.elapsed().as_secs_f64() / steps as f64;

        println!("N = {:5}, direct step = {:8.6} s,   BH step = {:8.6} s", n, direct_per_step, bh_per_step);
    }
}

/// Helper to build a manual System of size `n`
fn make_system3(n: usize) -> System3 {
    let mut bodies = Vec::with_capacity(n);

    for i in 0..n {
        let i_f = i as f64;
        let x = NVec3::new(
            (i_f * 0.37).sin() * 5.0,
            (i_f * 0.13).cos() * 5.0,
            (i_f * 0.07).sin() * 5.0,
        );

        bodies.push(Body3 {
            x,
            v: NVec3::zeros(),
            m: 1.0,
            radius: 0.01,
        });
    }

    System3 { bodies, t: 0.0 }
}

/// Helper to build a manual System of size `n`
fn make_params() -> Parameters {
    Parameters {
        t_end: 100.0,
        h0: 0.001,
        atol: 1.0e-9,
        rtol: 1.0e-6,
        merge_t: 1.0e-3,
        seed: 42.0,
        eps2: 1e-4,
        G: 0.1,
    }
}

/// Benchmark the real verlet_integrator_3d for a range of n
/// Paste output directly into excel to graph
pub fn bench_verlet_curve() {

    println!("N,direct_ms,bh_ms");

    // Steps of 200 to give smoother graph
    for n in (200..=12800).step_by(200) {
        // Small n: average over a few steps to smooth noise
        // Large n: only 1 step to avoid minutes of runtime
        let steps_direct = if n <= 800 { 5 } else { 1 };
        let steps_bh = if n <= 2000 { 3 } else { 1 };

        let sys_template = make_system3(n);

        // Shared parameter template
        let params_template = make_params();

        // Direct gravity
        let mut sys_direct = sys_template.clone();
        let mut params_direct = params_template.clone();

        let mut forces_direct = AccelSet3::new().with(NewtonianGravity3 {
            G: params_direct.G,
            eps2: params_direct.eps2,
        });

        // Warm-up one step
        // if n <= 800 {
        //     let _ = Instant::now();
        //     verlet_integrator_3d(&mut sys_direct, &mut forces_direct, &mut params_direct);
        // }

        let t0 = Instant::now();
        for _ in 0..steps_direct {
            verlet_integrator_3d(&mut sys_direct, &mut forces_direct, &mut params_direct);
        }
        let elapsed_direct = t0.elapsed().as_secs_f64() * 1000.0; // ms total
        let ms_direct = elapsed_direct / steps_direct as f64;

        // Barnes–Hut gravity
        let mut sys_bh = sys_template.clone();
        let mut params_bh = params_template.clone();

        let mut forces_bh = AccelSet3::new().with(NewtonianGravityBarnesHut3 {
            G: params_bh.G,
            eps2: params_bh.eps2,
            theta: 0.7,
        });

        // Warm-up one step
        // if n <= 800 {
        //     let _ = Instant::now();
        //     verlet_integrator_3d(&mut sys_bh, &mut forces_bh, &mut params_bh);
        // }

        let t1 = Instant::now();
        for _ in 0..steps_bh {
            verlet_integrator_3d(&mut sys_bh, &mut forces_bh, &mut params_bh);
        }
        let elapsed_bh = t1.elapsed().as_secs_f64() * 1000.0; // ms total
        let ms_bh = elapsed_bh / steps_bh as f64;

        println!("{},{:.6},{:.6}", n, ms_direct, ms_bh);
    }
}