## Overview

This project is a small physics engine and visualization tool for exploring
gravitational n-body systems in 2D and 3D, focused on
black-hole objects and orbital dynamics.

## Bit of commentary

A glaring problem (among many I am sure), is the duplicate code from 2D to 3D
The plan was to refactor to choose between using NVec2 and NVec3 dynamically based on the scenario file read, but I kept building out instead and don't have time to fix this problem before the deadline.

Some other up front problems that need to be addressed before moving onto parallelizing: It isn't terrible, but fmt tells me there is import spaghetti. Also the acceleration function doesn't need to have time as a parameter, that is apart of System (this is a relic)


## Features
- **n-body gravitational dynamics**
  - Modeled acceleration for multiple acceleration contributors
  - Direct O(n^2) newtonian gravity for small systems (more accurate)
  - Optional Barnes–Hut octree (quadtree for 2D not implemented) for O(nlogn) scaling at larger n
- **Integrator**
  - Choose between single/double verlet or rk4 (Runge-Kutta)
  - rk4 is not available yet
- **2D and 3D modes**
  - 2D simpler scenarios
  - 3D for more realistic orbits and black-hole scenarios
- **Scenarios built in yaml**
  - Engine parameters (dimension, integrator, Barnes–Hut on/off, etc.)
  - Physics simulation parameters (timestep, total time, softening, G, merge threshold, seed)
  - Body list (position, velocity, mass, radius)
- **Bevy visualization**
  - 3D camera, and dimension axes for visual reference
  - One sphere per body
  - Velocity-based color mapping
  - Size of object scales with distance to camera
- **Benchmarking**
  - Timing brute-force vs Barnes-Hut
  - Timing of physics step (ms per frame)
  - Benchmarking is manual, not configurable right now

## Future features — (Stretch Goals)
- **Optimizations:**
    - Parallelize with rayon (easier)
    - Math on gpu (much harder)
- **Bevy changes:**
    - Different coloring schemes (redshifting, distance to center)
    - Live data on screen
- **Scenario generator changes:**
    - Scenario generator dump yaml into the scenarios folder
    - Configurable generator (change xis, shape, distribution)
    - Dimension agnostic
- **Collisions/detection**
- **Relativistic corrections:**
  - Post-Newtonian terms (PN1, PN2)
- **Gravitational waves**
  - Emission and absorbtion
  - Orbital decay
- **Merging/ringdown models:**
  - Final spin and mass formulas
- **Accretion**
- **Frame dragging (Kerr metric approximations)**
- **Hawking radiation (toy models)**


## Usage

    Prerequisites:
        Cargo installed
        System that can run Bevy (wgpu)
            - Linux (Vulkan/GL)
            - Windows (DirectX or Vulkan)
            - Mac (?)
        Scenario files in a folder named "scenarios" in the project root or specify path

    Build:
        cargo build
        or
        cargo build --release

    Run:
        Use -f flag to specify yaml file
        Use --release to run in release mode

        Default scenario:
            cargo run
            or
            cargo run --release
        Choose scenario:
            cargo run -- -f filename.yaml
            or
            cargo run --release -- -f filename.yaml

        Example: cargo run -- -f x-axis_orbit.yaml


## Scenario builder
This is a python script included in the "scenario_builder" folder just outside the project root.

For now the yaml file needs to be dragged into the "scenarios" folder.

There is only one generator available for now, I have more but they are all duplicate code, and I'd like to make it robust before I make it available.

    Prerequisites:
        python3 installed
        Install pyyaml with:
            pip install pyyaml

    Run:
        python nameofgenerator.py numberofbodies outputfile.yaml
        
        Example:
            python 3D_y-axis_gen.py 200 y-axis_orbit.yaml


## Example scenario yaml file

    engine:
      dimension: true        # false = 2D, true = 3D
      integrator: "verlet"   # currently only verlet
      barnes_hut: true       # true = use Barnes–Hut O(nlogn), false = direct O(n^2)
      theta: 0.7             # opening angle for Barnes-Hut approximatio (lower = higher accuracy)

    parameters:
      t_end: 200.0           # total simulation time (in simulation units)
      h0: 0.001              # fixed timestep
      atol: 1.0e-9           # absolute tolerance (for future (rk4) adaptive methods)
      rtol: 1.0e-6           # relative tolerance
      merge_t: 1.0e-3        # collision/merge threshold
      seed: 42.0             # RNG seed for deterministic scenarios
      eps2: 1.0e-4           # softening (to avoid divide by 0)
      G: 0.1                 # gravitational constant

    bodies:
    - x: [0.0, 0.0, 0.0]     # position in simulation units
      v: [0.0, 0.0, 0.0]     # velocity in simulation units / time
      m: 1000.0              # mass
      radius: 0.05           # radius (for merging + visualization scaling)

    - x: [3.0, 0.0, 0.0]
      v: [0.0, 1.0, 0.0]
      m: 1.0
      radius: 0.01

    - x: [-4.0, 0.5, 0.0]
      v: [0.0, -0.8, 0.0]
      m: 0.8
      radius: 0.01

    ...


### Crates:
    bevy: real time visualization and UI (uses wgpu)
        - 2D planar viewer first, and 3D viewer after, drawing bodies to screen

    nalgebra: vector/matrix math
        - positions/velocities, dot/norm/cross operations

    rayon: parallelism

    serde: serialization
        - config, scenarios and starting conditions

    ode_solvers: implements RK45 (Runge-Kutta) methods
        - dopri5 (Dormand-Prince) method will be used

    others: 
        - clap (CLI)
        - approx (testing)
        - criterion (benchmarking)
        - tracing (logging)


### File structure and data structures:
    - bhsim/

        - scenarios/
            - scenario.yaml

        - src/
            - benchmark/
                - benchmark.rs

            - configuration/
                - config.rs

            - simulation/
                - barnes_hut.rs
                - engine.rs
                - forces.rs
                - integrator.rs
                - params.rs
                - scenario.rs
                - states.rs

            - visualization/
                - bhsim-vis2d.rs
                - bhsim-vis3d.rs

        - lib.rs
        - main.rs

        - tests/
            - two_body_tests.rs
            - forces_tests.rs
            - bench_tests.rs


## Description of above file structure:
### bhsim/

### `scenarios/`

    Example / saved YAML files that describe initial conditions and engine settings.

    These get loaded, deserialized into config structs, and then turned into runtime scenarios.

---

### `visualization/`

#### `bhsim-vis2d/`
    2D viewer for the simulation:

    - Reads the runtime `Scenario` / `System`.
    - Renders bodies in 2D (positions, trails, velocity based coloring).

#### `bhsim-vis3d/`
    Future 3D viewer:

    - Same idea, but using full 3D positions/velocities when the engine supports them.
    - Includes velocity based coloring

---

### `simulation/`

#### `engine.rs`

    Defines `Engine`:

    - `dimension`: 2D vs 3D.
    - `integrator`: which integrator to use (`double/single verlet`, `rk4`).
    - `barnes_hut`: whether to use direct N² forces or a Barnes–Hut tree.

    High-level runtime engine settings, usually coming from `EngineConfig`.



#### `states.rs`

    Core runtime state containers:

    - `struct Body { x: Vector3<f64>, v: Vector3<f64>, m: f64, radius: f64 }`  
    One simulated object: position, velocity, mass, and an effective radius (for collisions/softening/visualization).

    - `struct System { bodies: Vec<Body>, params: Parameters, time: f64 }`  
    The full N-body state at a single time:
    - All bodies.
    - The current simulation parameters (or a reference to them).
    - Current simulation time.

#### `forces.rs`

    Abstractions for “things that produce accelerations”:

    - `trait Acceleration`  
    Something that can compute acceleration vectors for all bodies at a given time:  
    `fn accelerations(&self, t, sys, out)` fills `out[i]` with contributions.

    - `struct AccelSet { terms: Vec<Box<dyn Acceleration>> }`  
    A collection of acceleration sources (gravity, external fields, etc.).  
    Provides a method like:
    - `fn accumulate_accels(...)`  
        Zeroes the buffer, then asks each term to add its contribution.

    - `struct NewtonianGravity { G, eps2 }`  
    Implements pure Newtonian gravity with softening.  
    `impl Acceleration for NewtonianGravity`:  
    Loops over body pairs and adds pairwise gravitational acceleration into the `out` buffer.


#### `integrator.rs`

    Time integration and ODE wiring.

    Fixed-step integrators:

    - `fn verlet_integrator(sys: &mut System, forces: &mut AccelSet, params: &mut Parameters)`  
    Symplectic velocity-Verlet step:
    - Uses `AccelSet` to compute accelerations.
    - Updates positions and velocities by one fixed `dt = h0`.

    - `fn rk4_integrator(...)`  
    Classical 4th-order Runge–Kutta step:
    - Uses multiple force evaluations (`k1..k4`) for higher local accuracy.

    **ODE solver adapter:**

    - `struct NBodyModel { n, masses, accels, sys, accel_set }`  
    Wrapper that adapts the current N-body system to the `ode_solvers` crate’s `OdeSystem` interface.  
    Knows:
    - How many bodies.
    - Their masses.
    - Where to store temporary accelerations.
    - How to call the force set.

    - `impl NBodyModel { fn rhs(...) }`  
    Right-hand side of the ODE:
    - Given state `(x, v)` and time `t`, computes time-derivatives `(dx/dt, dv/dt)`.

    **Packing/unpacking helpers:**

    - `fn pack_state(&System) -> Vec<f64>`  
    Flatten positions and velocities into a single state vector for `ode_solvers`.

    - `fn unpack_state(Vec<f64>) -> (System)` or `(x, v)`  
    Take the solver’s state vector and map it back to structured positions/velocities.


#### `params.rs`

    - `struct Parameters { t_end, h0, atol, rtol, merge_t, seed, eps2, G }`  

    Runtime version of the numerical parameters.

    Used directly by integrators and forces:

    - `h0` is the current base timestep.  
    - `atol`, `rtol` for future adaptive schemes.  
    - `eps2`, `G` for force laws.  
    - `t_end`, `merge_t`, `seed` for run control.


#### `scenario.rs`

    - struct Scenario { engine, parameters, system, forces } 

    The fully-initialized runtime “bundle”:
    - `engine`: chosen integrator, dimension, BH flag, etc.
    - `parameters`: runtime parameters.
    - `system`: bodies + time.
    - `forces`: acceleration set (e.g. Newtonian gravity).

    - impl Scenario { fn build_scenario(cfg: ScenarioConfig) }

    Glue from config to runtime:
    - Takes `ScenarioConfig` from YAML.
    - Builds `Engine`, `Parameters`, `System` (bodies at `t = 0`).
    - Initializes `AccelSet` (e.g. adds `NewtonianGravity`).
    - Returns a ready-to-run `Scenario`.

---

### `configuration/`

#### `config.rs`

    YAML-facing configuration layer (pure data, deserialized via `serde`).

    - struct ScenarioConfig { engine, parameters, bodies }  
    Top-level config matching the YAML file layout.

    - struct EngineConfig { ... } 
    Config-time version of engine settings:  
    `dimension`, `integrator`, `barnes_hut`, `theta`, etc.

    - struct ParametersConfig { time_end, h0, atol, rtol, merge_t, seed, eps2, G }
    All numerical knobs as read from YAML.

    - struct BodyConfig { x: Vec<f64>, v: Vec<f64>, m: Vec<f64> }
    Initial positions, velocities, masses as simple arrays.

    - enum MergeMode { ... }`, `enum CollisionMode { ... }

    Configuration for how close encounters / overlaps are handled.  
    For now set to defaults (e.g. rigid merging), more advanced behavior later.

    These types exist only to map YAML → Rust; they’re later converted into runtime `Engine`, `Parameters`, `System`, etc.

---

### `main.rs`

    Entry point:

    - `fn load_scenario_from_yaml() -> Result<ScenarioConfig>`  
    Reads a YAML file from `scenarios/`, deserializes it into `ScenarioConfig`.

    Constructs a `Scenario` via `Scenario::build_scenario`.

    Kicks off the simulation and visualization.

---

### `tests/`
#### Gravity tests
    `gravity_newton_third_law`
    `gravity_points_toward_other_body`
    `gravity_inverse_square_law`
    `gravity_softening_prevents_blowup`

---

### `benchmarking/`

#### `bench_tests.rs`
    For now benchmarking is manual, I will be including ways to configure benchmarking in the future.
    Benchmark-style tests:
    - Performance scaling with number of bodies.
    - Compare different integrators or force implementations.

---
