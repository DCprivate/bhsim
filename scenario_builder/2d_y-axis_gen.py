'''
Built by AI with the prompt "Build a scenario generator that randomly distrubutes bodies around a center mass",
I also gave the prompt an example yaml file, I then did some tweeking to fit my needs.

Prerequisites:
    python3 installed

Usage:
    python nameofgenerator.py <number of bodies> nameofoutputfile.yaml
    
    Example:
        python y-axis_gen.py 200 y-axis_orbit.yaml
'''

#!/usr/bin/env python3
import math
import random
import sys

try:
    import yaml  # pip install pyyaml
except ImportError:
    print("Please 'pip install pyyaml' first.", file=sys.stderr)
    sys.exit(1)


# Custom list type so only x/v use inline [a, b, c] format
class Vec(list):
    pass


def vec_representer(dumper, data):
    # Print as [x, y, z] (flow style)
    return dumper.represent_sequence("tag:yaml.org,2002:seq", data, flow_style=True)


# Register on SafeDumper so safe_dump can handle Vec
yaml.SafeDumper.add_representer(Vec, vec_representer)


def generate_config_yaxis(
    num_bodies=100,
    G=0.1,
    central_mass=200.0,
    central_radius=0.05,
    r_min=1.0,      # distance from origin in the xz-plane
    r_max=5.0,
    y_spread=0.3,   # how far along the y-axis bodies can be
    vel_perturb=0.1,
    vy_spread=0.05,
    seed=42,
):
    random.seed(seed)

    config = {
        "engine": {
            "dimension": True,
            "integrator": "verlet",
            "barnes_hut": True,
            "theta": 0.7,
        },
        "parameters": {
            "t_end": 200.0,
            "h0": 0.001,
            "atol": 1.0e-9,
            "rtol": 1.0e-6,
            "merge_t": 1.0e-3,
            "seed": float(seed),
            "eps2": 1.0e-4,
            "G": G,
        },
        "bodies": [],
    }

    # Central massive body at origin
    config["bodies"].append(
        {
            "x": Vec([0.0, 0.0]),
            "v": Vec([0.0, 0.0]),
            "m": central_mass,
            "radius": central_radius,
        }
    )

    for _ in range(num_bodies):
        # Cylindrical coords around the y-axis:
        # (x, z) on a ring, y ~ small
        rho = random.uniform(r_min, r_max)             # radius in the xz-plane
        phi = random.uniform(0.0, 2.0 * math.pi)       # angle in xz-plane
        y = random.uniform(-y_spread, y_spread)        # small offset along y-axis

        x = rho * math.cos(phi)
        z = rho * math.sin(phi)

        # distance from origin for gravity
        r_norm = math.sqrt(rho * rho + y * y)

        # circular orbit speed around central point mass at origin
        v_circ = math.sqrt(G * central_mass / r_norm)

        # Tangential direction in xz-plane (orbiting the y-axis):
        # radial in xz-plane is (cosφ, 0, sinφ)
        # tangential is      (-sinφ, 0, cosφ)
        base = 1.0 + random.uniform(-vel_perturb, vel_perturb)

        vx = base * v_circ * (-math.sin(phi))
        vz = base * v_circ * ( math.cos(phi))
        vy = random.uniform(-vy_spread, vy_spread)     # small motion along y

        # body mass / radius
        m = random.uniform(0.5, 1.0)
        radius = 0.015 * (m ** (1.0 / 3.0))

        config["bodies"].append(
            {
                "x": Vec([x, y]),
                "v": Vec([vx, vy]),
                "m": m,
                "radius": radius,
            }
        )

    return config


def main():
    # Usage: python gen_config_yaxis.py [num_bodies] [output.yaml]
    if len(sys.argv) >= 2:
        num_bodies = int(sys.argv[1])
    else:
        num_bodies = 100

    if len(sys.argv) >= 3:
        out_path = sys.argv[2]
        out_file = open(out_path, "w", encoding="utf-8")
    else:
        out_file = sys.stdout

    cfg = generate_config_yaxis(num_bodies=num_bodies)

    yaml.safe_dump(
        cfg,
        out_file,
        sort_keys=False,
        default_flow_style=False,
        indent=2,
    )

    if out_file is not sys.stdout:
        out_file.close()


if __name__ == "__main__":
    main()