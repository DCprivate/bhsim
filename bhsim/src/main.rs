use bhsim::{ScenarioConfig, Scenario, Scenario3D};
use bhsim::{run_2d, run_3d};
use bhsim::{bench_gravity, bench_verlet, bench_verlet_curve};

use clap::Parser;
use anyhow::Result;
 
use std::fs::File;
use std::io::BufReader;
use std::path::PathBuf;

#[derive(Parser, Debug)]
struct Args {
    #[arg(short, default_value = "test_file.yaml")]
    file_name: String,
}

// load here to keep main clean
fn load_scenario_from_yaml() -> Result<ScenarioConfig> {
    let args = Args::parse();
    let file_name = args.file_name;
    //println!("{}", file_name);

    let config_path = PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("scenarios").join(&file_name);
    let file = File::open(&config_path)?;
    let reader = BufReader::new(file);
    let scenario_cfg: ScenarioConfig = serde_yaml::from_reader(reader)?;

    //println!("{:?}", scenario_cfg);

    Ok(scenario_cfg)
}

fn main() -> Result<()> {

    let scenario_cfg = load_scenario_from_yaml().expect("failed to load scenario");

    //println!("{:?}", scenario);
    if scenario_cfg.engine.dimension == false {
        let scenario = Scenario::build_scenario(scenario_cfg);
        run_2d(scenario);
    }
    else {
        let scenario = Scenario3D::build_scenario_3d(scenario_cfg);
        run_3d(scenario);
    }

    //bench_gravity();
    //bench_verlet();
    //bench_verlet_curve();

    Ok(())
}