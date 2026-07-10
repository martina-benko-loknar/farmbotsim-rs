use crate::experiment::runner::run_single_station_experiment;
use crate::experiment::sweeps::sweep_utils::create_sweep_output_directory;
use crate::experiment::sweeps::sweep_utils::print_experiment_info;
use crate::experiment::models::{ExperimentType, ExperimentInfo};
use crate::experiment::config::ExperimentConfig;

pub fn run_fleet_sweep(
    output_dir: &str
) -> Result<(), Box<dyn std::error::Error>> {

    let fleet_sizes = vec![1, 2, 3, 4];
    let seeds = 0..5;
    let resolution = 15;
    let label = "M";
    let output_dir_sweep = create_sweep_output_directory(output_dir, "raw/fleet_sweep")?;
  
    println!("\n===== EXPERIMENT: Fleet size sweep ===================================");

    for n_agents in fleet_sizes {

        for seed in seeds.clone() {

            let exp = ExperimentConfig {
                seed,
                n_agents: n_agents as u32,
                ..Default::default()
            };

            print_experiment_info(label, &exp);

            let filename=  format!(
                "size={}_fleet={}_batt={}_soc={}",
                label,
                exp.n_agents,
                exp.battery_capacity_wh,
                exp.soc_threshold_percent,
            );

            let timestamp = chrono::Utc::now()
                .format("%H%M%S")
                .to_string();

            let info = ExperimentInfo {
                experiment_type: ExperimentType::FleetSweep,
                timestamp: timestamp.clone(),
            };

            run_single_station_experiment(
                resolution,
                &filename,
                &output_dir_sweep,
                exp, 
                info
            );

            println!("======================================================================\n");

        }
    }

    Ok(())
}