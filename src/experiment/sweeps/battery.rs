use crate::experiment::runner::run_single_station_experiment;
use crate::experiment::sweeps::sweep_utils::create_sweep_output_directory;
use crate::experiment::sweeps::sweep_utils::print_experiment_info;
use crate::experiment::models::{ExperimentType, ExperimentInfo};
use crate::experiment::config::ExperimentConfig;

pub fn run_battery_sweep(
    output_dir: &str
) -> Result<(), Box<dyn std::error::Error>> {

    let capacities = vec![65.0, 70.0, 75.0, 80.0];
    let seeds = 0..5;
    let resolution = 15;
    let label = "M";
    let output_dir_sweep = create_sweep_output_directory(output_dir, "raw/battery_sweep")?;


    println!("\n===== EXPERIMENT: Battery capacity sweep ===================================");

    for battery_capacity_wh in capacities {

        for seed in seeds.clone() {

            let exp = ExperimentConfig {
                seed,
                battery_capacity_wh: battery_capacity_wh as f32,
                ..Default::default()
            };

            print_experiment_info(label, &exp);

            let filename=  format!(
                "size={}_fleet={}_batt={}_soc={}",
                label,
                exp.n_agents,
                battery_capacity_wh,
                exp.soc_threshold_percent,
            );

            let timestamp = chrono::Utc::now()
                .format("%H%M%S")
                .to_string();

            let info = ExperimentInfo {
                experiment_type: ExperimentType::BatterySweep,
                timestamp: timestamp.clone(),
            };

            run_single_station_experiment(
                resolution,
                &filename,
                &output_dir_sweep,
                exp,
                info,
            );

            println!("======================================================================\n");
        }
    }

    Ok(())
}