use crate::experiment::runner::run_single_station_experiment;
use crate::experiment::output::create_results_subdir;
use crate::experiment::sweeps::sweep_utils::print_experiment_info;
use crate::experiment::models::{ExperimentType, ExperimentInfo};
use crate::ExperimentConfig;

pub fn run_field_size_sweep(
    output_dir: &str,
) -> Result<(), Box<dyn std::error::Error>> {

    let field_sizes = vec![
        "configs/field_configs/vineyard/small.json",
        "configs/field_configs/vineyard/medium.json",
        "configs/field_configs/vineyard/large.json",
        "configs/field_configs/vineyard/xlarge.json",
    ];

    let seeds = 0..5;
    let resolution = 15;
    let output_dir_sweep = create_results_subdir(output_dir, "raw/field_sweep")?;

    println!("\n===== EXPERIMENT: Field size sweep ===================================");

    for path in field_sizes {

        for seed in seeds.clone() {

            let exp = ExperimentConfig {
                seed,
                field_config_path: path.to_string(),
                // Pinned explicitly: this sweep is the CC-CV/physics/Leo-Rover
                // story, independent of whatever ExperimentConfig::default()
                // currently is.
                battery_capacity_wh: 73.2,
                battery_voltage_v: 10.8,
                ..Default::default()
            };

            print_experiment_info(&exp);

            let filename=  format!(
                "size={}_fleet={}_batt={}_soc={}_seed={}",
                exp.field_size_label(),
                exp.n_agents,
                exp.battery_capacity_wh,
                exp.soc_threshold_percent,
                seed,
            );

            let timestamp = chrono::Utc::now()
                .format("%H%M%S")
                .to_string();

            let info = ExperimentInfo {
                experiment_type: ExperimentType::FieldSweep,
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