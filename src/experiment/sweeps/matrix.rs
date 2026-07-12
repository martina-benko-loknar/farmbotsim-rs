use crate::experiment::runner::run_single_station_experiment;
use crate::experiment::output::create_results_subdir;
use crate::experiment::sweeps::sweep_utils::print_experiment_info;
use crate::experiment::models::{ExperimentType, ExperimentInfo};
use crate::experiment::config::ExperimentConfig;

pub fn run_combined_sweep(
    output_dir: &str
) -> Result<(), Box<dyn std::error::Error>> {

    println!("\n===== EXPERIMENT: Combined sweep ===================================");

    let fleet_sizes = vec![1, 2, 4, 8];
    let battery_sizes = vec![350.0, 400.0, 450.0, 500.0];
    let soc_thresholds = vec![50.0, 60.0, 70.0, 80.0];
    let field_sizes = vec![
        "configs/field_configs/vineyard/small.json",
        "configs/field_configs/vineyard/medium.json",
        "configs/field_configs/vineyard/large.json",
        "configs/field_configs/vineyard/xlarge.json",
    ];
    let resolution = 15;
    let seeds = 0..5;
    let output_dir_sweep = create_results_subdir(output_dir, "raw/combined_sweep")?;


    for path in field_sizes {

        for n_agents in fleet_sizes.clone() {

            for battery in battery_sizes.clone() {

                for soc in soc_thresholds.clone() {

                    for seed in seeds.clone() {

                        let exp = ExperimentConfig {
                            seed,
                            battery_capacity_wh: battery as f32,
                            battery_voltage_v: Default::default(),
                            n_agents,
                            critical_soc_percent: Default::default(),
                            soc_threshold_percent: soc,
                            field_resolution: Default::default(),
                            field_config_path: path.to_string(),
                        };

                        print_experiment_info(&exp);

                        let filename=  format!(
                            "size={}_fleet={}_batt={}_soc={}",
                            exp.field_size_label(),
                            exp.n_agents,
                            exp.battery_capacity_wh,
                            exp.soc_threshold_percent,
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
                            info
                        );

                        println!("======================================================================\n");
                    }
                }
            }
        }
    }

    Ok(())
}