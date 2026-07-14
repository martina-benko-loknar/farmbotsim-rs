use crate::experiment::runner::{
    run_single_station_experiment,
    run_single_evaluation,
};
use crate::experiment::output::create_results_subdir;
use crate::experiment::sweeps::sweep_utils::print_experiment_info;
use crate::experiment::models::{ExperimentType, ExperimentInfo};
use crate::experiment::config::ExperimentConfig;

pub fn run_battery_sweep(
    output_dir: &str
) -> Result<(), Box<dyn std::error::Error>> {

    let capacities = vec![65.0, 67.5, 70.0, 72.5, 73.2, 75.0, 77.5, 80.0];
    let seeds = 0..15;
    let resolution = 15;
    let output_dir_sweep = create_results_subdir(output_dir, "raw/battery_sweep")?;


    println!("\n===== EXPERIMENT: Battery capacity sweep ===================================");

    // ---------------------------------------------------------
    // Find the best station position once (grid search + EGO),
    // then run the sensitivity sweep around that fixed position.
    // ---------------------------------------------------------

    let baseline_exp = ExperimentConfig {
        // Pinned explicitly: this sweep is the CC-CV/physics/Leo-Rover
        // story, independent of whatever ExperimentConfig::default() is.
        battery_capacity_wh: 73.2,
        battery_voltage_v: 10.8,
        ..Default::default()
    };
    let filename = format!(
        "size={}_fleet={}_batt={}_soc={}",
        baseline_exp.field_size_label(),
        baseline_exp.n_agents,
        baseline_exp.battery_capacity_wh,
        baseline_exp.soc_threshold_percent,
    );

    let timestamp = chrono::Utc::now()
        .format("%H%M%S")
        .to_string();

    let info = ExperimentInfo {
        experiment_type: ExperimentType::BatterySweep,
        timestamp: timestamp.clone(),
    };

    let (_, station_position) = run_single_station_experiment(
        resolution,
        &filename,
        &output_dir_sweep,
        baseline_exp,
        info,
    );

    for battery_capacity_wh in capacities {

        for seed in seeds.clone() {

            let exp = ExperimentConfig {
                seed,
                battery_capacity_wh: battery_capacity_wh as f32,
                // Pinned explicitly: this sweep is the CC-CV/physics/Leo-Rover
                // story, independent of whatever ExperimentConfig::default() is.
                battery_voltage_v: 10.8,
                ..Default::default()
            };

            print_experiment_info(&exp);

            let filename=  format!(
                "size={}_fleet={}_batt={}_soc={}_seed={}",
                exp.field_size_label(),
                exp.n_agents,
                battery_capacity_wh,
                exp.soc_threshold_percent,
                seed,
            );

            let timestamp = chrono::Utc::now()
                .format("%H%M%S")
                .to_string();

            let info = ExperimentInfo {
                experiment_type: ExperimentType::BatterySweep,
                timestamp: timestamp.clone(),
            };

            run_single_evaluation(
                station_position,
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