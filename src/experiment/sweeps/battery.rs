use crate::experiment::runner::{
    run_single_station_experiment,
    run_single_evaluation,
};
use crate::experiment::output::create_results_subdir;
use crate::experiment::sweeps::sweep_utils::{
    print_experiment_info, INITIAL_SOC_RANGE, TASK_DURATION_JITTER_RANGE,
};
use crate::experiment::models::{ExperimentType, ExperimentInfo};
use crate::experiment::config::ExperimentConfig;
use crate::experiment::profile::ExperimentProfile;

/// Runs the battery-capacity sweep twice per capacity/seed -- see
/// `run_soc_sweep`'s doc comment (`soc.rs`) for what the `"spawn_only"` vs
/// `"full_noise"` condition means and why; this is the same robustness check
/// applied to the battery-capacity axis instead of SoC threshold.
pub fn run_battery_sweep(
    profile: ExperimentProfile,
    output_dir: &str
) -> Result<(), Box<dyn std::error::Error>> {

    let capacities: Vec<f32> = match profile {
        ExperimentProfile::Legacy => vec![400.0, 420.0, 423.0, 440.0, 460.0],
        ExperimentProfile::Vineyard => vec![65.0, 66.25, 67.5, 68.75, 70.0, 71.25, 72.5, 73.2, 73.75, 75.0, 76.25, 77.5, 78.75, 80.0],
    };
    let seeds = 0..15;
    let resolution = 50;
    let output_dir_sweep = create_results_subdir(
        output_dir,
        &format!("raw/{}/battery_sweep", profile.label()),
    )?;


    println!("\n===== EXPERIMENT: Battery capacity sweep ({}) ================", profile.label());

    // ---------------------------------------------------------
    // Find the best station position once (grid search + EGO),
    // then run the sensitivity sweep around that fixed position.
    // ---------------------------------------------------------

    let baseline_exp = ExperimentConfig::for_profile(profile);
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
        crate::optimization::constants::DEFAULT_EGO_INITIAL_SAMPLES,
        crate::optimization::constants::DEFAULT_EGO_MAX_ITERATIONS,
        &filename,
        &output_dir_sweep,
        baseline_exp,
        info,
    );

    let conditions: [(&str, Option<((f32, f32), (f32, f32))>); 2] = [
        ("spawn_only", None),
        ("full_noise", Some((INITIAL_SOC_RANGE, TASK_DURATION_JITTER_RANGE))),
    ];

    for battery_capacity_wh in capacities {

        for (condition, ranges) in conditions {

            for seed in seeds.clone() {

                let exp = ExperimentConfig {
                    seed,
                    battery_capacity_wh: battery_capacity_wh as f32,
                    initial_soc_min_percent: ranges.map(|(soc, _)| soc.0),
                    initial_soc_max_percent: ranges.map(|(soc, _)| soc.1),
                    task_duration_jitter_min_factor: ranges.map(|(_, jitter)| jitter.0),
                    task_duration_jitter_max_factor: ranges.map(|(_, jitter)| jitter.1),
                    ..ExperimentConfig::for_profile(profile)
                };

                print_experiment_info(&exp);

                let filename=  format!(
                    "size={}_fleet={}_batt={}_soc={}_cond={}_seed={}",
                    exp.field_size_label(),
                    exp.n_agents,
                    battery_capacity_wh,
                    exp.soc_threshold_percent,
                    condition,
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
    }

    Ok(())
}