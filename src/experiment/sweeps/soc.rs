use crate::experiment::runner::{
    run_single_station_experiment,
    run_single_evaluation,
};
use crate::experiment::output::create_results_subdir;
use crate::experiment::sweeps::sweep_utils::print_experiment_info;
use crate::experiment::models::{ExperimentType, ExperimentInfo};
use crate::experiment::profile::ExperimentProfile;
use crate::ExperimentConfig;

pub fn run_soc_sweep(
    profile: ExperimentProfile,
    output_dir: &str
)-> Result<(), Box<dyn std::error::Error>> {

    let thresholds = vec![50.0, 55.0, 60.0, 65.0, 70.0, 75.0, 80.0];
    let seeds = 0..15;
    let resolution = 15;
    let output_dir_sweep = create_results_subdir(
        output_dir,
        &format!("raw/{}/soc_sweep", profile.label()),
    )?;

    println!("\n===== EXPERIMENT: SoC threshold sweep ({}) ===================", profile.label());

    // ---------------------------------------------------------
    // Find the best station position once (grid search + EGO),
    // then run the sensitivity sweep around that fixed position.
    // ---------------------------------------------------------

    let baseline_exp = ExperimentConfig {
        n_agents: 3,
        ..ExperimentConfig::for_profile(profile)
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
        experiment_type: ExperimentType::SocSweep,
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

    for threshold in thresholds {

        for seed in seeds.clone() {
            let exp = ExperimentConfig {
                seed,
                n_agents: 3,
                soc_threshold_percent: threshold as f32,
                ..ExperimentConfig::for_profile(profile)
            };

            print_experiment_info(&exp);

            let filename=  format!(
                "size={}_fleet={}_batt={}_soc={}_seed={}",
                exp.field_size_label(),
                exp.n_agents,
                exp.battery_capacity_wh,
                threshold,
                seed,
            );

            let timestamp = chrono::Utc::now()
                .format("%H%M%S")
                .to_string();

            let info = ExperimentInfo {
                experiment_type: ExperimentType::SocSweep,
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