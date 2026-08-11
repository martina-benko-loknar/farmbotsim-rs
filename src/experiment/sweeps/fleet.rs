use crate::experiment::runner::run_ego_experiment;
use crate::experiment::output::create_results_subdir;
use crate::experiment::sweeps::sweep_utils::print_experiment_info;
use crate::experiment::models::{ExperimentType, ExperimentInfo};
use crate::experiment::config::ExperimentConfig;
use crate::experiment::profile::ExperimentProfile;

pub fn run_fleet_sweep(
    profile: ExperimentProfile,
    output_dir: &str
) -> Result<(), Box<dyn std::error::Error>> {

    let fleet_sizes = vec![1, 2, 3, 4];
    let seeds = 0..15;
    let output_dir_sweep = create_results_subdir(
        output_dir,
        &format!("raw/{}/fleet_sweep", profile.label()),
    )?;

    println!("\n===== EXPERIMENT: Fleet size sweep ({}) ======================", profile.label());

    for n_agents in fleet_sizes {

        for seed in seeds.clone() {

            let exp = ExperimentConfig {
                seed,
                n_agents: n_agents as u32,
                ..ExperimentConfig::for_profile(profile)
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
                experiment_type: ExperimentType::FleetSweep,
                timestamp: timestamp.clone(),
            };

            run_ego_experiment(
                1,
                crate::optimization::constants::DEFAULT_EGO_INITIAL_SAMPLES,
                crate::optimization::constants::DEFAULT_EGO_MAX_ITERATIONS,
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