use crate::experiment::runner::run_single_station_experiment;
use crate::experiment::output::create_results_subdir;
use crate::experiment::sweeps::sweep_utils::print_experiment_info;
use crate::experiment::models::{ExperimentType, ExperimentInfo};
use crate::experiment::profile::ExperimentProfile;
use crate::ExperimentConfig;

/// Vineyard-only for now: field size is a meaningful sweep axis for the
/// vineyard small/medium/large/xlarge fields, but field1-4 in the legacy
/// profile are four distinct fields rather than a size progression, so
/// what a "legacy field sweep" should even mean is still undecided.
pub fn run_field_size_sweep(
    output_dir: &str,
) -> Result<(), Box<dyn std::error::Error>> {

    let profile = ExperimentProfile::Vineyard;
    let field_sizes = profile.field_configs();

    let seeds = 0..5;
    let resolution = 15;
    let output_dir_sweep = create_results_subdir(
        output_dir,
        &format!("raw/{}/field_sweep", profile.label()),
    )?;

    println!("\n===== EXPERIMENT: Field size sweep ({}) ======================", profile.label());

    for path in field_sizes {

        for seed in seeds.clone() {

            let exp = ExperimentConfig {
                seed,
                field_config_path: path.to_string(),
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