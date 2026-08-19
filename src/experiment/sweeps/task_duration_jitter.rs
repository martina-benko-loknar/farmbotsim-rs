use crate::experiment::runner::{
    run_single_station_experiment,
    run_single_evaluation,
};
use crate::experiment::output::create_results_subdir;
use crate::experiment::sweeps::sweep_utils::{print_experiment_info, TASK_DURATION_JITTER_RANGE};
use crate::experiment::models::{ExperimentType, ExperimentInfo};
use crate::experiment::profile::ExperimentProfile;
use crate::ExperimentConfig;

/// Sensitivity sweep for the task-duration-jitter randomization axis (see
/// `EnvOverrides::task_duration_jitter_min_factor`/`max_factor` in
/// `env.rs`, and `TaskDurationJitter` in `task_manager.rs`).
///
/// For each (field size, fleet size, seed), runs the *same* setup twice:
/// once with every task at its configured duration (`condition = "fixed"`,
/// i.e. today's default/old behavior), and once with each task instance's
/// duration jittered by an independent uniform factor drawn from
/// `TASK_DURATION_JITTER_RANGE` (`condition = "randomized"`). Comparing the
/// seed-to-seed spread of the two conditions in `analysis/` answers the
/// same shape of question `initial_soc.rs` asks for the SoC axis: does
/// adding task-duration heterogeneity actually move outcome variance beyond
/// what spawn-position (and, if enabled, initial-SoC) randomization already
/// captures?
///
/// Both conditions reuse the same `seed` for spawn position/heading *and*
/// (in the randomized condition) for the task-duration draw -- i.e.
/// `task_duration_jitter_seed` is left unset so it piggybacks off `seed`.
/// That's the right choice for "does more realistic seed-to-seed
/// variability change the results"; it is not an ablation that isolates the
/// task-duration axis from the spawn axis. For that, drive
/// `task_duration_jitter_seed` independently of `seed` instead (see
/// `EnvOverrides::task_duration_jitter_seed`'s doc comment).
///
/// `TASK_DURATION_JITTER_RANGE` itself now lives in `sweep_utils` -- shared
/// with the "full_noise" condition in `soc.rs`/`battery.rs`.

/// Field sizes covered -- the full vineyard size progression, see
/// `ExperimentProfile::field_configs`. Matches `initial_soc.rs`.
const FIELD_SIZES: &[&str] = &[
    "configs/field_configs/vineyard/small.json",
    "configs/field_configs/vineyard/medium.json",
    "configs/field_configs/vineyard/large.json",
    "configs/field_configs/vineyard/xlarge.json",
];

/// Fleet sizes covered, bracketing fleet=3. Matches `initial_soc.rs`.
const FLEET_SIZES: &[u32] = &[1, 3, 5];

pub fn run_task_duration_jitter_sweep(
    profile: ExperimentProfile,
    output_dir: &str,
) -> Result<(), Box<dyn std::error::Error>> {

    let output_dir_sweep = create_results_subdir(
        output_dir,
        &format!("raw/{}/task_duration_jitter_sweep", profile.label()),
    )?;

    println!("\n===== EXPERIMENT: Task duration jitter sweep ({}) ========", profile.label());

    for &field_config_path in FIELD_SIZES {
        for &n_agents in FLEET_SIZES {
            run_task_duration_jitter_sweep_for(
                profile,
                &output_dir_sweep,
                field_config_path,
                n_agents,
            );
        }
    }

    Ok(())
}

/// Runs the fixed-vs-randomized comparison (15 seeds each) for one
/// (field size, fleet size) combination, around a station position found
/// once for that combination.
fn run_task_duration_jitter_sweep_for(
    profile: ExperimentProfile,
    output_dir_sweep: &str,
    field_config_path: &str,
    n_agents: u32,
) {
    let seeds = 0..15;
    let resolution = 50;

    // ---------------------------------------------------------
    // Find the best station position once for this combination,
    // then run both conditions around that fixed position.
    // ---------------------------------------------------------

    let baseline_exp = ExperimentConfig {
        n_agents,
        field_config_path: field_config_path.to_string(),
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
        experiment_type: ExperimentType::TaskDurationJitterSweep,
        timestamp: timestamp.clone(),
    };

    let (_, station_position) = run_single_station_experiment(
        resolution,
        crate::optimization::constants::DEFAULT_EGO_INITIAL_SAMPLES,
        crate::optimization::constants::DEFAULT_EGO_MAX_ITERATIONS,
        &filename,
        output_dir_sweep,
        baseline_exp,
        info,
    );

    let conditions: [(&str, Option<(f32, f32)>); 2] = [
        ("fixed", None),
        ("randomized", Some(TASK_DURATION_JITTER_RANGE)),
    ];

    for (condition, range) in conditions {

        for seed in seeds.clone() {

            let exp = ExperimentConfig {
                seed,
                n_agents,
                field_config_path: field_config_path.to_string(),
                task_duration_jitter_min_factor: range.map(|(min, _)| min),
                task_duration_jitter_max_factor: range.map(|(_, max)| max),
                ..ExperimentConfig::for_profile(profile)
            };

            print_experiment_info(&exp);

            let filename = format!(
                "size={}_fleet={}_batt={}_soc={}_taskjit={}_seed={}",
                exp.field_size_label(),
                exp.n_agents,
                exp.battery_capacity_wh,
                exp.soc_threshold_percent,
                condition,
                seed,
            );

            let timestamp = chrono::Utc::now()
                .format("%H%M%S")
                .to_string();

            let info = ExperimentInfo {
                experiment_type: ExperimentType::TaskDurationJitterSweep,
                timestamp: timestamp.clone(),
            };

            run_single_evaluation(
                station_position,
                &filename,
                output_dir_sweep,
                exp,
                info,
            );

            println!("======================================================================\n");
        }
    }
}
