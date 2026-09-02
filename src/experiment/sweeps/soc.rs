use crate::experiment::runner::{
    run_single_station_experiment,
    run_single_evaluation,
};
use crate::experiment::output::create_results_subdir;
use crate::experiment::sweeps::sweep_utils::{
    print_experiment_info, TASK_DURATION_JITTER_RANGE,
};
use crate::experiment::models::{ExperimentType, ExperimentInfo};
use crate::experiment::profile::ExperimentProfile;
use crate::ExperimentConfig;

/// Initial-SoC range for this sweep's `"full_noise"` condition -- deliberately
/// its own range, NOT `sweep_utils::INITIAL_SOC_RANGE` (which
/// `initial_soc.rs`/`battery_sweep`'s `"full_noise"` use). `INITIAL_SOC_RANGE`
/// is `(50.0, 90.0)`, which fully contains this sweep's own swept range
/// (`thresholds`, 50.0..=80.0) -- every `full_noise` run would then have some
/// chance of spawning already below the very threshold it's testing, and
/// that chance rises with the threshold value (0% at threshold=50%, 75% at
/// threshold=80%; P(initial_soc < t) = (t - 50) / 40 under
/// `INITIAL_SOC_RANGE`). That's a confound, not just noise: it washes out
/// the swept parameter's own effect in a way that gets stronger exactly
/// where the "spawn_only" curve is flattest, which is what produced the
/// suspiciously flat `full_noise` sensitivity curve this range was introduced
/// to fix.
///
/// Pushing the range *below* 50.0 instead doesn't work: `critical_soc_percent`
/// (45.0) + `SOC_THRESHOLD_MARGIN_ABOVE_CRITICAL` (5.0, see
/// `optimization/constants.rs`) = 50.0 is already the project's own floor
/// for a sane SoC-threshold-like value -- there's no room below the sweep's
/// minimum that isn't also critical-battery territory (a different,
/// immediate-charge-override behavior, i.e. a new confound). So instead
/// this sits above the sweep's maximum (80.0), inside the same sane range
/// the sweep itself is drawn from (`SOC_THRESHOLD_MAX_PERCENT` = 90.0):
/// every `full_noise` run starts above every threshold value under test, so
/// P(initial_soc < t) = 0 for every t in `thresholds`, independent of t.
/// Narrower (8 points, vs. `INITIAL_SOC_RANGE`'s 40) because that's all the
/// headroom available between the sweep's ceiling and the project's own
/// sane-SoC ceiling.
const SOC_SWEEP_FULL_NOISE_INITIAL_SOC_RANGE: (f32, f32) = (82.0, 90.0);

/// Runs the SoC-threshold sweep twice per threshold/seed: once with only
/// spawn-position randomization on (`condition = "spawn_only"`, the sweep's
/// original/default behavior), and once with initial-SoC and
/// task-duration-jitter randomization also turned on
/// (`condition = "full_noise"`) -- a robustness check on whether the
/// SoC-threshold sensitivity curve found under "clean" (single-axis) noise
/// still holds once every realistic noise source is active together. See
/// `various/randomness_axes_sweep_design.txt` for why this wasn't just
/// folded into the existing runs, and `initial_soc.rs`/
/// `task_duration_jitter.rs` for the sweeps that isolate those two axes
/// individually.
///
/// `full_noise`'s initial-SoC draw uses `SOC_SWEEP_FULL_NOISE_INITIAL_SOC_RANGE`
/// (see its doc comment for why, not `sweep_utils::INITIAL_SOC_RANGE`);
/// task-duration jitter has no analogous confound with `soc_threshold`
/// (different units, no structural correlation) so it keeps the shared
/// `TASK_DURATION_JITTER_RANGE`.
///
/// `full_noise` reuses `seed` for the SoC/duration draws too (leaves
/// `initial_soc_seed`/`task_duration_jitter_seed` unset) -- same "piggyback,
/// not decoupled" choice `initial_soc.rs`/`task_duration_jitter.rs` make:
/// this answers "does more realistic seed-to-seed variability change the
/// results", not a factorial ablation of which axis contributes what.
pub fn run_soc_sweep(
    profile: ExperimentProfile,
    output_dir: &str
)-> Result<(), Box<dyn std::error::Error>> {

    let thresholds = vec![50.0, 52.5, 55.0, 57.5, 60.0, 62.5, 65.0, 67.5, 70.0, 72.5, 75.0, 77.5, 80.0];
    let seeds = 0..15;
    let resolution = 50;
    let output_dir_sweep = create_results_subdir(
        output_dir,
        &format!("raw/{}/soc_sweep", profile.label()),
    )?;

    println!("\n===== EXPERIMENT: SoC threshold sweep ({}) ===================", profile.label());

    // ---------------------------------------------------------
    // Find the best station position once (grid search + EGO),
    // then run the sensitivity sweep around that fixed position.
    // ---------------------------------------------------------

    // n_agents left at ExperimentConfig::for_profile's default (1) rather
    // than overridden, so this sweep's fixed station position/fleet size
    // actually matches the Level II SoC-threshold optimizer and the
    // battery-capacity sweep it's meant to sit alongside (both also run at
    // the default fleet=1) -- previously hardcoded to 3 here only, which
    // silently broke the "same fixed station position"/cross-validation
    // claims made about this sweep in the paper text. Old fleet=3 data is
    // left in place under the same `soc_sweep` directory (filenames encode
    // fleet size, so there's no collision).
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

    let conditions: [(&str, Option<((f32, f32), (f32, f32))>); 2] = [
        ("spawn_only", None),
        ("full_noise", Some((SOC_SWEEP_FULL_NOISE_INITIAL_SOC_RANGE, TASK_DURATION_JITTER_RANGE))),
    ];

    for threshold in thresholds {

        for (condition, ranges) in conditions {

            for seed in seeds.clone() {
                let exp = ExperimentConfig {
                    seed,
                    soc_threshold_percent: threshold as f32,
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
                    exp.battery_capacity_wh,
                    threshold,
                    condition,
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
    }

    Ok(())
}