use crate::experiment::runner::run_ego_experiment;
use crate::experiment::output::create_results_subdir;
use crate::experiment::sweeps::sweep_utils::print_experiment_info;
use crate::experiment::models::{ExperimentType, ExperimentInfo};
use crate::experiment::config::ExperimentConfig;
use crate::experiment::profile::ExperimentProfile;

/// Fleet sizes beyond the original 1..=4 scaling range, added 2026-09-03 to
/// extend the fleet-slots queueing-contention comparison
/// (fig:fleet-slots-comparison) further out. Kept separate from this
/// module's own `fleet_sizes` base range rather than folded into
/// `fleet::run_fleet_sweep`'s own list -- that sweep backs the *published*
/// fleet-size-scaling figure (fig:fleet-sweep-scaling, comparison_fleet.py/
/// convergence_fleet.py), which wasn't asked to grow too, and widening its
/// own list would have silently done that. `comparison_fleet.py` also
/// defensively filters to the original range for the same reason, in case
/// this ever changes.
const EXTRA_FLEET_SIZES: [u32; 4] = [8, 10, 12, 14];

/// Same fleet-size sweep as `fleet::run_fleet_sweep`, but with the single
/// station's charging slots matched 1:1 to fleet size (n_station_slots =
/// n_agents) instead of fixed at 1. Diagonal-only: doesn't cover the full
/// slots x fleet_size matrix, just "1 slot" (fleet_sweep) vs "matched
/// slots" (this) at each fleet size -- enough to tell whether fleet-size
/// energy overhead (see comparison_fleet.py) is queueing contention on the
/// shared slot rather than a travel-distance effect, since matched slots
/// makes queueing impossible.
///
/// Runs the matched-slots condition at 1, 2, 3, 4, 8, 10, 12, 14 agents
/// (`EXTRA_FLEET_SIZES` appended to the original range) and additionally
/// backfills the missing "1 slot" baseline at the four new sizes into
/// `fleet::run_fleet_sweep`'s own `fleet_sweep` raw directory -- 1..=4
/// already has that baseline from `--fleet-sweep`, so only the new sizes
/// are run here. `comparison_fleet_slots.py`'s existing merge on
/// `fleet_size` then picks up all eight sizes from both directories
/// automatically, no script change needed.
pub fn run_fleet_slots_sweep(
    profile: ExperimentProfile,
    output_dir: &str
) -> Result<(), Box<dyn std::error::Error>> {

    let fleet_sizes: Vec<u32> = [1, 2, 3, 4].into_iter().chain(EXTRA_FLEET_SIZES).collect();
    let seeds = 0..15;
    let output_dir_sweep = create_results_subdir(
        output_dir,
        &format!("raw/{}/fleet_slots_sweep", profile.label()),
    )?;

    println!("\n===== EXPERIMENT: Fleet size sweep, slots matched to fleet size ({}) ======================", profile.label());

    for n_agents in fleet_sizes {

        for seed in seeds.clone() {

            let exp = ExperimentConfig {
                seed,
                n_agents,
                n_station_slots: n_agents,
                ..ExperimentConfig::for_profile(profile)
            };

            print_experiment_info(&exp);

            let filename=  format!(
                "size={}_fleet={}_slots={}_batt={}_soc={}_seed={}",
                exp.field_size_label(),
                exp.n_agents,
                exp.n_station_slots,
                exp.battery_capacity_wh,
                exp.soc_threshold_percent,
                seed,
            );

            let timestamp = chrono::Utc::now()
                .format("%H%M%S")
                .to_string();

            let info = ExperimentInfo {
                experiment_type: ExperimentType::FleetSlotsSweep,
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

    // ---------------------------------------------------------
    // Backfill the "1 slot" baseline at the new sizes only, into the same
    // `fleet_sweep` directory `fleet::run_fleet_sweep` writes -- mirrors
    // that function's own loop body exactly (naming included) so the two
    // directories stay indistinguishable to downstream loaders/scripts.
    // ---------------------------------------------------------
    let one_slot_output_dir = create_results_subdir(
        output_dir,
        &format!("raw/{}/fleet_sweep", profile.label()),
    )?;

    for n_agents in EXTRA_FLEET_SIZES {

        for seed in seeds.clone() {

            let exp = ExperimentConfig {
                seed,
                n_agents,
                ..ExperimentConfig::for_profile(profile)
            };

            print_experiment_info(&exp);

            let filename = format!(
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
                &one_slot_output_dir,
                exp,
                info
            );

            println!("======================================================================\n");

        }
    }

    Ok(())
}
