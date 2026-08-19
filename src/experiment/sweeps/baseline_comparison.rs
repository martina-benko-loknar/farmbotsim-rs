use crate::experiment::config::ExperimentConfig;
use crate::experiment::evaluation::evaluate_station_layout;
use crate::experiment::export::save_single_evaluation_results;
use crate::experiment::models::{ExperimentInfo, ExperimentType, StationLayout};
use crate::experiment::output::create_results_subdir;
use crate::experiment::profile::ExperimentProfile;
use crate::experiment::station_layouts::specialist_layouts;
use crate::experiment::sweeps::sweep_utils::print_experiment_info;
use crate::optimization::constants::{DEFAULT_EGO_INITIAL_SAMPLES, DEFAULT_EGO_MAX_ITERATIONS};
use crate::optimization::ego::optimize_station_positions_ego;

/// One (field, battery, fleet) scenario to compare naive layouts against EGO on.
struct ComparisonCombo {
    field_config_path: String,
    battery_capacity_wh: f32,
    n_agents: u32,
}

/// Compares EGO's best 2-station layout against the specialist heuristic
/// layouts (including `task_centroid`) at a fixed set of (field, battery,
/// fleet) combos, across multiple seeds, to get mean +/- std per layout --
/// the reviewer-requested "true baseline vs. optimizer" comparison (R1.2)
/// that was previously missing (existing multi-station study only ran each
/// layout once, at seed=0).
///
/// - `legacy`: the single combo that produced the paper's existing Table 9
///   (field1, default battery, 4 agents), so this patches that table with
///   real baselines and variance instead of replacing the scenario.
/// - `vineyard`: field size {small, large} x battery {65, 80} Wh, 3 agents --
///   fills the response letter's already-drafted sensitivity table skeleton.
pub fn run_baseline_comparison_sweep(
    profile: ExperimentProfile,
    output_dir: &str,
) -> Result<(), Box<dyn std::error::Error>> {

    let seeds = 0..15;
    let output_dir_sweep = create_results_subdir(
        output_dir,
        &format!("raw/{}/baseline_comparison", profile.label()),
    )?;

    println!("\n===== EXPERIMENT: Baseline-vs-EGO comparison ({}) ================", profile.label());

    let combos: Vec<ComparisonCombo> = match profile {
        ExperimentProfile::Legacy => vec![
            ComparisonCombo {
                // field4, not `profile.default_field_config()` (field1): field1 is a
                // single uninterrupted row block, so split_center/symmetry heuristics
                // land arbitrarily between rows. field4 is two row-blocks with a real
                // gap between them, so those heuristics correspond to an actual
                // sub-field split.
                field_config_path: "configs/field_configs/legacy/field4.json".to_string(),
                battery_capacity_wh: profile.battery_capacity_wh(),
                n_agents: 4,
            },
        ],
        ExperimentProfile::Vineyard => {
            let mut combos = Vec::new();
            for field_config_path in [
                "configs/field_configs/vineyard/small.json",
                "configs/field_configs/vineyard/large.json",
            ] {
                for battery_capacity_wh in [65.0, 80.0] {
                    combos.push(ComparisonCombo {
                        field_config_path: field_config_path.to_string(),
                        battery_capacity_wh,
                        n_agents: 3,
                    });
                }
            }
            combos
        }
    };

    for combo in combos {

        let baseline_exp = ExperimentConfig {
            field_config_path: combo.field_config_path.clone(),
            battery_capacity_wh: combo.battery_capacity_wh,
            n_agents: combo.n_agents,
            ..ExperimentConfig::for_profile(profile)
        };

        let scene_config = baseline_exp.load_scene_config();
        let field_config = baseline_exp.load_field_config();

        // ---------------------------------------------------------
        // Layouts: the specialist heuristics (now including task_centroid)
        // plus EGO's own best 2-station layout, found once per combo.
        // ---------------------------------------------------------
        let mut layouts = specialist_layouts(&field_config);

        println!(
            "\n=========== EGO optimization (field={}, battery={} Wh, fleet={}) ===========",
            combo.field_config_path, combo.battery_capacity_wh, combo.n_agents,
        );
        let ego_results = optimize_station_positions_ego(
            2,
            DEFAULT_EGO_INITIAL_SAMPLES,
            DEFAULT_EGO_MAX_ITERATIONS,
            &baseline_exp,
        );
        layouts.push(StationLayout {
            name: "ego_best".to_string(),
            stations: ego_results.summary.optimal_position,
        });

        // ---------------------------------------------------------
        // mean +/- std across seeds, for every layout, at this fixed combo
        // ---------------------------------------------------------
        for layout in &layouts {
            for seed in seeds.clone() {

                let exp = ExperimentConfig {
                    seed,
                    ..baseline_exp.clone()
                };

                print_experiment_info(&exp);
                println!("Layout         : {}", layout.name);

                let metrics = evaluate_station_layout(&layout.stations, &scene_config, &exp);

                let filename = format!(
                    "layout={}_size={}_fleet={}_batt={}_soc={}_seed={}",
                    layout.name,
                    exp.field_size_label(),
                    exp.n_agents,
                    combo.battery_capacity_wh,
                    exp.soc_threshold_percent,
                    seed,
                );

                let timestamp = chrono::Utc::now()
                    .format("%H%M%S")
                    .to_string();

                let info = ExperimentInfo {
                    experiment_type: ExperimentType::BaselineComparison,
                    timestamp,
                };

                save_single_evaluation_results(
                    &metrics,
                    &exp,
                    &info,
                    &filename,
                    &output_dir_sweep,
                );

                println!("======================================================================\n");
            }
        }
    }

    Ok(())
}
