use egui::Pos2;

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
    /// `None`: run `n_ego_restarts` fresh, independent EGO optimizations for
    /// this combo and keep the best (by BO's own reported objective value)
    /// -- the mitigation the Discussion section itself recommends for the
    /// documented bimodal-basin problem (a single BO run can land ~14%
    /// worse purely from optimizer-seed luck, and this does not improve
    /// with a larger budget: even 110 evaluations still landed in the bad
    /// basin 2 of 3 times in the calibration study). `Some(coords)`: skip
    /// optimization entirely and evaluate this fixed layout as `ego_best`
    /// instead -- used by `run_baseline_comparison_multiagent_sweep` to
    /// reuse the exact layout already reported in the paper's multi-agent
    /// table (Section 4.4), so only the evaluation seed varies relative to
    /// that single run.
    fixed_ego_layout: Option<Vec<Pos2>>,
    /// Number of independent EGO restarts when `fixed_ego_layout` is
    /// `None`; ignored otherwise. 1 reproduces the old single-shot
    /// behavior (kept for `Legacy`, out of scope for this restart fix).
    n_ego_restarts: usize,
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
/// - `vineyard`: one-at-a-time sensitivity around the demoed multi-agent
///   config (XL field, 73.2 Wh, fleet 4 -- the anchor point, shared with
///   `run_baseline_comparison_multiagent_sweep`): field size swept at
///   fleet 4/73.2 Wh, fleet size swept at XL/73.2 Wh, battery capacity
///   swept at XL/fleet 4 -- one axis varied at a time rather than a full
///   field x fleet x battery factorial (48 combos), which we tried first
///   and abandoned: at 1 EGO run per combo it was dominated by single-run
///   BO-basin noise (e.g.\ neighboring fleet sizes at the same field/
///   battery swinging from +30% to -15%), not real configuration-
///   dependence, exactly the bimodal-basin problem the Discussion section
///   documents. Every combo here instead uses `n_ego_restarts = 3`
///   (best-of-3, matching the Discussion's own recommended mitigation) to
///   suppress that noise.
pub fn run_baseline_comparison_sweep(
    profile: ExperimentProfile,
    output_dir: &str,
) -> Result<(), Box<dyn std::error::Error>> {

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
                fixed_ego_layout: None,
                n_ego_restarts: 1,
            },
        ],
        ExperimentProfile::Vineyard => {
            let anchor_field = profile.default_field_config().to_string(); // xlarge
            let anchor_battery = profile.battery_capacity_wh(); // 73.2
            let anchor_fleet: u32 = 4;

            let mut combos = vec![ComparisonCombo {
                field_config_path: anchor_field.clone(),
                battery_capacity_wh: anchor_battery,
                n_agents: anchor_fleet,
                fixed_ego_layout: None,
                n_ego_restarts: 3,
            }];

            // Field-size axis (fleet, battery held at anchor); XL is the
            // anchor itself, not repeated here.
            for field_config_path in [
                "configs/field_configs/vineyard/small.json",
                "configs/field_configs/vineyard/medium.json",
                "configs/field_configs/vineyard/large.json",
            ] {
                combos.push(ComparisonCombo {
                    field_config_path: field_config_path.to_string(),
                    battery_capacity_wh: anchor_battery,
                    n_agents: anchor_fleet,
                    fixed_ego_layout: None,
                    n_ego_restarts: 3,
                });
            }

            // Fleet-size axis (field, battery held at anchor); 4 is the
            // anchor itself, not repeated here.
            for n_agents in [1, 2, 3] {
                combos.push(ComparisonCombo {
                    field_config_path: anchor_field.clone(),
                    battery_capacity_wh: anchor_battery,
                    n_agents,
                    fixed_ego_layout: None,
                    n_ego_restarts: 3,
                });
            }

            // Battery-capacity axis (field, fleet held at anchor); 73.2 Wh
            // is the anchor itself, not repeated here.
            for battery_capacity_wh in [65.0, 80.0] {
                combos.push(ComparisonCombo {
                    field_config_path: anchor_field.clone(),
                    battery_capacity_wh,
                    n_agents: anchor_fleet,
                    fixed_ego_layout: None,
                    n_ego_restarts: 3,
                });
            }

            combos
        }
    };

    run_comparison_combos(profile, output_dir, "baseline_comparison", combos)
}

/// Seed-averaged (mean +/- std, 15 seeds) companion to the single-run
/// "Multi-agent, multi-station" study (`--multi-station-study`, vineyard
/// profile): the *exact same* combo (XL field, profile's default battery --
/// 73.2 Wh for the Leo Rover -- 4 agents) *and* the exact same EGO layout
/// already reported there ((3.36, 51.96), (4.96, 19.90)), fixed rather than
/// re-optimized, so only the evaluation seed varies relative to that single
/// run -- isolating the seed effect without also reintroducing BO run-to-run
/// variance as a second confound. Written to its own
/// `baseline_comparison_multiagent` results dir so it isn't pooled with the
/// `baseline_comparison` sweep's (fleet 3, S/L, 65/80 Wh) aggregate.
pub fn run_baseline_comparison_multiagent_sweep(
    profile: ExperimentProfile,
    output_dir: &str,
) -> Result<(), Box<dyn std::error::Error>> {

    let combo = ComparisonCombo {
        field_config_path: profile.default_field_config().to_string(),
        battery_capacity_wh: profile.battery_capacity_wh(),
        n_agents: 4,
        fixed_ego_layout: Some(vec![
            Pos2::new(3.36, 51.96),
            Pos2::new(4.96, 19.90),
        ]),
        n_ego_restarts: 1, // unused: fixed_ego_layout takes precedence
    };

    run_comparison_combos(profile, output_dir, "baseline_comparison_multiagent", vec![combo])
}

fn run_comparison_combos(
    profile: ExperimentProfile,
    output_dir: &str,
    sweep_name: &str,
    combos: Vec<ComparisonCombo>,
) -> Result<(), Box<dyn std::error::Error>> {

    let seeds = 0..15;
    let output_dir_sweep = create_results_subdir(
        output_dir,
        &format!("raw/{}/{}", profile.label(), sweep_name),
    )?;

    println!("\n===== EXPERIMENT: Baseline-vs-EGO comparison ({}, {}) ================", profile.label(), sweep_name);

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

        let ego_stations = match &combo.fixed_ego_layout {
            Some(stations) => {
                println!(
                    "\n=========== EGO layout fixed (field={}, battery={} Wh, fleet={}): reusing ({:.2}, {:.2}), ({:.2}, {:.2}) ===========",
                    combo.field_config_path, combo.battery_capacity_wh, combo.n_agents,
                    stations[0].x, stations[0].y, stations[1].x, stations[1].y,
                );
                stations.clone()
            }
            None => {
                println!(
                    "\n=========== EGO optimization, best of {} restart(s) (field={}, battery={} Wh, fleet={}) ===========",
                    combo.n_ego_restarts, combo.field_config_path, combo.battery_capacity_wh, combo.n_agents,
                );
                let mut best: Option<crate::experiment::models::EgoSummary> = None;
                for restart in 0..combo.n_ego_restarts.max(1) {
                    let ego_results = optimize_station_positions_ego(
                        2,
                        DEFAULT_EGO_INITIAL_SAMPLES,
                        DEFAULT_EGO_MAX_ITERATIONS,
                        &baseline_exp,
                    );
                    let energy = ego_results.summary.best_metrics.energy_wh;
                    let p = &ego_results.summary.optimal_position;
                    println!(
                        "  restart {}: ({:.2}, {:.2}), ({:.2}, {:.2}) -> {:.2} Wh{}",
                        restart, p[0].x, p[0].y, p[1].x, p[1].y, energy,
                        if best.as_ref().is_none_or(|b| energy < b.best_metrics.energy_wh) { "  <-- new best" } else { "" },
                    );
                    if best.as_ref().is_none_or(|b| energy < b.best_metrics.energy_wh) {
                        best = Some(ego_results.summary);
                    }
                }
                // Safety: `n_ego_restarts.max(1)` guarantees the loop above
                // ran at least once, so `best` is always populated here.
                #[allow(clippy::unwrap_used)]
                best.unwrap().optimal_position
            }
        };
        layouts.push(StationLayout {
            name: "ego_best".to_string(),
            stations: ego_stations,
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
