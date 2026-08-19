#![deny(clippy::unwrap_used)]
#![deny(clippy::expect_used)]
#![deny(clippy::clone_on_copy)]
#![deny(clippy::redundant_clone)]
#![deny(clippy::manual_map)]
#![deny(clippy::manual_filter)]
#![deny(clippy::borrowed_box)]
#![deny(clippy::dbg_macro)]
#![deny(clippy::vec_init_then_push)]
#![deny(clippy::cast_lossless)]

//use std::process::Command;

pub mod app_module;
pub mod tool_module;
pub mod rendering;
pub mod agent_module;
pub mod battery_module;
pub mod movement_module;
pub mod environment;
pub mod path_finding_module;
pub mod task_module;
pub mod utilities;
pub mod units;
pub mod cfg;
pub mod logger;
pub mod experiment;
pub mod optimization;
pub mod terrain;

use crate::app_module::app::App;
use crate::experiment::visualization::run_viz_script;
use crate::experiment::runner::{
    run_ego_experiment,
    run_grid_search_experiment,
    run_single_station_experiment,
    run_multi_station_experiment,
    run_single_evaluation,
    run_soc_threshold_experiment,
};
use crate::experiment::config::ExperimentConfig;
use crate::experiment::profile::ExperimentProfile;
use crate::experiment::models::{ExperimentInfo, ExperimentType};
use crate::experiment::output::create_results_subdir;

//use egui::Pos2;
use env_logger::Env;

pub fn init_logging() {
    let _ = env_logger::Builder::from_env(
        Env::default().default_filter_or("error")
    )
    .is_test(false)
    .try_init();
}

fn get_arg_value(args: &[String], flag: &str) -> Option<String> {
    args.iter()
        .position(|x| x == flag)
        .and_then(|pos| args.get(pos + 1))
        .cloned()
}

fn create_output_directory() -> Result<String, Box<dyn std::error::Error>> {
    let now = chrono::Local::now();

    let base_output = format!("results/{}", now.format("%Y-%m-%d"));

    std::fs::create_dir_all(&base_output)?;

    Ok(base_output)
}

fn get_grid_resolution(args: &[String], default: usize) -> usize {
    get_arg_value(args, "--grid-search")
        .and_then(|v| v.parse::<usize>().ok())
        .unwrap_or(default)
}

fn get_station_number(args: &[String]) -> usize {
    get_arg_value(args, "--optimize-ego")
        .and_then(|v| v.parse::<usize>().ok())
        .unwrap_or(1)
}

fn get_soc_iterations(args: &[String]) -> usize {
    get_arg_value(args, "--optimize-soc")
        .and_then(|v| v.parse::<usize>().ok())
        .unwrap_or(20)
}

/// Station-placement EGO budget, shared by --optimize-ego, --single-station-study,
/// and --multi-station-study so a calibration run can override both without
/// touching each mode's own flag.
fn get_ego_initial(args: &[String]) -> usize {
    get_arg_value(args, "--ego-initial")
        .and_then(|v| v.parse::<usize>().ok())
        .unwrap_or(optimization::constants::DEFAULT_EGO_INITIAL_SAMPLES)
}

fn get_ego_iterations(args: &[String]) -> usize {
    get_arg_value(args, "--ego-iterations")
        .and_then(|v| v.parse::<usize>().ok())
        .unwrap_or(optimization::constants::DEFAULT_EGO_MAX_ITERATIONS)
}

/// Fleet-size override, applied on top of `ExperimentConfig::for_profile`'s
/// default of 1 agent. Lets calibration/sweeps be re-run at a specific fleet
/// size from the CLI without a code change.
fn get_n_agents(args: &[String], default: u32) -> u32 {
    get_arg_value(args, "--n-agents")
        .and_then(|v| v.parse::<u32>().ok())
        .unwrap_or(default)
}

/// Lets --grid-search target a specific field config (e.g. a non-default
/// vineyard field size) from the CLI without a code change.
fn get_field_config_override(args: &[String]) -> Option<String> {
    get_arg_value(args, "--field-config")
}


fn run_gui() -> Result<(), eframe::Error> {
    let options = eframe::NativeOptions {
        vsync: true,

        viewport: egui::ViewportBuilder::default()
            .with_decorations(true)
            .with_maximized(true)
            .with_resizable(true),

        ..Default::default()
    };

    eframe::run_native(
        "farmbotsim-rs",
        options,
        Box::new(|_cc| Ok(Box::new(App::default()))),
    )
}

fn run_debug_tests() {
    // Terrain
    let terrain = crate::terrain::loader::TerrainLoader::from_gps_csv(
        "configs/scene_configs/vineyard_scene/baggy-altitude-empirical-lut.csv"
    );

    terrain.print_points();

    // Slip model
    let slip_model = crate::terrain::slip::SlipModel::from_json_file(
        "configs/scene_configs/vineyard_scene/baggy-slip-linear.json"
    );

    let slope = 0.15;
    let wheel_speed = 0.5;

    let slip = slip_model.compute_slip(slope, wheel_speed);
    let robot_speed = slip_model.compute_robot_speed(wheel_speed, slope);

    println!("slip = {slip}, robot_speed = {robot_speed}");

    // LUT (consumption)
    let lut = crate::battery_module::discharging::VoltageDropLUT::from_csv(
        "configs/movement_configs/consumption/fitted_lut.csv"
    );

    let voltage_drop_per_m = lut.get(slope, robot_speed);

    println!("voltage_drop_per_m = {voltage_drop_per_m}");
}

fn main() ->  Result<(), Box<dyn std::error::Error>> {
    init_logging();

    let args: Vec<String> = std::env::args().collect();

    let profile = ExperimentProfile::from_flag(
        get_arg_value(&args, "--profile").as_deref()
    );

    let base_output = create_output_directory()?;

    let run_viz = args.contains(&"--viz".to_string());

    let timestamp = chrono::Utc::now()
                .format("%H%M%S")
                .to_string();

    // ---------- Debug tests ----------------
    if args.contains(&"--debug-test".to_string()) {
        run_debug_tests();
        return Ok(());
    }   

    // ---------- Single evaluation ----------------
    if args.contains(&"--single-evaluation".to_string()) {

        let mut exp = ExperimentConfig::for_profile(profile);
        exp.n_agents = get_n_agents(&args, exp.n_agents);
        let output_dir_exp = create_results_subdir(
            &base_output,
            &format!("raw/{}/single_evaluation_exp", profile.label()),
        )?;
        let filename=  format!(
            "size={}_fleet={}_batt={:.0}_soc={}",
            exp.field_size_label(),
            exp.n_agents,
            exp.battery_capacity_wh,
            exp.soc_threshold_percent,
        );
        let info = ExperimentInfo {
                experiment_type: ExperimentType::SingleRun,
                timestamp,
            };

        let station_position = exp.load_scene_config().station_configs[0].pose.position;

        run_single_evaluation(
            station_position,
            &filename,
            &output_dir_exp,
            exp,
            info
        );

        return Ok(());
    }

    // ---------- Optimization (EGO) ---------------
    if args.contains(&"--optimize-ego".to_string()) {

        let n_stations = get_station_number(&args);
        let mut exp = ExperimentConfig::for_profile(profile);
        exp.n_agents = get_n_agents(&args, exp.n_agents);
        let output_dir_exp = create_results_subdir(
            &base_output,
            &format!("raw/{}/ego_exp", profile.label()),
        )?;
        let filename=  format!(
            "size={}_fleet={}_batt={:.0}_soc={}",
            exp.field_size_label(),
            exp.n_agents,
            exp.battery_capacity_wh,
            exp.soc_threshold_percent,
        );
        let info = ExperimentInfo {
                experiment_type: ExperimentType::EGO,
                timestamp: timestamp.clone(),
            };

        let run = run_ego_experiment(
            n_stations,
            get_ego_initial(&args),
            get_ego_iterations(&args),
            &filename,
            &output_dir_exp,
            exp,
            info
        );

        if run_viz {
            let figures_dir_exp = create_results_subdir(
                &base_output,
                &format!("figures/{}/ego_exp", profile.label()),
            )?;
            let run_stem = format!("{timestamp}_{filename}");
            run_viz_script(
                "analysis/plotting/ego_viz.py",
                &run.results_path,
                &figures_dir_exp,
                &run_stem,
            );
        }

        return Ok(());
    }

    // ---------- Grid search experiment -----------
    if args.contains(&"--grid-search".to_string()) {

        let mut exp = ExperimentConfig::for_profile(profile);
        exp.n_agents = get_n_agents(&args, exp.n_agents);
        if let Some(field_config) = get_field_config_override(&args) {
            exp.field_config_path = field_config;
        }
        let resolution = get_grid_resolution(&args, exp.field_resolution);
        let output_dir_exp = create_results_subdir(
            &base_output,
            &format!("raw/{}/grid_search_exp", profile.label()),
        )?;
        let filename=  format!(
            "size={}_fleet={}_batt={:.0}_soc={}",
            exp.field_size_label(),
            exp.n_agents,
            exp.battery_capacity_wh,
            exp.soc_threshold_percent,
        );
        let info = ExperimentInfo {
                experiment_type: ExperimentType::GridSearch,
                timestamp: timestamp.clone(),
            };

        let run = run_grid_search_experiment(
            resolution,
            &filename,
            &output_dir_exp,
            exp,
            info,
            );

        if run_viz {
            let figures_dir_exp = create_results_subdir(
                &base_output,
                &format!("figures/{}/grid_search_exp", profile.label()),
            )?;
            let run_stem = format!("{timestamp}_{filename}");
            run_viz_script(
                "analysis/plotting/single_station_viz.py",
                &run.results_path,
                &figures_dir_exp,
                &run_stem,
            );
        }

        return Ok(());
    }

    // --- Single-station study (EGO + grid search) ---
    if args.contains(&"--single-station-study".to_string()) {

        let mut exp = ExperimentConfig::for_profile(profile);
        exp.n_agents = get_n_agents(&args, exp.n_agents);
        let resolution = get_grid_resolution(&args, exp.field_resolution);
        let output_dir_exp = create_results_subdir(
            &base_output,
            &format!("raw/{}/single_station_exp", profile.label()),
        )?;
        let filename=  format!(
            "size={}_fleet={}_batt={:.0}_soc={}",
            exp.field_size_label(),
            exp.n_agents,
            exp.battery_capacity_wh,
            exp.soc_threshold_percent,
        );
        let info = ExperimentInfo {
                experiment_type: ExperimentType::SingleStation,
                timestamp: timestamp.clone(),
            };

        let (run, _) = run_single_station_experiment(
            resolution,
            get_ego_initial(&args),
            get_ego_iterations(&args),
            &filename,
            &output_dir_exp,
            exp,
            info
        );

        if run_viz {
            let figures_dir_exp = create_results_subdir(
                &base_output,
                &format!("figures/{}/single_station_exp", profile.label()),
            )?;
            let run_stem = format!("{timestamp}_{filename}");
            run_viz_script(
                "analysis/plotting/single_station_viz.py",
                &run.results_path,
                &figures_dir_exp,
                &run_stem,
            );
        }

        return Ok(());
    }

    // --- Multi(2)-station study (EGO + specialist layouts) -----
    if args.contains(&"--multi-station-study".to_string()) {

        let mut exp = ExperimentConfig::for_profile(profile);
        exp.n_agents = get_n_agents(&args, exp.n_agents);
        if profile == ExperimentProfile::Legacy {
            // field1 (the profile's own default) is a single uninterrupted
            // row block, so the specialist layouts' split/tight-center
            // heuristics (station_layouts.rs) have no real sub-field
            // structure to land in and can fall arbitrarily between rows.
            // field4 is two row-blocks with a real gap, matching what
            // baseline_comparison.rs already uses for the same reason.
            exp.field_config_path = "configs/field_configs/legacy/field4.json".to_string();
        }
        let output_dir_exp = create_results_subdir(
            &base_output,
            &format!("raw/{}/multi_station_exp", profile.label()),
        )?;
        let filename=  format!(
            "size={}_fleet={}_batt={:.0}_soc={}",
            exp.field_size_label(),
            exp.n_agents,
            exp.battery_capacity_wh,
            exp.soc_threshold_percent,
        );
        let info = ExperimentInfo {
                experiment_type: ExperimentType::MultiStation,
                timestamp: timestamp.clone(),
            };

        let run = run_multi_station_experiment(
            get_ego_initial(&args),
            get_ego_iterations(&args),
            &filename,
            &output_dir_exp,
            exp,
            info);

        if run_viz {
            let figures_dir_exp = create_results_subdir(
                &base_output,
                &format!("figures/{}/multi_station_exp", profile.label()),
            )?;
            let run_stem = format!("{timestamp}_{filename}");
            run_viz_script(
                "analysis/plotting/multi_station_viz.py",
                &run.results_path,
                &figures_dir_exp,
                &run_stem,
            );
        }

        return Ok(());
    }

    // --- SoC-threshold optimization (Level II) ------------------
    if args.contains(&"--optimize-soc".to_string()) {

        let max_iterations = get_soc_iterations(&args);
        let mut exp = ExperimentConfig::for_profile(profile);
        exp.n_agents = get_n_agents(&args, exp.n_agents);
        let output_dir_exp = create_results_subdir(
            &base_output,
            &format!("raw/{}/soc_optimization_exp", profile.label()),
        )?;
        let filename = format!(
            "size={}_fleet={}_batt={:.0}",
            exp.field_size_label(),
            exp.n_agents,
            exp.battery_capacity_wh,
        );
        let info = ExperimentInfo {
                experiment_type: ExperimentType::SocOptimization,
                timestamp: timestamp.clone(),
            };

        run_soc_threshold_experiment(
            max_iterations,
            &filename,
            &output_dir_exp,
            exp,
            info
        );

        return Ok(());
    }

    // --- Sweeps ------------------------------------------------

    if args.contains(&"--battery-sweep".to_string()) {
        experiment::sweeps::battery::run_battery_sweep(profile, &base_output)?;
        return Ok(())
    }

    if args.contains(&"--fleet-sweep".to_string()) {
        experiment::sweeps::fleet::run_fleet_sweep(profile, &base_output)?;
        return Ok(())
    }

    if args.contains(&"--fleet-slots-sweep".to_string()) {
        experiment::sweeps::fleet_slots::run_fleet_slots_sweep(profile, &base_output)?;
        return Ok(())
    }

    if args.contains(&"--field-sweep".to_string()) {
        // Vineyard-only: see run_field_size_sweep's doc comment.
        experiment::sweeps::field::run_field_size_sweep(&base_output)?;
        return Ok(())
    }

    if args.contains(&"--soc-sweep".to_string()) {
        experiment::sweeps::soc::run_soc_sweep(profile, &base_output)?;
        return Ok(())
    }

    if args.contains(&"--initial-soc-sweep".to_string()) {
        experiment::sweeps::initial_soc::run_initial_soc_sweep(profile, &base_output)?;
        return Ok(())
    }

    if args.contains(&"--baseline-comparison-sweep".to_string()) {
        experiment::sweeps::baseline_comparison::run_baseline_comparison_sweep(profile, &base_output)?;
        return Ok(())
    }

    if args.contains(&"--task-duration-jitter-sweep".to_string()) {
        experiment::sweeps::task_duration_jitter::run_task_duration_jitter_sweep(profile, &base_output)?;
        return Ok(())
    }

    run_gui()
        .map_err(|e| format!("GUI failed: {e}").into())

}