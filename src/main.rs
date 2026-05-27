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
use crate::experiment::{
    single_evaluation::run_single_evaluation,
    //grid_search::run_grid_search_experiment,
    visualization::run_viz_script
};
use crate::experiment::runner::{
    run_ego_experiment,
    run_grid_search_experiment,
    run_single_station_experiment,
    run_multi_station_experiment
};
// use rate::optimization::ego::optimize_station_positions_ego;

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

fn create_output_directory() -> Result<String, eframe::Error> {
    let now = chrono::Local::now();

    let base_output =
        format!("results/{}", now.format("%Y-%m-%d"));

    std::fs::create_dir_all(&base_output)
        .map_err(|e| {
            eprintln!("Failed to create output directory: {e}");

            eframe::Error::AppCreation(Box::new(e))
        })?;

    Ok(base_output)
}

fn get_grid_resolution(args: &[String]) -> usize {
    get_arg_value(args, "--grid-search")
        .and_then(|v| v.parse::<usize>().ok())
        .unwrap_or(5)
}

fn get_station_number(args: &[String]) -> usize {
    get_arg_value(args, "--optimize-ego")
        .and_then(|v| v.parse::<usize>().ok())
        .unwrap_or(1)
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

fn main() -> Result<(), eframe::Error> {
    init_logging();

    let args: Vec<String> = std::env::args().collect();

    let base_output = create_output_directory()?;

    let run_viz = args.contains(&"--viz".to_string());

    // ---------- Debug tests ----------------
    if args.contains(&"--debug-test".to_string()) {
        run_debug_tests();
        return Ok(());
    }   

    // ---------- Single evaluation ----------------
    if args.contains(&"--single-evaluation".to_string()) {
        run_single_evaluation();
        return Ok(());
    }

    // ---------- Optimization (EGO) ---------------
    if args.contains(&"--optimize-ego".to_string()) {

        let n_stations = get_station_number(&args);

        let run = run_ego_experiment(
            n_stations, 
            10, 
            &base_output);

        if run_viz {
            run_viz_script(
                "viz/optimization_viz.py",
                &run.results_path,
                &base_output,
            );
        }

        return Ok(());
    }

    // ---------- Grid search experiment -----------
    if args.contains(&"--grid-search".to_string()) {

        let resolution = get_grid_resolution(&args);

        let run = run_grid_search_experiment(
            resolution, 
            &base_output);

        if run_viz {
            run_viz_script(
                "viz/single_station_viz.py",
                &run.results_path,
                &base_output,
            );
        }

        return Ok(());
    }

    // --- Single-station study (EGO + grid search) ---
    if args.contains(&"--single-station-study".to_string()) {

        let resolution = get_grid_resolution(&args);

        let run = run_single_station_experiment(
            resolution,
            &base_output,
        );

        if run_viz {
            run_viz_script(
                "viz/single_station_viz.py",
                &run.results_path,
                &base_output,
            );
        }

        return Ok(());
    }

    // --- Multi(2)-station study (EGO + specialist layouts) -----
    if args.contains(&"--multi-station-study".to_string()) {

        let run = run_multi_station_experiment(
            10, 
            &base_output);

        if run_viz {
            run_viz_script(
                "viz/multi_station_viz.py",
                &run.results_path,
                &base_output,
            );
        }

        return Ok(());
    }

    run_gui()
}