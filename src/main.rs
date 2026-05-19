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
// const PRINTOUT_WIDTH: usize = 70;

// fn print_section(title: &str) {
//     println!("{}", "=".repeat(PRINTOUT_WIDTH));
//     println!("{title}");
//     println!("{}", "=".repeat(PRINTOUT_WIDTH));
// }

// fn print_phase(title: &str) {
//     println!("{} {} {}", "=".repeat(6), title, "=".repeat(PRINTOUT_WIDTH.saturating_sub(8 + title.chars().count())));
// }

fn get_arg_value(args: &[String], flag: &str) -> Option<String> {
    args.iter()
        .position(|x| x == flag)
        .and_then(|pos| args.get(pos + 1))
        .cloned()
}

// fn run_viz_script(
//     script: &str,
//     json_path: &str,
//     output_dir: &str,
// ) {
//     let status = Command::new("python")
//         .arg(script)
//         .arg(json_path)
//         .arg(output_dir)
//         .status();

//     match status {
//         Ok(s) if s.success() => {}
//         Ok(_) => {
//             eprintln!("Python script failed (non-zero exit code): {}",script);
//         }
//         Err(e) => {
//             eprintln!("Failed to run Python script {}: {}", script, e);
//         }
//     }
// }

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

// fn parse_optimization_minimum(
//     args: &[String],
// ) -> Option<(Pos2, f64)> {
//     let pos =
//         args.iter().position(|x| x == "--min")?;

//     if pos + 3 >= args.len() {
//         return None;
//     }

//     let x = args[pos + 1].parse::<f32>().ok()?;
//     let y = args[pos + 2].parse::<f32>().ok()?;
//     let value =args[pos + 3].parse::<f64>().ok()?;

//     Some((Pos2::new(x, y), value))
// }

// fn run_grid_search_mode(
//     args: &[String],
//     run_viz: bool,
//     base_output: &str,
// ) {
//     let resolution = get_grid_resolution(args);

//     let optimization_minimum =
//         parse_optimization_minimum(args);

//     print_section("GRID SEARCH EXPERIMENT");
//     println!("Resolution      : {}x{}", resolution, resolution);
//     println!("Output directory: {base_output}");
//     print_phase("Phase 1 : DATA GENERATION");

//     run_grid_search_experiment(
//         resolution,
//         optimization_minimum,
//         base_output,
//     );

//     let json_path = format!(
//         "{}/grid_search_{}x{}_results.json",
//         base_output,
//         resolution,
//         resolution
//     );

//     if run_viz {
//         print_phase("Phase 2 : VISUALIZATION");
//         run_viz_script(
//             "viz/single_station_viz.py",
//             &json_path,
//             base_output,
//         );
//     }
// }

// fn run_optimize_mode(run_viz: bool, base_output: &str) {
//     print_section("OPTIMIZATION (EGO)");

//     let (_, json_path) =
//         optimize_station_positions_ego(20, base_output);

//     if run_viz {
//         print_phase("Phase 3 : VISUALIZATION");
//         run_viz_script(
//             "viz/optimization_viz.py",
//             &json_path,
//             base_output,
//         );
//     }
// }

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

fn main() -> Result<(), eframe::Error> {
    init_logging();

    let args: Vec<String> = std::env::args().collect();

    let base_output = create_output_directory()?;

    let run_viz = args.contains(&"--viz".to_string());

    // ---------- Single evaluation ----------------
    if args.contains(&"--single-evaluation".to_string()) {
        run_single_evaluation();
        return Ok(());
    }

    // ---------- Optimization (EGO) ---------------
    if args.contains(&"--optimize-ego".to_string()) {

        let n_stations = get_station_number(&args);

        run_ego_experiment(
            n_stations, 
            10, 
            &base_output);
        return Ok(());
    }

    // ---------- Grid search experiment -----------
    if args.contains(&"--grid-search".to_string()) {

        let resolution = get_grid_resolution(&args);

        run_grid_search_experiment(
            resolution, 
            &base_output);
        return Ok(());
    }

    // --- Single-station study (EGO + grid search) ---
    if args.contains(&"--single-station-study".to_string()) {

        let resolution = get_grid_resolution(&args);

        run_single_station_experiment(
            resolution,
            &base_output,
        );

        if run_viz {
            run_viz_script(
                "viz/single_station_viz.py",
                &format!(
                    "{}/single_station_results.json",
                    base_output
                ),
                &base_output,
            );
        }

        return Ok(());
    }

    // --- Multi(2)-station study (EGO + specialist layouts) -----
    if args.contains(&"--multi-station-study".to_string()) {

        run_multi_station_experiment(
            10, 
            &base_output);
        return Ok(());
    }

    run_gui()
}