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

use std::process::Command;

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
use crate::experiment::single_evaluation::run_single_evaluation;
use crate::experiment::grid_search::run_grid_search_experiment;
use crate::experiment::visualization::multi_station_plot_function;
use crate::optimization::ego::optimize_station_positions_ego;

use egui::Pos2;

fn separator() {
    println!("{}", "=".repeat(70));
}

fn get_arg_value(args: &[String], flag: &str) -> Option<String> {
    args.iter()
        .position(|x| x == flag)
        .and_then(|pos| args.get(pos + 1))
        .cloned()
}

fn run_python_viz(json_path: &str, output_dir: &str) {

    let status = Command::new("python")
        .arg("viz/single_station_viz.py")
        .arg(&json_path)
        .arg(output_dir)
        .status();

    match status {
        Ok(s) if s.success() => {
            //println!("Python visualization completed successfully.");
        }
        Ok(_) => {
            eprintln!("Python visualization failed (non-zero exit code).");
        }
        Err(e) => {
            eprintln!("Failed to run Python visualization: {e}");
        }
    }
}

fn main() -> Result<(), eframe::Error> {
    let args: Vec<String> = std::env::args().collect();

    // -----------------------------
    // Global CLI flags
    // -----------------------------
    let run_viz = args.contains(&"--viz".to_string());
    let output_name = get_arg_value(&args, "--out")
        .unwrap_or_else(|| "default_run".to_string());

    let base_output = format!("results/{}", output_name);

    // -----------------------------
    // Single evaluation
    // -----------------------------
    if args.contains(&"--single-evaluation".to_string()) {
        run_single_evaluation();
        return Ok(());
    }

    // -----------------------------
    // Optimization (EGO)
    // -----------------------------
    if args.contains(&"--optimize".to_string()) {
        optimize_station_positions_ego(50);
        return Ok(());
    }

    // -----------------------------
    // Grid search experiment
    // -----------------------------
    if args.contains(&"--grid-search".to_string()) {
        let grid_resolution = get_arg_value(&args, "--grid-search")
            .and_then(|v| v.parse::<usize>().ok())
            .unwrap_or(10);

        // optional minimum argument: --min x y value
        let optimization_minimum = if let Some(pos) = args.iter().position(|x| x == "--min") {
            if pos + 3 < args.len() {
                let x = args[pos + 1].parse::<f32>().unwrap_or(0.0);
                let y = args[pos + 2].parse::<f32>().unwrap_or(0.0);
                let value = args[pos + 3].parse::<f64>().unwrap_or(0.0);
                Some((Pos2::new(x, y), value))
            } else {
                None
            }
        } else {
            None
        };

        separator();
        println!("GRID SEARCH EXPERIMENT");
        separator();
        println!("Resolution      : {}x{}", grid_resolution, grid_resolution);
        println!("Output directory: {}", base_output);

        println!("\n====== Phase 1 : DATA GENERATION =====================================");
        
        // separator_bottom();
        // println!("Phase 1 -- DATA GENERATION");
        // separator_top();
        run_grid_search_experiment(grid_resolution, optimization_minimum, &base_output);

        let json_path = format!(
            "{}/grid_search_{}x{}_results.json",
            base_output,
            grid_resolution,
            grid_resolution
        );

        if run_viz {
            println!("\n====== Phase 2 : VISUALIZATION =======================================");
            run_python_viz(&json_path, &base_output);
        }

        return Ok(());
    }

    // -----------------------------
    // Multi-station plot mode
    // -----------------------------
    if args.contains(&"--plot-multiple".to_string()) {
        multi_station_plot_function();
        return Ok(());
    }

    // -----------------------------
    // GUI mode (default)
    // -----------------------------
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