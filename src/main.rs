#![deny(clippy::unwrap_used)]       // Disallow unwrap
#![deny(clippy::expect_used)]       // Disallow expect
#![deny(clippy::clone_on_copy)]     // Disallow cloning unnecessarily
#![deny(clippy::redundant_clone)]   // Disallow redundant clones
#![deny(clippy::manual_map)]        // Disallow manual map
#![deny(clippy::manual_filter)]     // Disallow manual filter
// #![deny(clippy::panic)]             // Disallow panic!
#![deny(clippy::borrowed_box)]      // Disallow unnecessary borrowed Box
#![deny(clippy::dbg_macro)]         // Disallow dbg!() in production code
#![deny(clippy::vec_init_then_push)] // Disallow inefficient Vec initialization
#![deny(clippy::cast_lossless)]     // Disallow using `as` for conversions that could fail


pub mod app_module;
use crate::app_module::app::App;
use egui::Pos2;

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

fn main() -> Result<(), eframe::Error> {
    let args: Vec<String> = std::env::args().collect();
    
    // Check for experiment argument
    if args.contains(&"--experiment".to_string()) {
        experiment::run_experiment(); // Call the experiment function
        return Ok(());
    } else if args.contains(&"--optimize".to_string()) {
        optimization::optimize_station_positions_ego(100);
        return Ok(());                
    } else if args.contains(&"--grid-search".to_string()) {
        // Check for grid resolution argument
        let grid_resolution = if let Some(pos) = args.iter().position(|x| x == "--grid-search") {
            if pos + 1 < args.len() {
                args[pos + 1].parse::<usize>().unwrap_or(10)
            } else {
                10 // Default resolution
            }
        } else {
            10
        };
        // Check for minimum argument (expects: --min x y value)
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
        experiment::run_grid_search_experiment(grid_resolution, optimization_minimum);
        return Ok(());
    } else if args.contains(&"--plot-multiple".to_string()) {
        experiment::multi_station_plot_function();
        return Ok(());
    }

    let options = eframe::NativeOptions {
        vsync: true,
        viewport: egui::ViewportBuilder::default()
            .with_decorations(true)
            .with_maximized(true)
            // .with_inner_size([1200.0, 800.0]) // Width, height
            // .with_min_inner_size([400.0, 300.0])
            .with_resizable(true),
        ..Default::default()
    };
    eframe::run_native(
        "farmbotsim-rs",
        options,
        Box::new(|_cc| Ok(Box::new(App::default()))),
    )
}