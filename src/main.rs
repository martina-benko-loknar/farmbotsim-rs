#![deny(clippy::unwrap_used)]
#![deny(clippy::expect_used)]
#![deny(clippy::clone_on_copy)]
#![deny(clippy::redundant_clone)]
#![deny(clippy::manual_map)]
#![deny(clippy::manual_filter)]
// #![deny(clippy::panic)]
#![deny(clippy::borrowed_box)]
#![deny(clippy::dbg_macro)]
#![deny(clippy::vec_init_then_push)]
#![deny(clippy::cast_lossless)]

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
pub mod visualization;

fn main() -> Result<(), eframe::Error> {
    let args: Vec<String> = std::env::args().collect();
    
    // Check for experiment argument
    if args.contains(&"--experiment".to_string()) {
        let result = experiment::run_experiment();
        println!("\n=== Experiment Results ===");
        println!("Energy consumed: {:.2} Wh", result.total_energy_consumed);
        println!("Total distance: {:.2} m", result.total_distance_driven);
        println!("Charging distance: {:.2} m", result.total_charging_distance);
        println!("Tasks completed: {} stationary, {} moving", 
                 result.completed_stationary_tasks, result.completed_moving_tasks);
        println!("Simulation time: {:.2} seconds", result.simulation_time_seconds);
        return Ok(());
        
    } else if args.contains(&"--optimize".to_string()) {
        let optimal_config = optimization::optimize_station_positions_ego(100);
        println!("\n=== Optimization Complete ===");
        println!("Optimal configuration: {}", optimal_config);
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
        
        println!("Starting grid search with resolution {}x{}", grid_resolution, grid_resolution);
        
        // Run experiment to get data
        let results = experiment::run_grid_search_experiment_data_only(grid_resolution, optimization_minimum);
        
        // Generate all visualizations
        println!("Generating visualizations...");
        visualization::generate_all_grid_plots(&results, optimization_minimum);
        
        println!("Grid search complete with visualizations saved!");
        return Ok(());
        
    } else if args.contains(&"--plot-multiple".to_string()) {
        println!("Generating multi-station demonstration plots...");
        
        // Create demo data
        let multi_station_data = experiment::create_multi_station_demo_data();
        
        // Generate plots
        visualization::generate_all_multi_station_plots(&multi_station_data);
        
        println!("Multi-station plots generated successfully!");
        return Ok(());
        
    } else if args.contains(&"--plot-from-file".to_string()) {
        // Generate plots from a specific JSON file path
        let file_path = if let Some(pos) = args.iter().position(|x| x == "--plot-from-file") {
            if pos + 1 < args.len() {
                args[pos + 1].clone()
            } else {
                eprintln!("Error: --plot-from-file requires a file path");
                eprintln!("Usage: cargo run -- --plot-from-file <path/to/results.json>");
                return Ok(());
            }
        } else {
            eprintln!("Error: --plot-from-file requires a file path");
            return Ok(());
        };
        
        println!("Loading results from: {}", file_path);
        
        match load_grid_search_results(&file_path) {
            Ok(results) => {
                println!("Successfully loaded {} data points", results.results.len());
                visualization::generate_all_grid_plots(&results, None);
                println!("Plots generated from: {}", file_path);
            }
            Err(e) => {
                eprintln!("Failed to load results from {}: {}", file_path, e);
                eprintln!("Make sure the file exists and contains valid grid search results.");
            }
        }
        return Ok(());
        
    } else if args.contains(&"--plot-only".to_string()) {
        // Legacy: generate plots from standard results location 
        let grid_resolution = if let Some(pos) = args.iter().position(|x| x == "--plot-only") {
            if pos + 1 < args.len() {
                args[pos + 1].parse::<usize>().unwrap_or(10)
            } else {
                10
            }
        } else {
            10
        };
        
        let results_file = format!("results/grid_search_{}x{}_results.json", grid_resolution, grid_resolution);
        
        match load_grid_search_results(&results_file) {
            Ok(results) => {
                println!("Loaded results from: {}", results_file);
                visualization::generate_all_grid_plots(&results, None);
                println!("Plots generated from saved data!");
            }
            Err(e) => {
                eprintln!("Failed to load results from {}: {}", results_file, e);
                eprintln!("Run --grid-search first to generate data, or use --plot-from-file with a specific path.");
            }
        }
        return Ok(());
        
    } else if args.contains(&"--help".to_string()) || args.contains(&"-h".to_string()) {
        print_help();
        return Ok(());
    }

    // Default: Run GUI application
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

/// Load grid search results from JSON file with metadata
fn load_grid_search_results(filename: &str) -> Result<visualization::GridSearchResults, Box<dyn std::error::Error>> {
    use serde_json::Value;
    
    let content = std::fs::read_to_string(filename)?;
    let json: Value = serde_json::from_str(&content)?;
    
    // Try new format first (with metadata section), then fall back to old format
    let (grid_resolution, field_bounds, obstacles) = if let Some(metadata) = json.get("metadata") {
        // New format with metadata section
        let grid_res = metadata["grid_resolution"].as_u64()
            .ok_or("Missing grid_resolution in metadata")? as usize;
        
        let field_bounds_obj = metadata["field_bounds"].as_object()
            .ok_or("Missing field_bounds in metadata")?;
        
        let bounds = (
            field_bounds_obj["min_x"].as_f64().ok_or("Missing min_x")? as f32,
            field_bounds_obj["max_x"].as_f64().ok_or("Missing max_x")? as f32,
            field_bounds_obj["min_y"].as_f64().ok_or("Missing min_y")? as f32,
            field_bounds_obj["max_y"].as_f64().ok_or("Missing max_y")? as f32,
        );
        
        // Load obstacles from default scene config for new format
        use crate::cfg::DEFAULT_SCENE_CONFIG_PATH;
        use crate::environment::{scene_config::SceneConfig, field_config::FieldConfig};
        
        let scene_config: SceneConfig = crate::utilities::utils::load_json_or_panic(DEFAULT_SCENE_CONFIG_PATH.to_string());
        let field_config: FieldConfig = crate::utilities::utils::load_json_or_panic(scene_config.field_config_path);
        let obs = field_config.get_obstacles();
        
        (grid_res, bounds, obs)
    } else {
        // Old format with field_config at top level
        let grid_res = json["grid_resolution"].as_u64()
            .ok_or("Missing grid_resolution")? as usize;
        
        let field_config = json["field_config"].as_object()
            .ok_or("Missing field_config")?;
        
        let field_bounds_obj = field_config["field_boundaries_used_in_grid_search"].as_object()
            .ok_or("Missing field_boundaries_used_in_grid_search")?;
        
        let bounds = (
            field_bounds_obj["min_x"].as_f64().ok_or("Missing min_x")? as f32,
            field_bounds_obj["max_x"].as_f64().ok_or("Missing max_x")? as f32,
            field_bounds_obj["min_y"].as_f64().ok_or("Missing min_y")? as f32,
            field_bounds_obj["max_y"].as_f64().ok_or("Missing max_y")? as f32,
        );
        
        // Load obstacles from field_config_raw in the JSON file
        let obs = if let Some(field_config_raw) = field_config.get("field_config_raw") {
            let field_config_str = field_config_raw.as_str()
                .ok_or("field_config_raw is not a string")?;
            
            use crate::environment::field_config::FieldConfig;
            let parsed_field_config: FieldConfig = serde_json::from_str(field_config_str)?;
            parsed_field_config.get_obstacles()
        } else {
            // Fallback to default if field_config_raw is missing
            use crate::cfg::DEFAULT_SCENE_CONFIG_PATH;
            use crate::environment::{scene_config::SceneConfig, field_config::FieldConfig};
            
            let scene_config: SceneConfig = crate::utilities::utils::load_json_or_panic(DEFAULT_SCENE_CONFIG_PATH.to_string());
            let field_config: FieldConfig = crate::utilities::utils::load_json_or_panic(scene_config.field_config_path);
            field_config.get_obstacles()
        };
        
        (grid_res, bounds, obs)
    };
    
    // Extract results data
    let results_array = json["grid_search_results"].as_array()
        .ok_or("Missing grid_search_results array")?;
    
    let mut results = Vec::new();
    for result in results_array {
        let x = result["x"].as_f64().ok_or("Missing x coordinate")? as f32;
        let y = result["y"].as_f64().ok_or("Missing y coordinate")? as f32;
        let energy = result["energy_consumption"].as_f64().ok_or("Missing energy_consumption")?;
        let total_dist = result["total_distance"].as_f64().ok_or("Missing total_distance")?;
        let charging_dist = result["charging_distance"].as_f64().ok_or("Missing charging_distance")?;
        
        results.push((Pos2::new(x, y), energy, total_dist, charging_dist));
    }
    
    println!("Loaded metadata: {}x{} grid, {} data points, field bounds: ({:.1}, {:.1}) to ({:.1}, {:.1})",
             grid_resolution, grid_resolution, results.len(),
             field_bounds.0, field_bounds.2, field_bounds.1, field_bounds.3);
    println!("Loaded {} obstacles from field configuration", obstacles.len());
    
    Ok(visualization::GridSearchResults {
        results,
        grid_resolution,
        obstacles,
        field_bounds,
    })
}

/// Print help information for command-line usage
fn print_help() {
    println!("FarmBot Simulation - Command Line Options");
    println!("=========================================");
    println!();
    println!("USAGE:");
    println!("  cargo run [OPTIONS]");
    println!();
    println!("OPTIONS:");
    println!("  (no args)                    Launch interactive GUI simulation");
    println!("  --experiment                 Run single experiment (4 agents, 1000 tasks)");
    println!("  --optimize                   Run intelligent optimization (EGO algorithm)");
    println!("  --grid-search [RESOLUTION]   Run grid search analysis");
    println!("                               Optional: specify grid resolution (default: 10)");
    println!("  --grid-search [RES] --min X Y VALUE");
    println!("                               Include optimization minimum reference point");
    println!("  --plot-multiple              Generate multi-station comparison plots");
    println!("  --plot-only [RESOLUTION]     Generate plots from previously saved data");
    println!("  --plot-from-file <PATH>      Generate plots from specific JSON file");
    println!("  --help, -h                   Show this help message");
    println!();
    println!("EXAMPLES:");
    println!("  cargo run                                    # GUI mode");
    println!("  cargo run -- --experiment                   # Single experiment");
    println!("  cargo run -- --grid-search 20               # 20x20 grid search");
    println!("  cargo run -- --grid-search 15 --min 12.5 11.5 16800.0");
    println!("                                               # Grid search with reference");
    println!("  cargo run -- --optimize                     # Bayesian optimization");
    println!("  cargo run -- --plot-only 20                 # Plot from saved 20x20 data");
    println!("  cargo run -- --plot-from-file results/grid_search_15x15_results.json");
    println!("                                               # Plot from specific file");
    println!();
    println!("OUTPUT:");
    println!("  Results:        results/");
    println!("  Experiments:    experiments/");  
    println!("  Optimization:   results/optimization_*");
    println!("  Plots:          results/*.html and results/*.svg");
}