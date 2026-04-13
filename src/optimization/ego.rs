use crate::cfg::{DEFAULT_SCENE_CONFIG_PATH};
use crate::environment::{
    scene_config::SceneConfig,
    field_config::FieldConfig
};
use crate::utilities::utils::load_json_or_panic;
use crate::optimization::StationPositions;
use crate::optimization::OptimizationContext;
use crate::optimization::station_objective_function;
use crate::optimization::round_to_centimeters;
use crate::optimization::visualization;
use crate::optimization::results;

// Main EGO optimization function
use egobox_ego::EgorBuilder;
use ndarray::{Array2, ArrayView2};
use std::sync::{Arc, RwLock};
use std::time::Instant;
use egui::Pos2;

// Define field boundaries for optimization (adjust these based on your actual farm layout)
const FIELD_MIN_X: f32 = 0.0;
const FIELD_MAX_X: f32 = 25.0;  // width in meters
const FIELD_MIN_Y: f32 = 0.0;
const FIELD_MAX_Y: f32 = 25.0;  // height in meters
const STATION_MARGIN: f32 = 0.4; // Keep stations at least 0.4m from field edges
const OBSTACLE_MARGIN: f32 = 0.4; // Keep stations at least 0.4m from obstacles

pub fn optimize_station_positions_ego(max_iterations: usize) -> StationPositions {
    println!("Starting EGO optimization for charging station positions...");
    println!("Field boundaries: X[{:.1}, {:.1}], Y[{:.1}, {:.1}]", 
             FIELD_MIN_X, FIELD_MAX_X, FIELD_MIN_Y, FIELD_MAX_Y);
    println!("Station margin: {:.1}m, Obstacle margin: {:.1}m", STATION_MARGIN, OBSTACLE_MARGIN);
    println!("Max iterations: {}", max_iterations);
    
    let start_time = Instant::now();
    
    // Load scene config to get number of stations and obstacles
    let scene_config: SceneConfig = load_json_or_panic(DEFAULT_SCENE_CONFIG_PATH.to_string());
    let n_stations = scene_config.station_configs.len();
    
    let field_config: FieldConfig = load_json_or_panic(scene_config.field_config_path);
    let obstacles = field_config.get_obstacles();
    
    println!("Optimizing {} charging stations", n_stations);

    // Define bounds for each station position
    let mut bounds_vec = Vec::with_capacity(n_stations * 4);
    for _ in 0..n_stations {
        bounds_vec.push(FIELD_MIN_X as f64 + STATION_MARGIN as f64); // min x
        bounds_vec.push(FIELD_MAX_X as f64 - STATION_MARGIN as f64); // max x
        bounds_vec.push(FIELD_MIN_Y as f64 + STATION_MARGIN as f64); // min y
        bounds_vec.push(FIELD_MAX_Y as f64 - STATION_MARGIN as f64); // max y
    }

    let bounds_array = Array2::from_shape_vec((n_stations * 2, 2), bounds_vec).unwrap();
    println!("Bounds array: {:?}", bounds_array);

    // Generate initial valid starting points
    let mut rng = rand::rng();
    let initial_positions = StationPositions::generate_initial_population(&obstacles, n_stations, 100, &mut rng);
    
    // Convert initial population to optimization vectors
    let initial_x: Array2<f64> = Array2::from_shape_vec(
        (initial_positions.len(), n_stations * 2),
        initial_positions
            .iter()
            .flat_map(|pos| {
                pos.station_positions
                    .iter()
                    .flat_map(|p| vec![p.x as f64, p.y as f64])
            })
            .collect(),
    ).unwrap();

    let evaluated_positions = Arc::new(RwLock::new(Vec::new()));
    let convergence_history = Arc::new(RwLock::new(Vec::new())); // Track best values over iterations

    // Create a closure that captures the context
    let objective_fn = {
        let context = OptimizationContext {
            obstacles: obstacles.clone(),
            n_stations,
        };
        let evaluated_positions = Arc::clone(&evaluated_positions); // Clone the Arc
        let convergence_history = Arc::clone(&convergence_history); // Clone the Arc for convergence tracking
        move |x: &ArrayView2<f64>| {
            let mut positions = evaluated_positions.write().unwrap(); // Lock for writing
            let mut convergence = convergence_history.write().unwrap(); // Lock for writing
            station_objective_function(x, &context, &mut positions, &mut convergence)
        }
    };


    fn constraint_fn(
        x: &[f64], 
        g: Option<&mut [f64]>, 
        _u: &mut egobox_ego::InfillObjData<f64>
    ) -> f64 {
        if let Some(g) = g {
            g[0] = 0.0; // Placeholder for gradient if needed
        }

        let scene_config: SceneConfig = load_json_or_panic(DEFAULT_SCENE_CONFIG_PATH.to_string());
        let n_stations = scene_config.station_configs.len();
        let field_config: FieldConfig = load_json_or_panic(scene_config.field_config_path);
        let obstacles = field_config.get_obstacles();
        
        // Convert slice to positions
        let n_vars = x.len();
        let expected_vars = n_stations * 2;
        
        if n_vars != expected_vars {
            return 1.0; // Constraint violation for wrong dimensionality
        }
        
        // Extract station positions from the flat array and check validity
        let mut constraint_violation = 0.0;
        for i in 0..n_stations {
            let x_coord = x[i * 2] as f32;
            let y_coord = x[i * 2 + 1] as f32;
            
            // Clamp to field boundaries
            let x_clamped = x_coord.clamp(FIELD_MIN_X + STATION_MARGIN, FIELD_MAX_X - STATION_MARGIN);
            let y_clamped = y_coord.clamp(FIELD_MIN_Y + STATION_MARGIN, FIELD_MAX_Y - STATION_MARGIN);
            
            // Round to centimeters to prevent floating-point precision issues
            let position = round_to_centimeters(Pos2::new(x_clamped, y_clamped));
            
            // Check if position is valid
            if !StationPositions::is_position_valid(position, &obstacles) {
                constraint_violation += 1.0; // Positive value indicates constraint violation
            }
        }
        
        // EGO constraints: <= 0 means feasible, > 0 means infeasible
        constraint_violation
    }



    // Run EGO optimization
    let result = EgorBuilder::optimize(objective_fn)
        .configure(|config| {
            config
                .max_iters(max_iterations)
                .doe(&initial_x)
        })
        .subject_to(vec![constraint_fn])
        .min_within(&bounds_array)
        .run();
    
    let elapsed = start_time.elapsed();
    
    match result {
        Ok(optimization_result) => {
            println!("EGO optimization completed in {:?}", elapsed);
            println!("Best energy consumption: {:.2} Wh", optimization_result.y_opt[0]);
            
            // Convert result back to StationPositions
            let best_positions = StationPositions::from_optimization_vector(
                &optimization_result.x_opt.view().insert_axis(ndarray::Axis(0)),
                &obstacles,
                n_stations
            );
            
            println!("Optimal configuration: {}", best_positions);

            // Generate research-ready visualizations
            visualization::visualize_optimization_results(&evaluated_positions.read().unwrap(), &obstacles);
            visualization::generate_convergence_plot(&convergence_history.read().unwrap());
            
            // Save convergence history to JSON file
            let convergence_data = convergence_history.read().unwrap();
            let total_evaluations = evaluated_positions.read().unwrap().len();
            results::save_convergence_history(&convergence_data, max_iterations, total_evaluations, elapsed);

            best_positions
        },
        Err(e) => {
            eprintln!("EGO optimization failed: {:?}", e);
            println!("Falling back to random initial position");
            
            // Return a random valid configuration as fallback
            let mut rng = rand::rng();
            StationPositions::generate_initial_population(&obstacles, n_stations, 100, &mut rng)
                .into_iter()
                .next()
                .unwrap()
        }
    }
}