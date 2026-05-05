use crate::cfg::{DEFAULT_SCENE_CONFIG_PATH};
use crate::environment::{
    scene_config::SceneConfig,
    field_config::FieldConfig
};
use crate::utilities::utils::load_json_or_panic;
use crate::optimization::station_positions::StationPositions;
use crate::optimization::objective::OptimizationContext;
use crate::optimization::objective::station_objective_function;
use crate::optimization::geometry::round_to_centimeters;
//use crate::optimization::visualization;
use crate::optimization::results;
use crate::optimization::geometry::is_position_valid;
use crate::optimization::constants::*;

// Main EGO optimization function
use egobox_ego::EgorBuilder;
use ndarray::{Array2, ArrayView2};
use std::sync::{Arc, RwLock};
use std::time::Instant;
use egui::Pos2;

pub fn optimize_station_positions_ego(
    max_iterations: usize,
    output_dir: &str
    ) -> (StationPositions, String) {
    
    let start_time = Instant::now();
    
    // Load scene config to get number of stations and obstacles
    let scene_config: SceneConfig = load_json_or_panic(DEFAULT_SCENE_CONFIG_PATH.to_string());
    let n_stations = scene_config.station_configs.len();
    
    let field_config: FieldConfig = load_json_or_panic(scene_config.field_config_path);
    let obstacles = field_config.get_obstacles();

    // Define bounds for each station position
    let mut bounds_vec = Vec::with_capacity(n_stations * 4);
    for _ in 0..n_stations {
        bounds_vec.push(FIELD_MIN_X as f64 + STATION_MARGIN as f64); // min x
        bounds_vec.push(FIELD_MAX_X as f64 - STATION_MARGIN as f64); // max x
        bounds_vec.push(FIELD_MIN_Y as f64 + STATION_MARGIN as f64); // min y
        bounds_vec.push(FIELD_MAX_Y as f64 - STATION_MARGIN as f64); // max y
    }

    let bounds_array = Array2::from_shape_vec((n_stations * 2, 2), bounds_vec).unwrap();
    //println!("Bounds array: {:?}", bounds_array);

    // Generate initial valid starting points
    let mut rng = rand::rng();
    let initial_positions = StationPositions::generate_initial_population(&obstacles, n_stations, 100, &mut rng);

    println!("Stations         : {}", n_stations);
    println!("Max iterations   : {}", max_iterations);
    println!("Initial samples  : {}", initial_positions.len());
    println!(
        "Field bounds     : X[{:.1}, {:.1}], Y[{:.1}, {:.1}]",
        FIELD_MIN_X, FIELD_MAX_X, FIELD_MIN_Y, FIELD_MAX_Y
    );   
    println!("Station margin   : {:.1}m", STATION_MARGIN);
    println!("Obstacle margin  : {:.1}m",  OBSTACLE_MARGIN);

    println!("\n====== Phase 1 : INITIAL SAMPLING ====================================");
    println!("Generated {} initial samples", initial_positions.len());
    println!();

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
    // let convergence_history = Arc::new(RwLock::new(Vec::new())); // Track best values over iterations

    // Create a closure that captures the context
    let objective_fn = {
        let context = OptimizationContext {
            obstacles: obstacles.clone(),
            n_stations,
            max_iterations
        };
        let evaluated_positions = Arc::clone(&evaluated_positions); // Clone the Arc
        // let convergence_history = Arc::clone(&convergence_history); // Clone the Arc for convergence tracking
        move |x: &ArrayView2<f64>| {
            let mut positions_log = evaluated_positions.write().unwrap(); // Lock for writing
            // let mut convergence = convergence_history.write().unwrap(); // Lock for writing
            //let y = station_objective_function(x, &context, &mut positions, &mut convergence);
            let y = station_objective_function(
                x,
                &context,
                &mut positions_log
            );

            // // Debug: verify exploration
            // println!("Evaluating batch of size {}", x.nrows());
            // println!("First candidate: {:?}", x.row(0));

            for (i, row) in x.outer_iter().enumerate() {
                let stations: Vec<(f64, f64)> = row
                    .iter()
                    .cloned()
                    .collect::<Vec<_>>()
                    .chunks(2)
                    .map(|c| (c[0], c[1]))
                    .collect();

                println!("\n[Eval {}] Candidate:", i + 1);
                for (j, (sx, sy)) in stations.iter().enumerate() {
                    println!("  S{} → ({:.2}, {:.2})", j + 1, sx, sy);
                }
            }

            for (i, val) in y.iter().enumerate() {
                println!("  Energy → {:.2} kWh", val / 1000.0);
}

            y
            // let iter = convergence.len();
            // let best = convergence.last().unwrap().1;
            // let progress = (iter as f32 / context.max_iterations as f32) * 100.0;

            // println!(
            //     "[{:>3}% | iter {}/{}] batch size = {} | best = {:.2} kWh",
            //     progress.round() as i32,
            //     iter,
            //     context.max_iterations,
            //     y.nrows(),
            //     best/1000.0
            // );

            // y
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
            if !is_position_valid(position, &obstacles) {
                constraint_violation += 1.0; // Positive value indicates constraint violation
            }
        }
        
        // EGO constraints: <= 0 means feasible, > 0 means infeasible
        constraint_violation
    }

    // Run EGO optimization
    println!("====== Phase 2 : BAYESIAN OPTIMIZATION ================================");
    println!("Fitting surrogate model from initial samples...");
    println!("Starting Bayesian optimization loop...");

    let result = EgorBuilder::optimize(objective_fn)
        .configure(|config| {
            config
                .max_iters(max_iterations)
                .doe(&initial_x)
        })
        .subject_to(vec![constraint_fn])
        .min_within(&bounds_array)
        .run();

    // let result = {
    //     let _stdout = Redirect::stdout(std::io::stdout()).unwrap();
    //     let _stderr = Redirect::stderr(std::io::stderr()).unwrap();

    //     EgorBuilder::optimize(objective_fn)
    //         .configure(|config| {
    //             config
    //                 .max_iters(max_iterations)
    //                 .doe(&initial_x)
    //         })
    //         .subject_to(vec![constraint_fn])
    //         .min_within(&bounds_array)
    //         .run()
    // };

    //println!("Optimization finished");

    let elapsed = start_time.elapsed();
    
    match result {
        Ok(optimization_result) => {
            // println!("EGO optimization completed in {:?}", elapsed);
            
            println!("Optimization finished in {:?}", elapsed);
            println!("Best x: {:?}", optimization_result.x_opt);
            println!("Best y: {:?}", optimization_result.y_opt);

            // Convert result back to StationPositions
            let best_positions = StationPositions::from_optimization_vector(
                &optimization_result.x_opt.view().insert_axis(ndarray::Axis(0)),
                &obstacles,
                n_stations
            );
            let secs = elapsed.as_secs_f64();
            let mins = (secs / 60.0).floor();
            let rem_secs = secs % 60.0;

            println!();
            println!("====== RESULTS ========================================================");
            println!("Best energy : {:.2} kWh", optimization_result.y_opt[0]/1000.0);
            println!("Stations: {}", best_positions.station_positions.len());
            for (i, p) in best_positions.station_positions.iter().enumerate() {
                println!("  {} → ({:.2}, {:.2})", i + 1, p.x, p.y);
            }
            //println!("Best positions : {}", best_positions);
            println!("Total time : {:.0}min {:.2}s", mins, rem_secs);

            // Generate research-ready visualizations
            //visualization::visualize_optimization_results(&evaluated_positions.read().unwrap(), &obstacles);
            //visualization::generate_convergence_plot(&convergence_history.read().unwrap());

            let evaluated = evaluated_positions.read().unwrap();

            let mut best_so_far = f64::INFINITY;
            let mut convergence_history_vec = Vec::new();

            for (i, (_pos, energy)) in evaluated.iter().enumerate() {
                if *energy < best_so_far {
                    best_so_far = *energy;
                }

                convergence_history_vec.push((
                    i + 1,
                    best_so_far,
                    _pos.clone(),
                ));
            }

            // Save convergence history to JSON file
            //let convergence_data = convergence_history.read().unwrap();
            let total_evaluations = evaluated_positions.read().unwrap().len();
            let json_path = results::save_convergence_history(
                &convergence_history_vec,
                max_iterations, 
                total_evaluations, 
                elapsed,
                output_dir,
                &evaluated_positions.read().unwrap(),
                &obstacles,
            );

            (best_positions, json_path)
        },
        Err(e) => {
            eprintln!("EGO optimization failed: {:?}", e);
            println!("Falling back to random initial position");
            
            // Return a random valid configuration as fallback
            let mut rng = rand::rng();
            let fallback = StationPositions::generate_initial_population(
                &obstacles, 
                n_stations, 
                100, 
                &mut rng
            )
            .into_iter()
            .next()
            .unwrap();

            (fallback, String::from(""))
        }
    }
}