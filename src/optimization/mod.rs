use crate::cfg::{DEFAULT_SCENE_CONFIG_PATH, DEFAULT_AGENT_CONFIG_PATH, 
    OPTIMIZATION_RESULTS_PATH};
use crate::environment::{
    datetime::DateTimeConfig, 
    env_module::env_config::EnvConfig,
    scene_config::SceneConfig,
    station_module::station_config::StationConfig,
    field_config::FieldConfig,
    obstacle::Obstacle
};
use crate::tool_module::experiment_tool::{ExperimentRunner, TerminationCondition};
use crate::units::{energy::Energy, duration::Duration};
use crate::utilities::utils::load_json_or_panic;
use crate::task_module::strategies::{ChargingStrategy, ChooseStationStrategy};

use egobox_ego::EgorBuilder;
use ndarray::{Array2, ArrayView2};
use rand::Rng;
use std::fmt;
use std::sync::{Arc, RwLock};
use std::time::Instant;
use egui::Pos2;
use serde::{Deserialize, Serialize};

// Serializable structure for convergence history
#[derive(Serialize, Deserialize, Clone, Debug)]
pub struct ConvergenceRecord {
    pub iteration: usize,
    pub best_energy: f64,
    pub best_positions: Vec<(f32, f32)>,
}

#[derive(Serialize, Deserialize, Clone, Debug)]
pub struct OptimizationResults {
    pub timestamp: String,
    pub max_iterations: usize,
    pub total_evaluations: usize,
    pub optimization_time_seconds: f64,
    pub final_best_energy: f64,
    pub convergence_history: Vec<ConvergenceRecord>,
    pub field_boundaries: (f32, f32, f32, f32), // (min_x, max_x, min_y, max_y)
    pub station_margin: f32,
    pub obstacle_margin: f32,
}

pub mod visualization;

// Define field boundaries for optimization (adjust these based on your actual farm layout)
const FIELD_MIN_X: f32 = 0.0;
const FIELD_MAX_X: f32 = 25.0;  // width in meters
const FIELD_MIN_Y: f32 = 0.0;
const FIELD_MAX_Y: f32 = 25.0;  // height in meters
const STATION_MARGIN: f32 = 0.4; // Keep stations at least 0.4m from field edges
const OBSTACLE_MARGIN: f32 = 0.4; // Keep stations at least 0.4m from obstacles

/// Round coordinates to 2 decimal places (centimeters) to prevent floating-point precision issues
fn round_to_centimeters(pos: Pos2) -> Pos2 {
    Pos2::new(
        (pos.x * 100.0).round() / 100.0,
        (pos.y * 100.0).round() / 100.0
    )
}

// Define the parameters for station position optimization 
#[derive(Clone)]
pub struct StationPositions {
    station_positions: Vec<Pos2>,
    obstacles: Vec<Obstacle>, // Keep obstacles for validation
}

impl StationPositions {
    // Create a new random set of station positions with FIXED count from scene config
    pub fn random() -> Self {
        let mut rng = rand::rng();
        
        // Load the default scene to get the number of existing stations
        let scene_config: SceneConfig = load_json_or_panic(DEFAULT_SCENE_CONFIG_PATH.to_string());
        let n_stations = scene_config.station_configs.len();
        
        // Load field config to get obstacles
        let field_config: FieldConfig = load_json_or_panic(scene_config.field_config_path);
        let obstacles = field_config.get_obstacles();
        
        let mut station_positions = Vec::with_capacity(n_stations);
        
        // Generate random positions within field boundaries, avoiding obstacles
        for _ in 0..n_stations {
            let position = Self::generate_valid_position(&obstacles, &mut rng);
            station_positions.push(position);
        }
        
        Self {
            station_positions,
            obstacles,
        }
    }

    // Create StationPositions from optimization vector
    pub fn from_optimization_vector(x: &ArrayView2<f64>, obstacles: &[Obstacle], n_stations: usize) -> Self {
        let mut station_positions = Vec::with_capacity(n_stations);
        
        // Extract positions from optimization vector [x1, y1, x2, y2, ...]
        for i in 0..n_stations {
            let x_coord = x[[0, i * 2]] as f32;
            let y_coord = x[[0, i * 2 + 1]] as f32;
            
            // Clamp to field boundaries
            let x_clamped = x_coord.clamp(FIELD_MIN_X + STATION_MARGIN, FIELD_MAX_X - STATION_MARGIN);
            let y_clamped = y_coord.clamp(FIELD_MIN_Y + STATION_MARGIN, FIELD_MAX_Y - STATION_MARGIN);
            
            // Round to centimeters to prevent floating-point precision issues
            station_positions.push(round_to_centimeters(Pos2::new(x_clamped, y_clamped)));
        }
        
        Self {
            station_positions,
            obstacles: obstacles.to_vec(),
        }
    }

    // Generate a valid position that doesn't intersect with obstacles
    fn generate_valid_position(obstacles: &[Obstacle], rng: &mut impl Rng) -> Pos2 {
        let max_attempts = 100; // Prevent infinite loops
        
        for _ in 0..max_attempts {
            let x = rng.random_range((FIELD_MIN_X + STATION_MARGIN)..(FIELD_MAX_X - STATION_MARGIN));
            let y = rng.random_range((FIELD_MIN_Y + STATION_MARGIN)..(FIELD_MAX_Y - STATION_MARGIN));
            let candidate = round_to_centimeters(Pos2::new(x, y));
            
            // Check if this position is valid (doesn't intersect with obstacles)
            if Self::is_position_valid(candidate, obstacles) {
                return candidate;
            }
        }
        
        // Fallback: return a position at field center if no valid position found
        println!("Warning: Could not find valid position after {} attempts, using field center", max_attempts);
        round_to_centimeters(Pos2::new(
            (FIELD_MIN_X + FIELD_MAX_X) / 2.0,
            (FIELD_MIN_Y + FIELD_MAX_Y) / 2.0
        ))
    }
    
    // Check if a position is valid (not inside or too close to obstacles)
    fn is_position_valid(position: Pos2, obstacles: &[Obstacle]) -> bool {
        for obstacle in obstacles {
            // Check if position is inside obstacle or too close to it
            if Self::is_point_near_obstacle(position, obstacle, OBSTACLE_MARGIN) {
                return false;
            }
        }
        true
    }
    
    // Check if a point is inside an obstacle or within a certain margin
    fn is_point_near_obstacle(point: Pos2, obstacle: &Obstacle, margin: f32) -> bool {
        // First check if point is inside the obstacle polygon
        if Self::is_point_inside_polygon(point, &obstacle.points) {
            return true;
        }
        
        // Check if point is too close to any edge of the obstacle
        for window in obstacle.points.windows(2) {
            if let [p1, p2] = window {
                let distance = Self::point_to_line_distance(point, *p1, *p2);
                if distance < margin {
                    return true;
                }
            }
        }
        
        // Also check the closing edge (last point to first point)
        if obstacle.points.len() >= 2 {
            let first = obstacle.points[0];
            let last = obstacle.points[obstacle.points.len() - 1];
            let distance = Self::point_to_line_distance(point, last, first);
            if distance < margin {
                return true;
            }
        }
        
        false
    }
    
    // Ray casting algorithm to determine if point is inside polygon
    fn is_point_inside_polygon(point: Pos2, polygon: &[Pos2]) -> bool {
        if polygon.len() < 3 {
            return false;
        }
        
        let mut inside = false;
        let mut j = polygon.len() - 1;
        
        for i in 0..polygon.len() {
            let pi = polygon[i];
            let pj = polygon[j];
            
            if ((pi.y > point.y) != (pj.y > point.y)) &&
               (point.x < (pj.x - pi.x) * (point.y - pi.y) / (pj.y - pi.y) + pi.x) {
                inside = !inside;
            }
            j = i;
        }
        
        inside
    }
    
    // Calculate distance from point to line segment
    fn point_to_line_distance(point: Pos2, line_start: Pos2, line_end: Pos2) -> f32 {
        let line_vec = Pos2::new(line_end.x - line_start.x, line_end.y - line_start.y);
        let point_vec = Pos2::new(point.x - line_start.x, point.y - line_start.y);
        
        let line_len_sq = line_vec.x * line_vec.x + line_vec.y * line_vec.y;
        if line_len_sq == 0.0 {
            // Line start and end are the same point
            return (point_vec.x * point_vec.x + point_vec.y * point_vec.y).sqrt();
        }
        
        let t = (point_vec.x * line_vec.x + point_vec.y * line_vec.y) / line_len_sq;
        let t = t.clamp(0.0, 1.0);
        
        let projection = Pos2::new(
            line_start.x + t * line_vec.x,
            line_start.y + t * line_vec.y
        );
        
        let dist_vec = Pos2::new(point.x - projection.x, point.y - projection.y);
        (dist_vec.x * dist_vec.x + dist_vec.y * dist_vec.y).sqrt()
    }
    
    // Evaluate the fitness (energy consumption) of a station configuration
    pub fn evaluate(&self) -> f64 {
        // Load default scene config
        let scene_config: SceneConfig = load_json_or_panic(DEFAULT_SCENE_CONFIG_PATH.to_string());
        
        // Create new scene config with updated station positions
        let mut new_scene_config = scene_config.clone();
        new_scene_config.station_configs = self.create_station_configs(&scene_config);
        
        // Save the modified scene config to a temporary file
        let random_id: u32 = rand::random();
        let temp_scene_path = format!("configs/scene_configs/temp_opt_{}.json", random_id);
        let serialized = serde_json::to_string(&new_scene_config).unwrap();
        std::fs::write(&temp_scene_path, &serialized).unwrap();
        
        // Create environment config using defaults
        let mut env_config = EnvConfig::default();
        env_config.scene_config_path = temp_scene_path.clone();
        env_config.agent_config_path = DEFAULT_AGENT_CONFIG_PATH.to_string();
        env_config.datetime_config = DateTimeConfig::from_string("01.01.2025 08:00:00".to_string());
        env_config.n_agents = 4;

        env_config.task_manager_config.charging_strategy = ChargingStrategy::CriticalOnly;
        env_config.task_manager_config.choose_station_strategy = ChooseStationStrategy::ClosestMinQueueManhattan;

        // Create experiment runner
        let runner_random_id: u32 = rand::random();
        let mut runner = ExperimentRunner {
            running: false,
            scene_config_path: temp_scene_path.clone(),
            agent_config_path: env_config.agent_config_path.clone(),
            datetime_config: env_config.datetime_config.clone(),
            env_config,
            termination_condition: TerminationCondition::NumberCompletedTasks(1000),
            env: None,
            save_to_file: false,
            save_file_name: format!("station_opt_{}", runner_random_id),
            start_datetime: None,
            start_time: None,
            total_energy_consumed: Energy::watt_hours(0.0),
            total_distance_driven: 0.0,
            total_charging_distance: 0.0,
            total_charging_approach_distance: 0.0,
            total_charging_departure_distance: 0.0,
            agents_departing_from_charging: Vec::new(),
            completed_stationary_tasks: 0,
            completed_moving_tasks: 0,
            agent_actions: Vec::new(),
            previous_agent_states: Vec::new(),
            previous_agent_positions: Vec::new(),
            step_start_time: Duration::ZERO,
        };
        
        // Run the experiment
        runner.run_simulation();
        
        // Clean up the temporary file
        let _ = std::fs::remove_file(temp_scene_path);
        
        // Add penalty for stations too close to obstacles (extra safety check)
        let mut penalty = 0.0;
        for position in &self.station_positions {
            if !Self::is_position_valid(*position, &self.obstacles) {
                penalty += 1000000000.0; // Heavy penalty for invalid positions
            }
        }
        
        // Return the energy consumption plus penalty
        runner.total_energy_consumed.value as f64 + penalty
    }
    
    // Helper method to create station configs from optimized positions
    fn create_station_configs(&self, original_scene: &SceneConfig) -> Vec<StationConfig> {
        let mut station_configs = Vec::new();
        
        // Use original stations as template but with new positions
        for (i, position) in self.station_positions.iter().enumerate() {
            let original_station = if i < original_scene.station_configs.len() {
                &original_scene.station_configs[i]
            } else {
                &original_scene.station_configs[0] // Fallback to first station
            };
            
            // Create new station config with optimized position but keep other properties
            let mut station_config = original_station.clone();
            station_config.pose.position = *position;
            
            station_configs.push(station_config);
        }
        
        station_configs
    }

    // Generate initial population for EGO
    fn generate_initial_population(obstacles: &[Obstacle], n_stations: usize, population_size: usize, rng: &mut impl Rng) -> Vec<StationPositions> {
        let mut population = Vec::with_capacity(population_size);
        
        for _ in 0..population_size {
            let mut station_positions = Vec::with_capacity(n_stations);
            
            for _ in 0..n_stations {
                let position = Self::generate_valid_position(obstacles, rng);
                station_positions.push(position);
            }
            
            population.push(StationPositions {
                station_positions,
                obstacles: obstacles.to_vec(),
            });
        }
        
        population
    }
}

// Display implementation for logging
impl fmt::Display for StationPositions {
    fn fmt(&self, f: &mut fmt::Formatter) -> fmt::Result {
        write!(f, "Stations: {} - Positions: [", self.station_positions.len())?;
        
        for (i, pos) in self.station_positions.iter().enumerate() {
            if i > 0 { write!(f, ", ")?; }
            write!(f, "({:.1},{:.1})", pos.x, pos.y)?;
        }
        
        write!(f, "]")
    }
}

// Context struct to hold optimization parameters
#[derive(Clone)]
struct OptimizationContext {
    obstacles: Vec<Obstacle>,
    n_stations: usize,
}

// Objective function for EGO optimization 
fn station_objective_function(
    x: &ArrayView2<f64>,
    context: &OptimizationContext,
    evaluated_positions: &mut Vec<(Vec<(f32, f32)>, f64)>,
    convergence_history: &mut Vec<(usize, f64, Vec<(f32, f32)>)>, // Track (iteration, best_energy, best_positions)
) -> Array2<f64> {
    let mut y: Array2<f64> = Array2::zeros((x.nrows(), 1));
    let mut current_best = f64::INFINITY;
    let mut current_best_positions = Vec::new();

    for (i, xi) in x.rows().into_iter().enumerate() {
        let positions = StationPositions::from_optimization_vector(
            &xi.insert_axis(ndarray::Axis(0)),
            &context.obstacles,
            context.n_stations,
        );
        let energy = positions.evaluate();

        // Update current best for this batch
        if energy < current_best {
            current_best = energy;
            current_best_positions = positions
                .station_positions
                .iter()
                .map(|p| (p.x, p.y))
                .collect();
        }

        // Log the evaluation
        println!("Evaluated positions: {} -> Energy: {:.2} Wh", positions, energy);

        // Store the evaluated positions and energy
        evaluated_positions.push((
            positions
                .station_positions
                .iter()
                .map(|p| (p.x, p.y))
                .collect(),
            energy,
        ));

        y[[i, 0]] = energy;
    }

    // Update convergence history with the best value found in this evaluation batch
    let current_iteration = convergence_history.len() + 1;
    
    // Get the global best so far (including previous iterations)
    let (global_best_energy, global_best_positions) = if let Some((_, prev_best_energy, prev_best_positions)) = convergence_history.last() {
        if current_best < *prev_best_energy {
            (current_best, current_best_positions)
        } else {
            (*prev_best_energy, prev_best_positions.clone())
        }
    } else {
        (current_best, current_best_positions)
    };
    
    convergence_history.push((current_iteration, global_best_energy, global_best_positions));
    println!("Iteration {}: Best energy so far: {:.2} Wh", current_iteration, global_best_energy);

    y
}



// Main EGO optimization function
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
            save_convergence_history(&convergence_data, max_iterations, total_evaluations, elapsed);

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

// Save the optimal station configuration
pub fn save_optimal_station_config(positions: &StationPositions, filename: &str) -> String {
    // Create a scene config from the optimal parameters
    let scene_config: SceneConfig = load_json_or_panic(DEFAULT_SCENE_CONFIG_PATH.to_string());
    let mut new_scene_config = scene_config.clone();
    new_scene_config.station_configs = positions.create_station_configs(&scene_config);
    
    // Save to a file
    let path = format!("configs/scenes/{}", filename);
    let serialized = serde_json::to_string_pretty(&new_scene_config).unwrap();
    std::fs::write(&path, serialized).unwrap_or_else(|e| {
        eprintln!("Failed to save optimal configuration: {}", e);
    });
    
    println!("Station positions validation:");
    for (i, pos) in positions.station_positions.iter().enumerate() {
        let valid = StationPositions::is_position_valid(*pos, &positions.obstacles);
        println!("  Station {}: ({:.1}, {:.1}) - {}", 
                i + 1, pos.x, pos.y, if valid { "✓ Valid" } else { "✗ Invalid" });
    }
    
    path
}

/// Save convergence history to JSON file
pub fn save_convergence_history(
    convergence_history: &[(usize, f64, Vec<(f32, f32)>)],
    max_iterations: usize,
    total_evaluations: usize,
    optimization_time: std::time::Duration,
) {
    let timestamp = chrono::Utc::now().format("%Y%m%d_%H%M%S");
    
    let convergence_records: Vec<ConvergenceRecord> = convergence_history
        .iter()
        .map(|(iteration, energy, positions)| ConvergenceRecord {
            iteration: *iteration,
            best_energy: *energy,
            best_positions: positions.clone(),
        })
        .collect();
    
    let final_best_energy = convergence_history
        .last()
        .map(|(_, energy, _)| *energy)
        .unwrap_or(f64::INFINITY);
    
    let optimization_results = OptimizationResults {
        timestamp: timestamp.to_string(),
        max_iterations,
        total_evaluations,
        optimization_time_seconds: optimization_time.as_secs_f64(),
        final_best_energy,
        convergence_history: convergence_records,
        field_boundaries: (FIELD_MIN_X, FIELD_MAX_X, FIELD_MIN_Y, FIELD_MAX_Y),
        station_margin: STATION_MARGIN,
        obstacle_margin: OBSTACLE_MARGIN,
    };
    
    let filename = format!("{}optimization_results_{}.json", OPTIMIZATION_RESULTS_PATH, timestamp);
    
    match serde_json::to_string_pretty(&optimization_results) {
        Ok(json_string) => {
            match std::fs::write(&filename, json_string) {
                Ok(_) => {
                    println!("Optimization results saved to: {}", filename);
                    println!("Results summary:");
                    println!("  Total iterations: {}", convergence_history.len());
                    println!("  Total evaluations: {}", total_evaluations);
                    println!("  Optimization time: {:.2} seconds", optimization_time.as_secs_f64());
                    println!("  Final best energy: {:.2} Wh", final_best_energy);
                },
                Err(e) => eprintln!("Failed to write optimization results to file: {}", e),
            }
        },
        Err(e) => eprintln!("Failed to serialize optimization results: {}", e),
    }
}