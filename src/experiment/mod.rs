use crate::cfg::{
    DEFAULT_SCENE_CONFIG_PATH, DEFAULT_AGENT_CONFIG_PATH,
};
use crate::tool_module::experiment_tool::{ExperimentRunner, TerminationCondition};
use crate::environment::{
    datetime::DateTimeConfig,
    env_module::env_config::EnvConfig,
    scene_config::SceneConfig,
    field_config::FieldConfig,
    obstacle::Obstacle,
};
use crate::task_module::strategies::{ChargingStrategy, ChooseStationStrategy};
use crate::units::{energy::Energy, duration::Duration};
use crate::experiment::visualization::generate_grid_search_plots;
use egui::Pos2;
use rand;

pub mod runner;
pub mod visualization;

/// Round coordinates to 2 decimal places (centimeters) to prevent floating-point precision issues
fn round_to_centimeters(pos: Pos2) -> Pos2 {
    Pos2::new(
        (pos.x * 100.0).round() / 100.0,
        (pos.y * 100.0).round() / 100.0
    )
}

/// Grid search experiment with optional optimization minimum point and value for visualization
pub fn run_grid_search_experiment(grid_resolution: usize, optimization_minimum: Option<(Pos2, f64)>) {
    println!("Starting grid search experiment for charging station optimization...");
    println!("Grid resolution: {}x{}", grid_resolution, grid_resolution);
    
    if let Some((opt_pos, opt_value)) = optimization_minimum {
        println!("Will visualize optimization minimum at: ({:.2}, {:.2}) with value: {:.2} Wh", 
                 opt_pos.x, opt_pos.y, opt_value);
    }
    
    // Load scene configuration to get field boundaries and obstacles
    let scene_config: SceneConfig = crate::utilities::utils::load_json_or_panic(DEFAULT_SCENE_CONFIG_PATH.to_string());
    let field_config: FieldConfig = crate::utilities::utils::load_json_or_panic(scene_config.field_config_path.clone());
    let obstacles = field_config.get_obstacles();
    
    // Define field boundaries (should match those in optimization.rs)
    const FIELD_MIN_X: f32 = 0.0;
    const FIELD_MAX_X: f32 = 25.0;
    const FIELD_MIN_Y: f32 = 0.0;
    const FIELD_MAX_Y: f32 = 25.0;
    const STATION_MARGIN: f32 = 0.4;
    const OBSTACLE_MARGIN: f32 = 0.4;
    
    // Generate grid points
    let grid_points = generate_valid_grid_points(
        FIELD_MIN_X + STATION_MARGIN,
        FIELD_MAX_X - STATION_MARGIN,
        FIELD_MIN_Y + STATION_MARGIN,
        FIELD_MAX_Y - STATION_MARGIN,
        grid_resolution,
        &obstacles,
        OBSTACLE_MARGIN,
    );
    
    println!("Generated {} valid grid points out of {} total grid points", 
             grid_points.len(), grid_resolution * grid_resolution);
    
    // Store results for analysis
    let mut results: Vec<(Pos2, f64, f64, f64)> = Vec::new(); // (position, energy, total_distance, charging_distance)
    let total_points = grid_points.len();
    
    // Run experiment for each grid point
    for (i, grid_point) in grid_points.iter().enumerate() {
        println!("Progress: {}/{} - Testing position ({:.2}, {:.2})", 
                 i + 1, total_points, grid_point.x, grid_point.y);
        
        // Update scene config with new station position
        let (energy_consumption, total_distance, charging_distance) = run_single_grid_experiment(*grid_point, &scene_config);
        results.push((*grid_point, energy_consumption, total_distance, charging_distance));
        
        println!("  → Energy: {:.2} Wh, Distance: {:.2} m, Charging dist: {:.2} m", 
                 energy_consumption, total_distance, charging_distance);
    }
    
    // Save results to file
    let results_file = format!("results/grid_search_{}x{}_results.json", grid_resolution, grid_resolution);
    save_grid_search_results(&results, &results_file, optimization_minimum, &field_config, grid_resolution);
    
    // Generate plots
    generate_grid_search_plots(&results, &obstacles, grid_resolution, optimization_minimum);
    
    // Find and report best position (based on energy consumption)
    let best_result = results.iter()
        .min_by(|a, b| a.1.partial_cmp(&b.1).unwrap())
        .unwrap();
    
    println!("\nGrid Search Experiment Completed!");
    println!("Best position: ({:.2}, {:.2})", best_result.0.x, best_result.0.y);
    println!("Best energy consumption: {:.2} Wh", best_result.1);
    println!("Total distance at best position: {:.2} m", best_result.2);
    println!("Charging distance at best position: {:.2} m", best_result.3);
    println!("Results saved to: {}", results_file);
}

/// Generate valid grid points that don't intersect with obstacles
fn generate_valid_grid_points(
    min_x: f32,
    max_x: f32,
    min_y: f32,
    max_y: f32,
    resolution: usize,
    obstacles: &[Obstacle],
    obstacle_margin: f32,
) -> Vec<Pos2> {
    let mut valid_points = Vec::new();
    
    let step_x = (max_x - min_x) / (resolution - 1) as f32;
    let step_y = (max_y - min_y) / (resolution - 1) as f32;
    
    for i in 0..resolution {
        for j in 0..resolution {
            let x = min_x + i as f32 * step_x;
            let y = min_y + j as f32 * step_y;
            let point = round_to_centimeters(Pos2::new(x, y));
            
            // Check if point is valid (not too close to obstacles)
            if is_position_valid(point, obstacles, obstacle_margin) {
                valid_points.push(point);
            }
        }
    }
    
    valid_points
}

/// Interpolate results to create smooth heatmap
fn interpolate_results(
    results: &[(Pos2, f64)], 
    obstacles: &[Obstacle], 
    resolution: usize
) -> (Vec<f64>, Vec<f64>, Vec<Vec<f64>>) {
    // Find bounds
    let min_x = results.iter().map(|(pos, _)| pos.x).fold(f32::INFINITY, f32::min);
    let max_x = results.iter().map(|(pos, _)| pos.x).fold(f32::NEG_INFINITY, f32::max);
    let min_y = results.iter().map(|(pos, _)| pos.y).fold(f32::INFINITY, f32::min);
    let max_y = results.iter().map(|(pos, _)| pos.y).fold(f32::NEG_INFINITY, f32::max);
    
    let step_x = (max_x - min_x) / (resolution - 1) as f32;
    let step_y = (max_y - min_y) / (resolution - 1) as f32;
    
    let mut x_grid = Vec::new();
    let mut y_grid = Vec::new();
    let mut z_grid = vec![vec![f64::NAN; resolution]; resolution];
    
    // Create grid coordinates
    for i in 0..resolution {
        x_grid.push((min_x + i as f32 * step_x) as f64);
    }
    for j in 0..resolution {
        y_grid.push((min_y + j as f32 * step_y) as f64);
    }
    
    // Interpolate values using inverse distance weighting
    for (i, &x) in x_grid.iter().enumerate() {
        for (j, &y) in y_grid.iter().enumerate() {
            let point = Pos2::new(x as f32, y as f32);
            
            // Skip points that are not valid charging station positions (same validation as grid generation)
            if !is_position_valid(point, obstacles, 0.4) {
                z_grid[j][i] = f64::NAN; // Will appear as gap in heatmap
                continue;
            }
            
            // Inverse distance weighting interpolation
            let mut weighted_sum = 0.0;
            let mut weight_sum = 0.0;
            
            for (result_pos, energy) in results {
                let distance = ((point.x - result_pos.x).powi(2) + (point.y - result_pos.y).powi(2)).sqrt();
                
                if distance < 0.001 {
                    // Very close to a data point, use exact value
                    z_grid[j][i] = *energy;
                    break;
                } else {
                    let weight = 1.0 / (distance as f64).powi(4);
                    weighted_sum += weight * energy;
                    weight_sum += weight;
                }
            }
            
            if weight_sum > 0.0 && z_grid[j][i].is_nan() {
                z_grid[j][i] = weighted_sum / weight_sum;
            }
        }
    }
    
    (x_grid, y_grid, z_grid)
}

/// Check if a position is valid (not inside or too close to obstacles)
fn is_position_valid(position: Pos2, obstacles: &[Obstacle], margin: f32) -> bool {
    for obstacle in obstacles {
        // Check if position is inside obstacle
        if is_point_inside_polygon(position, &obstacle.points) {
            return false;
        }
        
        // Check if position is too close to any edge of the obstacle
        for window in obstacle.points.windows(2) {
            if let [p1, p2] = window {
                let distance = point_to_line_distance(position, *p1, *p2);
                if distance < margin {
                    return false;
                }
            }
        }
        
        // Check the closing edge (last point to first point)
        if obstacle.points.len() >= 2 {
            let first = obstacle.points[0];
            let last = obstacle.points[obstacle.points.len() - 1];
            let distance = point_to_line_distance(position, last, first);
            if distance < margin {
                return false;
            }
        }
    }
    true
}

/// Run a single experiment with a specific station position
fn run_single_grid_experiment(station_position: Pos2, original_scene: &SceneConfig) -> (f64, f64, f64) {
    // Create a modified scene config with the new station position
    let mut modified_scene = original_scene.clone();
    
    // Update the first (and only) station position
    if !modified_scene.station_configs.is_empty() {
        modified_scene.station_configs[0].pose.position = station_position;
    } else {
        panic!("No charging stations found in scene config");
    }
    
    // Save the modified scene config to a temporary file
    let random_id: u32 = rand::random();
    let temp_scene_path = format!("configs/scene_configs/temp_grid_{}.json", random_id);
    let serialized = serde_json::to_string(&modified_scene).unwrap();
    std::fs::write(&temp_scene_path, &serialized).unwrap();
    
    // Create environment config
    let mut env_config = EnvConfig::default();
    env_config.scene_config_path = temp_scene_path.clone();
    env_config.agent_config_path = DEFAULT_AGENT_CONFIG_PATH.to_string();
    env_config.datetime_config = DateTimeConfig::from_string("01.01.2025 08:00:00".to_string());
    env_config.n_agents = 1;
    env_config.task_manager_config.charging_strategy = ChargingStrategy::CriticalOnly;
    env_config.task_manager_config.choose_station_strategy = ChooseStationStrategy::ClosestManhattan;
    
    // Create and run experiment
    let mut runner = ExperimentRunner {
        running: false,
        scene_config_path: temp_scene_path.clone(),
        agent_config_path: env_config.agent_config_path.clone(),
        datetime_config: env_config.datetime_config.clone(),
        env_config,
        termination_condition: TerminationCondition::NumberCompletedTasks(1000),
        env: None,
        save_to_file: false,
        save_file_name: format!("grid_temp_{}", random_id),
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
    
    // Run the simulation
    runner.run_simulation();
    
    // Clean up the temporary file
    let _ = std::fs::remove_file(temp_scene_path);
    
    // Return energy consumption, total distance driven, and total charging distance
    (
        runner.total_energy_consumed.value as f64,
        runner.total_distance_driven as f64,
        runner.total_charging_distance as f64,
    )

}

/// Save grid search results to JSON file
fn save_grid_search_results(
    results: &[(Pos2, f64, f64, f64)], 
    filename: &str, 
    optimization_minimum: Option<(Pos2, f64)>,
    field_config: &FieldConfig,
    grid_resolution: usize
) {
    use serde_json::{json, Value};
    
    let results_json: Vec<Value> = results.iter()
        .map(|(pos, energy, total_dist, charging_dist)| {
            json!({
                "x": pos.x,
                "y": pos.y,
                "energy_consumption": energy,
                "total_distance": total_dist,
                "charging_distance": charging_dist
            })
        })
        .collect();
    
    // Convert optimization minimum to JSON
    let optimization_minimum_json = optimization_minimum.map(|(pos, value)| {
        json!({
            "x": pos.x,
            "y": pos.y,
            "energy_consumption": value
        })
    });
    
    // Store field config information and reference to original file
    const FIELD_MIN_X: f32 = 0.0;
    const FIELD_MAX_X: f32 = 25.0;
    const FIELD_MIN_Y: f32 = 0.0;
    const FIELD_MAX_Y: f32 = 25.0;
    
    // Load the original field config path from scene config
    let scene_config: SceneConfig = crate::utilities::utils::load_json_or_panic(DEFAULT_SCENE_CONFIG_PATH.to_string());
    let field_config_path = scene_config.field_config_path.clone();
    
    // Read the raw JSON content from the original field config file
    let field_config_raw_json = match std::fs::read_to_string(&field_config_path) {
        Ok(content) => Some(content),
        Err(_) => None,
    };
    
    let field_config_json = json!({
        "field_config_path": field_config_path,
        "field_config_raw": field_config_raw_json,
        "field_boundaries_used_in_grid_search": {
            "min_x": FIELD_MIN_X,
            "max_x": FIELD_MAX_X,
            "min_y": FIELD_MIN_Y,
            "max_y": FIELD_MAX_Y
        },
        "num_obstacles": field_config.get_obstacles().len(),
        "num_field_configs": field_config.configs.len()
    });
    
    let output = json!({
        "grid_search_results": results_json,
        "total_points": results.len(),
        "grid_resolution": grid_resolution,
        "optimization_minimum": optimization_minimum_json,
        "field_config": field_config_json,
        "generated_at": chrono::Utc::now().to_rfc3339()
    });
    
    match std::fs::write(filename, serde_json::to_string_pretty(&output).unwrap()) {
        Ok(_) => println!("Grid search results saved to: {}", filename),
        Err(e) => eprintln!("Failed to save results: {}", e),
    }
}

// Helper functions (copied from optimization.rs to avoid duplication)

/// Ray casting algorithm to determine if point is inside polygon
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

/// Calculate distance from point to line segment
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

