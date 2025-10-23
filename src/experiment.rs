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
use egui::Pos2;
use rand;
use serde_json;

/// Single experiment result data
#[derive(Debug, Clone)]
pub struct ExperimentResult {
    pub total_energy_consumed: f64,
    pub total_distance_driven: f64,
    pub total_charging_distance: f64,
    pub completed_stationary_tasks: u32,
    pub completed_moving_tasks: u32,
    pub simulation_time_seconds: f64,
}

/// Run a single experiment and return results
pub fn run_experiment() -> ExperimentResult {
    println!("Starting farmbot simulation experiment...");
    
    // Configure environment settings
    let mut env_config = EnvConfig::default();
    env_config.n_agents = 4;
    env_config.task_manager_config.charging_strategy = ChargingStrategy::CriticalOnly;
    env_config.task_manager_config.choose_station_strategy = ChooseStationStrategy::ClosestManhattan;
    env_config.scene_config_path = DEFAULT_SCENE_CONFIG_PATH.to_string();
    env_config.agent_config_path = DEFAULT_AGENT_CONFIG_PATH.to_string();
    env_config.datetime_config = DateTimeConfig::from_string("01.01.2025 08:00:00".to_string());
    
    // Create experiment runner
    let mut runner = ExperimentRunner {
        running: false,
        scene_config_path: env_config.scene_config_path.clone(),
        agent_config_path: env_config.agent_config_path.clone(),
        datetime_config: env_config.datetime_config.clone(),
        env_config,
        termination_condition: TerminationCondition::NumberCompletedTasks(1000),
        env: None,
        save_to_file: true,
        save_file_name: "my_experiment_2025".to_string(),
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
    
    println!("Configuration:");
    println!("- Agents: {}", runner.env_config.n_agents);
    println!("- Charging Strategy: {:?}", runner.env_config.task_manager_config.charging_strategy);
    println!("- Station Strategy: {:?}", runner.env_config.task_manager_config.choose_station_strategy);
    println!("- Termination: {:?}", runner.termination_condition);
    
    // Run the experiment
    let start = std::time::Instant::now();
    runner.run_simulation();
    let elapsed = start.elapsed();
    
    println!("Experiment completed in {:?}", elapsed);
    println!("Results saved to: {}{}.json", 
        crate::cfg::EXPERIMENTS_PATH, 
        runner.save_file_name);
    
    ExperimentResult {
        total_energy_consumed: runner.total_energy_consumed.value as f64,
        total_distance_driven: runner.total_distance_driven as f64,
        total_charging_distance: runner.total_charging_distance as f64,
        completed_stationary_tasks: runner.completed_stationary_tasks,
        completed_moving_tasks: runner.completed_moving_tasks,
        simulation_time_seconds: elapsed.as_secs_f64(),
    }
}

/// Grid search experiment - returns data without plotting
pub fn run_grid_search_experiment_data_only(
    grid_resolution: usize, 
    optimization_minimum: Option<(Pos2, f64)>
) -> crate::visualization::GridSearchResults {
    println!("Starting grid search experiment for charging station optimization...");
    println!("Grid resolution: {}x{}", grid_resolution, grid_resolution);
    
    if let Some((opt_pos, opt_value)) = optimization_minimum {
        println!("Will include optimization minimum at: ({:.2}, {:.2}) with value: {:.2} Wh", 
                 opt_pos.x, opt_pos.y, opt_value);
    }
    
    // Load scene configuration
    let scene_config: SceneConfig = crate::utilities::utils::load_json_or_panic(DEFAULT_SCENE_CONFIG_PATH.to_string());
    let field_config: FieldConfig = crate::utilities::utils::load_json_or_panic(scene_config.field_config_path.clone());
    let obstacles = field_config.get_obstacles();
    
    // Define field boundaries
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
    
    // Run experiment for each grid point
    let mut results: Vec<(Pos2, f64, f64, f64)> = Vec::new();
    let total_points = grid_points.len();
    
    for (i, grid_point) in grid_points.iter().enumerate() {
        println!("Progress: {}/{} - Testing position ({:.2}, {:.2})", 
                 i + 1, total_points, grid_point.x, grid_point.y);
        
        let (energy_consumption, total_distance, charging_distance) = 
            run_single_grid_experiment(*grid_point, &scene_config);
        results.push((*grid_point, energy_consumption, total_distance, charging_distance));
        
        println!("  → Energy: {:.2} Wh, Distance: {:.2} m, Charging dist: {:.2} m", 
                 energy_consumption, total_distance, charging_distance);
    }
    
    // Save results to file with metadata
    let results_file = format!("results/grid_search_{}x{}_results.json", grid_resolution, grid_resolution);
    save_grid_search_results(
        &results, 
        &results_file,
        grid_resolution,
        (FIELD_MIN_X, FIELD_MAX_X, FIELD_MIN_Y, FIELD_MAX_Y),
        optimization_minimum
    );
    
    // Find and report best position
    let best_result = results.iter()
        .min_by(|a, b| a.1.partial_cmp(&b.1).unwrap())
        .unwrap();
    
    println!("\nGrid Search Experiment Completed!");
    println!("Best position: ({:.2}, {:.2})", best_result.0.x, best_result.0.y);
    println!("Best energy consumption: {:.2} Wh", best_result.1);
    println!("Total distance at best position: {:.2} m", best_result.2);
    println!("Charging distance at best position: {:.2} m", best_result.3);
    println!("Results saved to: {}", results_file);
    
    crate::visualization::GridSearchResults {
        results,
        grid_resolution,
        obstacles,
        field_bounds: (FIELD_MIN_X, FIELD_MAX_X, FIELD_MIN_Y, FIELD_MAX_Y),
    }
}

/// Create multi-station demo data for visualization
pub fn create_multi_station_demo_data() -> crate::visualization::MultiStationResults {
    println!("Creating multi-station demo data...");
    
    // Load scene configuration for obstacles and field bounds
    let scene_config: SceneConfig = crate::utilities::utils::load_json_or_panic(DEFAULT_SCENE_CONFIG_PATH.to_string());
    let field_config: FieldConfig = crate::utilities::utils::load_json_or_panic(scene_config.field_config_path.clone());
    let obstacles = field_config.get_obstacles();
    
    const FIELD_MIN_X: f32 = 0.0;
    const FIELD_MAX_X: f32 = 25.0;
    const FIELD_MIN_Y: f32 = 0.0;
    const FIELD_MAX_Y: f32 = 25.0;
    let field_bounds = (FIELD_MIN_X, FIELD_MAX_X, FIELD_MIN_Y, FIELD_MAX_Y);
    
    // Example data (in practice, this would come from actual optimization results)
    let optimal_stations = vec![Pos2::new(11.5, 21.6), Pos2::new(13.6, 6.7)];
    let optimal_energy = 16857.2;
    let optimal_distance = 45553.9;
    
    let suboptimal_configs_energy = vec![
        (vec![Pos2::new(2.0, 2.0), Pos2::new(23.0, 23.0)], 17062.0),
        (vec![Pos2::new(2.0, 12.5), Pos2::new(23.0, 12.5)], 17928.8),
        (vec![Pos2::new(6.25, 2.0), Pos2::new(6.25, 23.0)], 17348.5),
        (vec![Pos2::new(12.5, 7.5), Pos2::new(12.5, 17.5)], 17307.1),
        (vec![Pos2::new(17.5, 2.0), Pos2::new(20.0, 2.0)], 20526.6),
        (vec![Pos2::new(12.5, 11.5), Pos2::new(12.5, 13.5)], 18599.8),
    ];
    
    let suboptimal_configs_distance = vec![
        (vec![Pos2::new(2.0, 2.0), Pos2::new(23.0, 23.0)], 47268.2),
        (vec![Pos2::new(2.0, 12.5), Pos2::new(23.0, 12.5)], 48364.0),
        (vec![Pos2::new(6.25, 2.0), Pos2::new(6.25, 23.0)], 46135.0),
        (vec![Pos2::new(12.5, 7.5), Pos2::new(12.5, 17.5)], 46683.8),
        (vec![Pos2::new(17.5, 2.0), Pos2::new(20.0, 2.0)], 46872.1),
        (vec![Pos2::new(12.5, 11.5), Pos2::new(12.5, 13.5)], 48109.0),
    ];
    
    println!("Demo data created for {} station configurations", suboptimal_configs_energy.len() + 1);
    
    crate::visualization::MultiStationResults {
        optimal_stations,
        optimal_energy,
        optimal_distance,
        suboptimal_configs_energy,
        suboptimal_configs_distance,
        obstacles,
        field_bounds,
    }
}

// Helper functions

/// Round coordinates to 2 decimal places (centimeters)
fn round_to_centimeters(pos: Pos2) -> Pos2 {
    Pos2::new(
        (pos.x * 100.0).round() / 100.0,
        (pos.y * 100.0).round() / 100.0
    )
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
            
            if is_position_valid(point, obstacles, obstacle_margin) {
                valid_points.push(point);
            }
        }
    }
    
    valid_points
}

/// Check if a position is valid (not inside or too close to obstacles)
fn is_position_valid(position: Pos2, obstacles: &[Obstacle], margin: f32) -> bool {
    for obstacle in obstacles {
        if is_point_inside_polygon(position, &obstacle.points) {
            return false;
        }
        
        for window in obstacle.points.windows(2) {
            if let [p1, p2] = window {
                let distance = point_to_line_distance(position, *p1, *p2);
                if distance < margin {
                    return false;
                }
            }
        }
        
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
    
    (
        runner.total_energy_consumed.value as f64,
        runner.total_distance_driven as f64,
        runner.total_charging_distance as f64,
    )
}

/// Save grid search results with metadata to JSON file
fn save_grid_search_results(
    results: &[(Pos2, f64, f64, f64)], 
    filename: &str,
    grid_resolution: usize,
    field_bounds: (f32, f32, f32, f32),
    optimization_minimum: Option<(Pos2, f64)>
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
    
    let mut output = json!({
        "metadata": {
            "grid_resolution": grid_resolution,
            "field_bounds": {
                "min_x": field_bounds.0,
                "max_x": field_bounds.1,
                "min_y": field_bounds.2,
                "max_y": field_bounds.3
            },
            "total_data_points": results.len(),
            "timestamp": chrono::Utc::now().to_rfc3339(),
            "station_margin": 0.4,
            "obstacle_margin": 0.4
        },
        "grid_search_results": results_json
    });
    
    // Add optimization minimum if provided
    if let Some((opt_pos, opt_value)) = optimization_minimum {
        output["metadata"]["optimization_minimum"] = json!({
            "x": opt_pos.x,
            "y": opt_pos.y,
            "energy": opt_value
        });
    }
    
    match std::fs::write(filename, serde_json::to_string_pretty(&output).unwrap()) {
        Ok(_) => println!("Grid search results with metadata saved to: {}", filename),
        Err(e) => eprintln!("Failed to save results: {}", e),
    }
}

// Geometric utility functions

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

fn point_to_line_distance(point: Pos2, line_start: Pos2, line_end: Pos2) -> f32 {
    let line_vec = Pos2::new(line_end.x - line_start.x, line_end.y - line_start.y);
    let point_vec = Pos2::new(point.x - line_start.x, point.y - line_start.y);
    
    let line_len_sq = line_vec.x * line_vec.x + line_vec.y * line_vec.y;
    if line_len_sq == 0.0 {
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