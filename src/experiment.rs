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
use crate::units::energy::Energy;
use egui::Pos2;
use plotly::{Plot, Scatter, Scatter3D, HeatMap, Layout, common::{Marker, Mode}};
use rand;

pub fn run_experiment() {
    println!("Starting farmbot simulation experiment...");
    
    // Configure environment settings (similar to PerformanceMatrixTool)
    let mut env_config = EnvConfig::default();
    env_config.n_agents = 1;  // Number of agents
    
    // Configure task manager strategies (this is how PerformanceMatrixTool does it)
    env_config.task_manager_config.charging_strategy = ChargingStrategy::CriticalOnly;
    env_config.task_manager_config.choose_station_strategy = ChooseStationStrategy::ClosestManhattan;
    
    // Set scene and agent config paths
    env_config.scene_config_path = DEFAULT_SCENE_CONFIG_PATH.to_string();
    env_config.agent_config_path = DEFAULT_AGENT_CONFIG_PATH.to_string();
    
    // Set datetime config
    env_config.datetime_config = DateTimeConfig::from_string("01.01.2025 08:00:00".to_string());
    
    // Create experiment runner
    let mut runner = ExperimentRunner {
        running: false,
        scene_config_path: env_config.scene_config_path.clone(),
        agent_config_path: env_config.agent_config_path.clone(),
        datetime_config: env_config.datetime_config.clone(),
        env_config,
        termination_condition: TerminationCondition::NumberCompletedTasks(100), // Options: AllTasksCompleted, EnvDuration(Duration::days(1.0)), NumberCompletedTasks(3)
        env: None,
        save_to_file: true,
        save_file_name: "my_experiment_2025".to_string(),
        start_datetime: None,
        start_time: None,
        total_energy_consumed: Energy::ZERO,
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
        step_start_time: crate::units::duration::Duration::ZERO,
    };
    
    println!("Configuration:");
    println!("- Agents: {}", runner.env_config.n_agents);
    println!("- Charging Strategy: {:?}", runner.env_config.task_manager_config.charging_strategy);
    println!("- Station Strategy: {:?}", runner.env_config.task_manager_config.choose_station_strategy);
    println!("- Termination: {:?}", runner.termination_condition);
    println!("- Scene: {}", runner.scene_config_path);
    
    // Run the experiment
    let start = std::time::Instant::now();
    runner.run_simulation();
    let elapsed = start.elapsed();
    
    println!("Experiment completed in {:?}", elapsed);
    println!("Results saved to: {}{}.json", 
        crate::cfg::EXPERIMENTS_PATH, 
        runner.save_file_name);
}

/// Grid search experiment for finding optimal charging station position
pub fn run_grid_search_experiment(grid_resolution: usize) {
    println!("Starting grid search experiment for charging station optimization...");
    println!("Grid resolution: {}x{}", grid_resolution, grid_resolution);
    
    // Load scene configuration to get field boundaries and obstacles
    let scene_config: SceneConfig = crate::utilities::utils::load_json_or_panic(DEFAULT_SCENE_CONFIG_PATH.to_string());
    let field_config: FieldConfig = crate::utilities::utils::load_json_or_panic(scene_config.field_config_path.clone());
    let obstacles = field_config.get_obstacles();
    
    // Define field boundaries (should match those in optimization.rs)
    const FIELD_MIN_X: f32 = 0.0;
    const FIELD_MAX_X: f32 = 12.0;
    const FIELD_MIN_Y: f32 = 0.0;
    const FIELD_MAX_Y: f32 = 12.0;
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
    let mut results: Vec<(Pos2, f64)> = Vec::new();
    let total_points = grid_points.len();
    
    // Run experiment for each grid point
    for (i, grid_point) in grid_points.iter().enumerate() {
        println!("Progress: {}/{} - Testing position ({:.2}, {:.2})", 
                 i + 1, total_points, grid_point.x, grid_point.y);
        
        // Update scene config with new station position
        let energy_consumption = run_single_grid_experiment(*grid_point, &scene_config);
        results.push((*grid_point, energy_consumption));
        
        println!("  → Energy consumption: {:.2} Wh", energy_consumption);
    }
    
    // Save results to file
    let results_file = format!("results/grid_search_{}x{}_results.json", grid_resolution, grid_resolution);
    save_grid_search_results(&results, &results_file);
    
    // Generate plots
    generate_grid_search_plots(&results, &obstacles, grid_resolution);
    
    // Find and report best position
    let best_result = results.iter()
        .min_by(|a, b| a.1.partial_cmp(&b.1).unwrap())
        .unwrap();
    
    println!("\nGrid Search Experiment Completed!");
    println!("Best position: ({:.2}, {:.2})", best_result.0.x, best_result.0.y);
    println!("Best energy consumption: {:.2} Wh", best_result.1);
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
            let point = Pos2::new(x, y);
            
            // Check if point is valid (not too close to obstacles)
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
fn run_single_grid_experiment(station_position: Pos2, original_scene: &SceneConfig) -> f64 {
    // Create a modified scene config with the new station position
    let mut modified_scene = original_scene.clone();
    
    // Update the first (and only) station position
    if !modified_scene.station_configs.is_empty() {
        modified_scene.station_configs[0].pose.position = station_position;
        modified_scene.station_configs[0].update_slots_pose();
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
    env_config.n_agents = 1; // Use single agent for consistency
    env_config.task_manager_config.charging_strategy = ChargingStrategy::CriticalOnly;
    env_config.task_manager_config.choose_station_strategy = ChooseStationStrategy::ClosestManhattan;
    
    // Create and run experiment
    let mut runner = ExperimentRunner {
        running: false,
        scene_config_path: temp_scene_path.clone(),
        agent_config_path: env_config.agent_config_path.clone(),
        datetime_config: env_config.datetime_config.clone(),
        env_config,
        termination_condition: TerminationCondition::NumberCompletedTasks(10000), // Smaller number for grid search
        env: None,
        save_to_file: false, // Don't save individual experiment files
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
        step_start_time: crate::units::duration::Duration::ZERO,
    };
    
    // Run the simulation
    runner.run_simulation();
    
    // Clean up the temporary file
    let _ = std::fs::remove_file(temp_scene_path);
    
    // Return energy consumption
    runner.total_energy_consumed.value as f64
}

/// Save grid search results to JSON file
fn save_grid_search_results(results: &[(Pos2, f64)], filename: &str) {
    use serde_json::{json, Value};
    
    let results_json: Vec<Value> = results.iter()
        .map(|(pos, energy)| {
            json!({
                "position": {
                    "x": pos.x,
                    "y": pos.y
                },
                "energy_consumption": energy
            })
        })
        .collect();
    
    let output = json!({
        "timestamp": chrono::Utc::now().to_rfc3339(),
        "grid_search_results": results_json
    });
    
    // Ensure results directory exists
    std::fs::create_dir_all("results").unwrap();
    
    std::fs::write(filename, serde_json::to_string_pretty(&output).unwrap())
        .expect("Failed to save grid search results");
}

/// Generate visualization plots for grid search results
fn generate_grid_search_plots(results: &[(Pos2, f64)], obstacles: &[Obstacle], grid_resolution: usize) {
    generate_3d_plot(results, obstacles, grid_resolution);
    generate_heatmap_plot(results, obstacles, grid_resolution);
}

/// Generate 3D scatter plot (similar to optimization.rs)
fn generate_3d_plot(results: &[(Pos2, f64)], obstacles: &[Obstacle], grid_resolution: usize) {
    let x_coords: Vec<f64> = results.iter().map(|(pos, _)| pos.x as f64).collect();
    let y_coords: Vec<f64> = results.iter().map(|(pos, _)| pos.y as f64).collect();
    let energy_values: Vec<f64> = results.iter().map(|(_, energy)| *energy).collect();
    
    // Find the optimal (minimum) energy value for obstacle z-level
    let min_energy = energy_values.iter().fold(f64::INFINITY, |a, &b| a.min(b));
    
    let trace = Scatter3D::new(x_coords, y_coords, energy_values)
        .mode(Mode::Markers)
        .marker(Marker::new().size(4).color("blue"));
    
    let mut plot = Plot::new();
    plot.add_trace(trace);
    
    // Add obstacle boundaries as 3D traces
    for (i, obstacle) in obstacles.iter().enumerate() {
        let mut obstacle_x = Vec::new();
        let mut obstacle_y = Vec::new();
        let mut obstacle_z = Vec::new();

        // Add each edge of the obstacle
        for point in &obstacle.points {
            obstacle_x.push(point.x as f64);
            obstacle_y.push(point.y as f64);
            obstacle_z.push(min_energy); // Set z to the minimum energy value
        }

        // Close the polygon by connecting the last point to the first
        if let Some(first_point) = obstacle.points.first() {
            obstacle_x.push(first_point.x as f64);
            obstacle_y.push(first_point.y as f64);
            obstacle_z.push(min_energy);
        }

        // Create a trace for the obstacle
        let obstacle_trace = Scatter3D::new(obstacle_x, obstacle_y, obstacle_z)
            .mode(Mode::Lines)
            .name(&format!("Obstacle {}", i + 1))
            .show_legend(false)
            .line(plotly::common::Line::new().color("black").width(2.0));
        
        plot.add_trace(obstacle_trace);
    }
    
    let layout = Layout::new()
        .title(&format!("Grid Search Results - 3D View ({}x{})", grid_resolution, grid_resolution))
        .width(1200)
        .height(800)
        .x_axis(plotly::layout::Axis::new().title("X Position (m)"))
        .y_axis(plotly::layout::Axis::new().title("Y Position (m)"));
    
    plot.set_layout(layout);
    
    let filename = format!("results/grid_search_{}x{}_3d.html", grid_resolution, grid_resolution);
    plot.write_html(&filename);
    println!("3D plot saved to: {}", filename);
}

/// Generate 2D heatmap plot with interpolation
fn generate_heatmap_plot(results: &[(Pos2, f64)], obstacles: &[Obstacle], grid_resolution: usize) {
    // Create interpolated grid for smooth heatmap
    let interp_resolution = 1000; // Higher resolution for smooth interpolation
    let (x_grid, y_grid, z_grid) = interpolate_results(results, obstacles, interp_resolution);
    
    let heatmap = HeatMap::new(x_grid, y_grid, z_grid);
    
    let mut plot = Plot::new();
    plot.add_trace(heatmap);
    
    // Add obstacle boundaries as traces
    for (i, obstacle) in obstacles.iter().enumerate() {
        let mut obstacle_x = Vec::new();
        let mut obstacle_y = Vec::new();

        // Add each edge of the obstacle
        for point in &obstacle.points {
            obstacle_x.push(point.x as f64);
            obstacle_y.push(point.y as f64);
        }

        // Close the polygon by connecting the last point to the first
        if let Some(first_point) = obstacle.points.first() {
            obstacle_x.push(first_point.x as f64);
            obstacle_y.push(first_point.y as f64);
        }

        // Create a trace for the obstacle
        let obstacle_trace = Scatter::new(obstacle_x, obstacle_y)
            .mode(Mode::Lines)
            .name(&format!("Obstacle {}", i + 1))
            .show_legend(false)
            .line(plotly::common::Line::new().color("black").width(2.0));
        
        plot.add_trace(obstacle_trace);
    }
    
    let layout = Layout::new()
        .title(&format!("Grid Search Results - Heatmap ({}x{})", grid_resolution, grid_resolution))
        .width(1200)
        .height(800)
        .x_axis(plotly::layout::Axis::new().title("X Position (m)"))
        .y_axis(plotly::layout::Axis::new().title("Y Position (m)"));
    
    plot.set_layout(layout);
    
    let filename = format!("results/grid_search_{}x{}_heatmap.html", grid_resolution, grid_resolution);
    plot.write_html(&filename);
    println!("Heatmap saved to: {}", filename);
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
                
                if distance < 0.01 {
                    // Very close to a data point, use exact value
                    z_grid[j][i] = *energy;
                    break;
                } else {
                    let weight = 1.0 / (distance as f64).powi(2);
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

