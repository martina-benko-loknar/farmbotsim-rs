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
use plotly::{Plot, Scatter, Scatter3D, HeatMap, Layout, common::{Marker, Mode}, ImageFormat};
use rand;

pub fn run_experiment() {
    println!("Starting farmbot simulation experiment...");
    
    // Configure environment settings (similar to PerformanceMatrixTool)
    let mut env_config = EnvConfig::default();
    env_config.n_agents = 4;  // Number of agents
    
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
        termination_condition: TerminationCondition::NumberCompletedTasks(1000), // Options: AllTasksCompleted, EnvDuration(Duration::days(1.0)), NumberCompletedTasks(3)
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
    save_grid_search_results(&results, &results_file);
    
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
fn run_single_grid_experiment(station_position: Pos2, original_scene: &SceneConfig) -> (f64, f64, f64) {
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
        step_start_time: crate::units::duration::Duration::ZERO,
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
fn save_grid_search_results(results: &[(Pos2, f64, f64, f64)], filename: &str) {
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
    
    let output = json!({
        "grid_search_results": results_json,
        "total_points": results.len()
    });
    
    match std::fs::write(filename, serde_json::to_string_pretty(&output).unwrap()) {
        Ok(_) => println!("Grid search results saved to: {}", filename),
        Err(e) => eprintln!("Failed to save results: {}", e),
    }
}

/// Generate visualization plots for grid search results
fn generate_grid_search_plots(results: &[(Pos2, f64, f64, f64)], obstacles: &[Obstacle], grid_resolution: usize, optimization_minimum: Option<(Pos2, f64)>) {
    generate_3d_plot(results, obstacles, grid_resolution, optimization_minimum);
    generate_energy_heatmap_plot(results, obstacles, grid_resolution, optimization_minimum);
    generate_distance_heatmap_plot(results, obstacles, grid_resolution, optimization_minimum);
    generate_charging_distance_heatmap_plot(results, obstacles, grid_resolution, optimization_minimum);
}

/// Generate 3D scatter plot (similar to optimization.rs)
fn generate_3d_plot(results: &[(Pos2, f64, f64, f64)], obstacles: &[Obstacle], grid_resolution: usize, optimization_minimum: Option<(Pos2, f64)>) {
    let x_coords: Vec<f64> = results.iter().map(|(pos, _, _, _)| pos.x as f64).collect();
    let y_coords: Vec<f64> = results.iter().map(|(pos, _, _, _)| pos.y as f64).collect();
    let energy_values: Vec<f64> = results.iter().map(|(_, energy, _, _)| *energy).collect();
    
    // Find the optimal (minimum) energy value for obstacle z-level
    let min_energy = energy_values.iter().fold(f64::INFINITY, |a, &b| a.min(b));
    
    let trace = Scatter3D::new(x_coords, y_coords, energy_values)
        .mode(Mode::Markers)
        .marker(Marker::new().size(4).color("blue"));
    
    let mut plot = Plot::new();
    plot.add_trace(trace);
    
    // Add optimization minimum point if provided (using the actual energy value from optimization)
    if let Some((opt_pos, opt_value)) = optimization_minimum {
        let opt_trace = Scatter3D::new(
            vec![opt_pos.x as f64], 
            vec![opt_pos.y as f64], 
            vec![opt_value]
        )
        .mode(Mode::Markers)
        .name("Optimization Minimum")
        .marker(Marker::new().size(12).color("red"));
        
        plot.add_trace(opt_trace);
    }
    
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
        .x_axis(plotly::layout::Axis::new()
            .title("X Position (m)")
            .tick_font(plotly::common::Font::new().size(14)))
        .y_axis(plotly::layout::Axis::new()
            .title("Y Position (m)")
            .tick_font(plotly::common::Font::new().size(14)))
        .legend(plotly::layout::Legend::new()
            .font(plotly::common::Font::new().size(14)));
    
    plot.set_layout(layout);
    
    let filename_html = format!("results/grid_search_{}x{}_3d.html", grid_resolution, grid_resolution);
    let filename_svg = format!("results/grid_search_{}x{}_3d.svg", grid_resolution, grid_resolution);
    
    plot.write_html(&filename_html);
    if let Err(e) = plot.write_image(&filename_svg, ImageFormat::SVG, 800, 600, 1.0) {
        eprintln!("Failed to write SVG file: {}", e);
        println!("3D plot saved to: {}", filename_html);
    } else {
        println!("3D plot saved to: {} and {}", filename_html, filename_svg);
    }
}

/// Generate 2D heatmap plot for energy consumption with interpolation
fn generate_energy_heatmap_plot(results: &[(Pos2, f64, f64, f64)], obstacles: &[Obstacle], grid_resolution: usize, optimization_minimum: Option<(Pos2, f64)>) {
    // Extract energy data for heatmap
    let energy_results: Vec<(Pos2, f64)> = results.iter()
        .map(|(pos, energy, _, _)| (*pos, *energy))
        .collect();
    
    // Create interpolated grid for smooth heatmap
    let interp_resolution = 1000; // Higher resolution for smooth interpolation
    let (x_grid, y_grid, z_grid) = interpolate_results(&energy_results, obstacles, interp_resolution);
    
    let heatmap = HeatMap::new(x_grid, y_grid, z_grid)
        .color_bar(
            plotly::common::ColorBar::new()
                .title("Energy<br>Consumption<br>(Wh)")
                .tick_font(plotly::common::Font::new().size(14))
        );
    
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
    
    // Add optimization minimum point if provided (using black marker for better visibility)
    if let Some((opt_pos, _opt_value)) = optimization_minimum {
        let opt_trace = Scatter::new(vec![opt_pos.x as f64], vec![opt_pos.y as f64])
            .mode(Mode::Markers)
            .name("Optimization Minimum")
            .show_legend(false)  // Don't show in legend
            .marker(Marker::new().size(15).color("black"));
        
        plot.add_trace(opt_trace);
        
        // Add text annotation above the marker
        let annotation = plotly::layout::Annotation::new()
            .x(opt_pos.x as f64)
            .y(opt_pos.y as f64 + 1.0) 
            .text(&format!("Minimum"))
            .show_arrow(false)
            .font(plotly::common::Font::new().size(18).color("black"));

        let layout = Layout::new()
            .title(&format!("Grid Search Results - Energy Consumption Heatmap ({}x{})", grid_resolution, grid_resolution))
            .width(860)
            .height(800)
            .font(plotly::common::Font::new().size(16).color("black"))
            .x_axis(plotly::layout::Axis::new()
                .title("X Position (m)")
                .tick_font(plotly::common::Font::new().size(16)))
            .y_axis(plotly::layout::Axis::new()
                .title("Y Position (m)")
                .tick_font(plotly::common::Font::new().size(16)))
            .annotations(vec![annotation]);
        
        plot.set_layout(layout);

    } else {
        let layout = Layout::new()
                .title(&format!("Grid Search Results - Energy Consumption Heatmap ({}x{})", grid_resolution, grid_resolution))
                .width(860)
                .height(800)
                .font(plotly::common::Font::new().size(16).color("black"))
                .x_axis(plotly::layout::Axis::new()
                    .title("X Position (m)")
                    .tick_font(plotly::common::Font::new().size(16)))
                .y_axis(plotly::layout::Axis::new()
                    .title("Y Position (m)")
                    .tick_font(plotly::common::Font::new().size(16)));
            
            plot.set_layout(layout);
    }    
    
    let filename_html = format!("results/grid_search_{}x{}_energy_heatmap.html", grid_resolution, grid_resolution);
    let filename_svg = format!("results/grid_search_{}x{}_energy_heatmap.svg", grid_resolution, grid_resolution);
    
    plot.write_html(&filename_html);
    if let Err(e) = plot.write_image(&filename_svg, ImageFormat::SVG, 860, 800, 1.0) {
        eprintln!("Failed to write SVG file: {}", e);
        println!("Energy heatmap saved to: {}", filename_html);
    } else {
        println!("Energy heatmap saved to: {} and {}", filename_html, filename_svg);
    }
}

/// Generate 2D heatmap plot for total distance driven with interpolation
fn generate_distance_heatmap_plot(results: &[(Pos2, f64, f64, f64)], obstacles: &[Obstacle], grid_resolution: usize, optimization_minimum: Option<(Pos2, f64)>) {
    // Extract distance data for heatmap
    let distance_results: Vec<(Pos2, f64)> = results.iter()
        .map(|(pos, _, total_distance, _)| (*pos, *total_distance))
        .collect();
    
    // Create interpolated grid for smooth heatmap
    let interp_resolution = 1000; // Higher resolution for smooth interpolation
    let (x_grid, y_grid, z_grid) = interpolate_results(&distance_results, obstacles, interp_resolution);
    
    let heatmap = HeatMap::new(x_grid, y_grid, z_grid)
        .color_bar(
            plotly::common::ColorBar::new()
                .title("Total<br>Distance<br>Driven<br>(m)")
                .tick_font(plotly::common::Font::new().size(14))
        );
    
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
    
    // Add optimization minimum point if provided (using black marker for better visibility)
    if let Some((opt_pos, _opt_value)) = optimization_minimum {
        let opt_trace = Scatter::new(vec![opt_pos.x as f64], vec![opt_pos.y as f64])
            .mode(Mode::Markers)
            .name("Energy Consumption Optimization Minimum")
            .show_legend(false)  // Don't show in legend
            .marker(Marker::new().size(15).color("black"));
        
        plot.add_trace(opt_trace);
        
        // Add text annotation above the marker
        let annotation = plotly::layout::Annotation::new()
            .x(opt_pos.x as f64)
            .y(opt_pos.y as f64 + 1.0)  // Position text slightly above the marker
            .text(&format!("Minimum"))
            .show_arrow(false)
            .font(plotly::common::Font::new().size(18).color("black"));

        let layout = Layout::new()
            .title(&format!("Grid Search Results - Total Distance Heatmap ({}x{})", grid_resolution, grid_resolution))
            .width(820)
            .height(800)
            .font(plotly::common::Font::new().size(16).color("black"))
            .x_axis(plotly::layout::Axis::new()
                .title("X Position (m)")
                .tick_font(plotly::common::Font::new().size(16)))
            .y_axis(plotly::layout::Axis::new()
                .title("Y Position (m)")
                .tick_font(plotly::common::Font::new().size(16)))
            .annotations(vec![annotation]);
        
        plot.set_layout(layout);

    } else {
        let layout = Layout::new()
                .title(&format!("Grid Search Results - Total Distance Heatmap ({}x{})", grid_resolution, grid_resolution))
                .width(820)
                .height(800)
                .font(plotly::common::Font::new().size(16).color("black"))
                .x_axis(plotly::layout::Axis::new()
                    .title("X Position (m)")
                    .tick_font(plotly::common::Font::new().size(16)))
                .y_axis(plotly::layout::Axis::new()
                    .title("Y Position (m)")
                    .tick_font(plotly::common::Font::new().size(16)));
            
            plot.set_layout(layout);
    }    
    
    let filename_html = format!("results/grid_search_{}x{}_distance_heatmap.html", grid_resolution, grid_resolution);
    let filename_svg = format!("results/grid_search_{}x{}_distance_heatmap.svg", grid_resolution, grid_resolution);
    
    plot.write_html(&filename_html);
    if let Err(e) = plot.write_image(&filename_svg, ImageFormat::SVG, 860, 800, 1.0) {
        eprintln!("Failed to write SVG file: {}", e);
        println!("Total distance heatmap saved to: {}", filename_html);
    } else {
        println!("Total distance heatmap saved to: {} and {}", filename_html, filename_svg);
    }
}

/// Generate 2D heatmap plot for charging distance with interpolation
fn generate_charging_distance_heatmap_plot(results: &[(Pos2, f64, f64, f64)], obstacles: &[Obstacle], grid_resolution: usize, optimization_minimum: Option<(Pos2, f64)>) {
    // Extract charging distance data for heatmap
    let charging_distance_results: Vec<(Pos2, f64)> = results.iter()
        .map(|(pos, _, _, charging_distance)| (*pos, *charging_distance))
        .collect();
    
    // Create interpolated grid for smooth heatmap
    let interp_resolution = 1000; // Higher resolution for smooth interpolation
    let (x_grid, y_grid, z_grid) = interpolate_results(&charging_distance_results, obstacles, interp_resolution);
    
    let heatmap = HeatMap::new(x_grid, y_grid, z_grid)
        .color_bar(
            plotly::common::ColorBar::new()
                .title("Total<br>Charging<br>Distance<br>(m)")
                .tick_font(plotly::common::Font::new().size(14))
        );
    
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
    
    // Add optimization minimum point if provided (using black marker for better visibility)
    if let Some((opt_pos, _opt_value)) = optimization_minimum {
        let opt_trace = Scatter::new(vec![opt_pos.x as f64], vec![opt_pos.y as f64])
            .mode(Mode::Markers)
            .name("Energy Consumption Optimization Minimum")
            .show_legend(false)  // Don't show in legend
            .marker(Marker::new().size(15).color("black"));
        
        plot.add_trace(opt_trace);
        
        // Add text annotation above the marker
        let annotation = plotly::layout::Annotation::new()
            .x(opt_pos.x as f64)
            .y(opt_pos.y as f64 + 1.0)  // Position text slightly above the marker
            .text(&format!("Minimum"))
            .show_arrow(false)
            .font(plotly::common::Font::new().size(18).color("black"));

        let layout = Layout::new()
            .title(&format!("Grid Search Results - Charging Distance Heatmap ({}x{})", grid_resolution, grid_resolution))
            .width(820)
            .height(800)
            .font(plotly::common::Font::new().size(16).color("black"))
            .x_axis(plotly::layout::Axis::new()
                .title("X Position (m)")
                .tick_font(plotly::common::Font::new().size(16)))
            .y_axis(plotly::layout::Axis::new()
                .title("Y Position (m)")
                .tick_font(plotly::common::Font::new().size(16)))
            .annotations(vec![annotation]);
        
        plot.set_layout(layout);

    } else {
        let layout = Layout::new()
                .title(&format!("Grid Search Results - Charging Distance Heatmap ({}x{})", grid_resolution, grid_resolution))
                .width(820)
                .height(800)
                .font(plotly::common::Font::new().size(16).color("black"))
                .x_axis(plotly::layout::Axis::new()
                    .title("X Position (m)")
                    .tick_font(plotly::common::Font::new().size(16)))
                .y_axis(plotly::layout::Axis::new()
                    .title("Y Position (m)")
                    .tick_font(plotly::common::Font::new().size(16)));
            
            plot.set_layout(layout);
    }    
    
    let filename_html = format!("results/grid_search_{}x{}_charging_distance_heatmap.html", grid_resolution, grid_resolution);
    let filename_svg = format!("results/grid_search_{}x{}_charging_distance_heatmap.svg", grid_resolution, grid_resolution);
    
    plot.write_html(&filename_html);
    if let Err(e) = plot.write_image(&filename_svg, ImageFormat::SVG, 860, 800, 1.0) {
        eprintln!("Failed to write SVG file: {}", e);
        println!("Charging distance heatmap saved to: {}", filename_html);
    } else {
        println!("Charging distance heatmap saved to: {} and {}", filename_html, filename_svg);
    }
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

/// Generate 2D multi-station visualization with heatmap-style appearance
pub fn generate_multi_station_plot(
    optimal_stations: &[Pos2],
    optimal_energy: f64,
    suboptimal_configs: &[(Vec<Pos2>, f64)], // Up to 4-5 suboptimal configurations
    obstacles: &[Obstacle],
    field_bounds: (f32, f32, f32, f32), // (min_x, max_x, min_y, max_y)
) {
    let mut plot = Plot::new();
    
    // Collect all energy values for ranking
    let mut all_configs = vec![(optimal_stations.to_vec(), optimal_energy)];
    for (stations, energy) in suboptimal_configs {
        all_configs.push((stations.clone(), *energy));
    }
    
    // Sort by energy (ascending) to determine color ranking
    let mut energy_rank: Vec<(usize, f64)> = all_configs.iter().enumerate()
        .map(|(i, (_, energy))| (i, *energy))
        .collect();
    energy_rank.sort_by(|a, b| a.1.partial_cmp(&b.1).unwrap());
    
    // Heatmap-style color scheme (blue to red)
    let colors = [
        "#321decff",  "#5754f7ff", "#adb3ffff", "#f0c49cff", "#f3a172ff", "#f3584dff", "#ee2e2eff"
    ];
    
    // Add optimal configuration (first in sorted list)
    let opt_x: Vec<f64> = optimal_stations.iter().map(|p| p.x as f64).collect();
    let opt_y: Vec<f64> = optimal_stations.iter().map(|p| p.y as f64).collect();
    let optimal_rank = energy_rank.iter().position(|(i, _)| *i == 0).unwrap_or(0);
    let optimal_color_idx = optimal_rank.min(colors.len() - 1);
    
    let optimal_trace = Scatter::new(opt_x.clone(), opt_y.clone())
        .mode(Mode::Markers)
        .name(&format!("Optimal ({:.1} Wh)", optimal_energy))
        .marker(Marker::new()
            .size(18)
            .color(colors[optimal_color_idx])
            .line(plotly::common::Line::new().width(3.0).color("black"))
            .symbol(plotly::common::MarkerSymbol::Star)); // Always use star for optimal
    
    plot.add_trace(optimal_trace);
    
    
    // Add suboptimal configurations
    for (i, (stations, energy)) in suboptimal_configs.iter().enumerate() {
        let x_coords: Vec<f64> = stations.iter().map(|p| p.x as f64).collect();
        let y_coords: Vec<f64> = stations.iter().map(|p| p.y as f64).collect();
        
        let config_rank = energy_rank.iter().position(|(idx, _)| *idx == i + 1).unwrap_or(colors.len() - 1);
        let color_idx = config_rank.min(colors.len() - 1);
        
        // Use different marker shape for each config (cycle through available shapes)
        let marker_symbol = match i {
            0 => plotly::common::MarkerSymbol::Circle,
            1 => plotly::common::MarkerSymbol::Square,
            2 => plotly::common::MarkerSymbol::Diamond,
            3 => plotly::common::MarkerSymbol::Cross,
            4 => plotly::common::MarkerSymbol::X,
            _ => plotly::common::MarkerSymbol::Hexagon,
        };
        
        let trace = Scatter::new(x_coords.clone(), y_coords.clone())
            .mode(Mode::Markers)
            .name(&format!("Config {} ({:.1} Wh)", i + 1, energy))
            .marker(Marker::new()
                .size(14)
                .color(colors[color_idx])
                .line(plotly::common::Line::new().width(2.0).color("black"))
                .symbol(marker_symbol));
        
        plot.add_trace(trace);
    }
    
    // Add obstacles
    for (i, obstacle) in obstacles.iter().enumerate() {
        let mut obstacle_x = Vec::new();
        let mut obstacle_y = Vec::new();

        for point in &obstacle.points {
            obstacle_x.push(point.x as f64);
            obstacle_y.push(point.y as f64);
        }

        // Close the polygon
        if let Some(first_point) = obstacle.points.first() {
            obstacle_x.push(first_point.x as f64);
            obstacle_y.push(first_point.y as f64);
        }

        let obstacle_trace = Scatter::new(obstacle_x, obstacle_y)
            .mode(Mode::Lines)
            .name(&format!("Obstacle {}", i + 1))
            .show_legend(false)
            .line(plotly::common::Line::new().color("black").width(2.0));

        
        plot.add_trace(obstacle_trace);
    }
    
    // Add field boundaries as visible lines
    let boundary_x = vec![
        field_bounds.0 as f64, field_bounds.1 as f64, field_bounds.1 as f64, 
        field_bounds.0 as f64, field_bounds.0 as f64
    ];
    let boundary_y = vec![
        field_bounds.2 as f64, field_bounds.2 as f64, field_bounds.3 as f64, 
        field_bounds.3 as f64, field_bounds.2 as f64
    ];
    
    let boundary_trace = Scatter::new(boundary_x, boundary_y)
        .mode(Mode::Lines)
        .name("Field Boundary")
        .show_legend(false)
        .line(plotly::common::Line::new().color("black").width(2.0));
    
    plot.add_trace(boundary_trace);
    
    // Create layout matching heatmap style
    let layout = Layout::new()
        .title("Charging Station Position Optimization Results")
        .width(890)  // Match heatmap size
        .height(800)
        .font(plotly::common::Font::new().size(16).color("black"))
        .x_axis(plotly::layout::Axis::new()
            .title("X Position (m)")
            .range(vec![field_bounds.0 as f64, field_bounds.1 as f64])
            .auto_range(false)  // Disable auto range to use exact range
            .tick_font(plotly::common::Font::new().size(16))
            .show_grid(false)  // Remove grid
            .show_line(true)   // Show axis line
            .line_color("black")
            .line_width(1)
            .zero_line(false)  // Disable zero line
            .ticks(plotly::layout::TicksDirection::Outside)  // Show tick marks outside
            .tick_length(5)    // Set tick mark length
            .tick_width(1)     // Set tick mark width
            .tick_color("black"))  // Set tick mark color
        .y_axis(plotly::layout::Axis::new()
            .title("Y Position (m)")
            .range(vec![field_bounds.2 as f64, field_bounds.3 as f64])
            .auto_range(false)  // Disable auto range to use exact range
            .tick_font(plotly::common::Font::new().size(16))
            .show_grid(false)  // Remove grid
            .show_line(true)   // Show axis line
            .line_color("black")
            .line_width(1)
            .zero_line(false)  // Disable zero line
            .ticks(plotly::layout::TicksDirection::Outside)  // Show tick marks outside
            .tick_length(5)    // Set tick mark length
            .tick_width(1)     // Set tick mark width
            .tick_color("black"))  // Set tick mark color
        .legend(plotly::layout::Legend::new()
            .x(1.02)
            .y(1.0)
            .font(plotly::common::Font::new().size(16)))
        .plot_background_color("white")
        .paper_background_color("white")
        .show_legend(true);
    
    plot.set_layout(layout);
    
    // Save plot
    let timestamp = chrono::Utc::now().format("%Y%m%d_%H%M%S");
    let filename_html = format!("results/multi_station_optimization_energy_{}.html", timestamp);
    let filename_svg = format!("results/multi_station_optimization_energy_{}.svg", timestamp);

    plot.write_html(&filename_html);
    if let Err(e) = plot.write_image(&filename_svg, ImageFormat::SVG, 890, 800, 1.0) {
        eprintln!("Failed to write SVG file: {}", e);
        println!("Multi-station energy plot saved to: {}", filename_html);
    } else {
        println!("Multi-station energy plot saved to: {} and {}", filename_html, filename_svg);
    }
}

/// Generate 2D multi-station visualization focused on total traveling distance
pub fn generate_multi_station_distance_plot(
    optimal_stations: &[Pos2],
    optimal_distance: f64,
    suboptimal_configs: &[(Vec<Pos2>, f64)], // Station positions with their total distances
    obstacles: &[Obstacle],
    field_bounds: (f32, f32, f32, f32), // (min_x, max_x, min_y, max_y)
) {
    let mut plot = Plot::new();
    
    // Collect all distance values for ranking
    let mut all_configs = vec![(optimal_stations.to_vec(), optimal_distance)];
    for (stations, distance) in suboptimal_configs {
        all_configs.push((stations.clone(), *distance));
    }
    
    // Sort by distance (ascending) to determine color ranking
    let mut distance_rank: Vec<(usize, f64)> = all_configs.iter().enumerate()
        .map(|(i, (_, distance))| (i, *distance))
        .collect();
    distance_rank.sort_by(|a, b| a.1.partial_cmp(&b.1).unwrap());
    
    // Heatmap-style color scheme (blue to red) - blue for lower distances, red for higher
    let colors = [
        "#321decff",  "#5754f7ff", "#adb3ffff", "#f0c49cff", "#f3a172ff", "#f3584dff", "#ee2e2eff"
    ];
    
    // Add optimal configuration (first in sorted list)
    let opt_x: Vec<f64> = optimal_stations.iter().map(|p| p.x as f64).collect();
    let opt_y: Vec<f64> = optimal_stations.iter().map(|p| p.y as f64).collect();
    let optimal_rank = distance_rank.iter().position(|(i, _)| *i == 0).unwrap_or(0);
    let optimal_color_idx = optimal_rank.min(colors.len() - 1);
    
    let optimal_trace = Scatter::new(opt_x.clone(), opt_y.clone())
        .mode(Mode::Markers)
        .name(&format!("Optimal ({:.1} m)", optimal_distance))
        .marker(Marker::new()
            .size(18)
            .color(colors[optimal_color_idx])
            .line(plotly::common::Line::new().width(3.0).color("black"))
            .symbol(plotly::common::MarkerSymbol::Star)); // Always use star for optimal
    
    plot.add_trace(optimal_trace);
    
    // Add suboptimal configurations
    for (i, (stations, distance)) in suboptimal_configs.iter().enumerate() {
        let x_coords: Vec<f64> = stations.iter().map(|p| p.x as f64).collect();
        let y_coords: Vec<f64> = stations.iter().map(|p| p.y as f64).collect();
        
        let config_rank = distance_rank.iter().position(|(idx, _)| *idx == i + 1).unwrap_or(colors.len() - 1);
        let color_idx = config_rank.min(colors.len() - 1);
        
        // Use different marker shape for each config (cycle through available shapes)
        let marker_symbol = match i {
            0 => plotly::common::MarkerSymbol::Circle,
            1 => plotly::common::MarkerSymbol::Square,
            2 => plotly::common::MarkerSymbol::Diamond,
            3 => plotly::common::MarkerSymbol::Cross,
            4 => plotly::common::MarkerSymbol::X,
            _ => plotly::common::MarkerSymbol::Hexagon,
        };
        
        let trace = Scatter::new(x_coords.clone(), y_coords.clone())
            .mode(Mode::Markers)
            .name(&format!("Config {} ({:.1} m)", i + 1, distance))
            .marker(Marker::new()
                .size(14)
                .color(colors[color_idx])
                .line(plotly::common::Line::new().width(2.0).color("black"))
                .symbol(marker_symbol));
        
        plot.add_trace(trace);
    }
    
    // Add obstacles
    for (i, obstacle) in obstacles.iter().enumerate() {
        let mut obstacle_x = Vec::new();
        let mut obstacle_y = Vec::new();

        for point in &obstacle.points {
            obstacle_x.push(point.x as f64);
            obstacle_y.push(point.y as f64);
        }

        // Close the polygon
        if let Some(first_point) = obstacle.points.first() {
            obstacle_x.push(first_point.x as f64);
            obstacle_y.push(first_point.y as f64);
        }

        let obstacle_trace = Scatter::new(obstacle_x, obstacle_y)
            .mode(Mode::Lines)
            .name(&format!("Obstacle {}", i + 1))
            .show_legend(false)
            .line(plotly::common::Line::new().color("black").width(2.0));
        
        plot.add_trace(obstacle_trace);
    }
    
    // Add field boundaries as visible lines
    let boundary_x = vec![
        field_bounds.0 as f64, field_bounds.1 as f64, field_bounds.1 as f64, 
        field_bounds.0 as f64, field_bounds.0 as f64
    ];
    let boundary_y = vec![
        field_bounds.2 as f64, field_bounds.2 as f64, field_bounds.3 as f64, 
        field_bounds.3 as f64, field_bounds.2 as f64
    ];
    
    let boundary_trace = Scatter::new(boundary_x, boundary_y)
        .mode(Mode::Lines)
        .name("Field Boundary")
        .show_legend(false)
        .line(plotly::common::Line::new().color("black").width(2.0));
    
    plot.add_trace(boundary_trace);
    
    // Create layout matching heatmap style
    let layout = Layout::new()
        .title("Charging Station Position Optimization Results - Total Traveling Distance")
        .width(890)  // Match heatmap size
        .height(800)
        .font(plotly::common::Font::new().size(16).color("black"))
        .x_axis(plotly::layout::Axis::new()
            .title("X Position (m)")
            .range(vec![field_bounds.0 as f64, field_bounds.1 as f64])
            .auto_range(false)  // Disable auto range to use exact range
            .tick_font(plotly::common::Font::new().size(16))
            .show_grid(false)  // Remove grid
            .show_line(true)   // Show axis line
            .line_color("black")
            .line_width(1)
            .zero_line(false)  // Disable zero line
            .ticks(plotly::layout::TicksDirection::Outside)  // Show tick marks outside
            .tick_length(5)    // Set tick mark length
            .tick_width(1)     // Set tick mark width
            .tick_color("black"))  // Set tick mark color
        .y_axis(plotly::layout::Axis::new()
            .title("Y Position (m)")
            .range(vec![field_bounds.2 as f64, field_bounds.3 as f64])
            .auto_range(false)  // Disable auto range to use exact range
            .tick_font(plotly::common::Font::new().size(16))
            .show_grid(false)  // Remove grid
            .show_line(true)   // Show axis line
            .line_color("black")
            .line_width(1)
            .zero_line(false)  // Disable zero line
            .ticks(plotly::layout::TicksDirection::Outside)  // Show tick marks outside
            .tick_length(5)    // Set tick mark length
            .tick_width(1)     // Set tick mark width
            .tick_color("black"))  // Set tick mark color
        .legend(plotly::layout::Legend::new()
            .x(1.02)
            .y(1.0)
            .font(plotly::common::Font::new().size(16)))
        .plot_background_color("white")
        .paper_background_color("white")
        .show_legend(true);
    
    plot.set_layout(layout);
    
    // Save plot
    let timestamp = chrono::Utc::now().format("%Y%m%d_%H%M%S");
    let filename_html = format!("results/multi_station_distance_{}.html", timestamp);
    let filename_svg = format!("results/multi_station_distance_{}.svg", timestamp);
    
    plot.write_html(&filename_html);
    if let Err(e) = plot.write_image(&filename_svg, ImageFormat::SVG, 890, 800, 1.0) {
        eprintln!("Failed to write SVG file: {}", e);
        println!("Multi-station distance plot saved to: {}", filename_html);
    } else {
        println!("Multi-station distance plot saved to: {} and {}", filename_html, filename_svg);
    }
}



/// Example function demonstrating how to generate plots for multi-station configurations
pub fn multi_station_plot_function() {
    use egui::Pos2;
    
    // Load scene configuration to get field boundaries and obstacles (same as other plot functions)
    let scene_config: SceneConfig = crate::utilities::utils::load_json_or_panic(DEFAULT_SCENE_CONFIG_PATH.to_string());
    let field_config: FieldConfig = crate::utilities::utils::load_json_or_panic(scene_config.field_config_path.clone());
    let obstacles = field_config.get_obstacles();
    
    // Define field boundaries (same as used in grid search experiments)
    const FIELD_MIN_X: f32 = 0.0;
    const FIELD_MAX_X: f32 = 25.0;
    const FIELD_MIN_Y: f32 = 0.0;
    const FIELD_MAX_Y: f32 = 25.0;
    let field_bounds = (FIELD_MIN_X, FIELD_MAX_X, FIELD_MIN_Y, FIELD_MAX_Y);
    
    // Example optimal and suboptimal station configurations (scaled to field dimensions)
    let optimal_stations = vec![Pos2::new(11.5, 21.6), Pos2::new(13.6, 6.7)];
    let optimal_energy = 16857.2;
    let optimal_distance = 45553.9; // Example optimal total traveling distance
    
    let suboptimal_configs_energy = vec![
        (vec![Pos2::new(2.0, 2.0), Pos2::new(23.0, 23.0)], 17062.0),
        (vec![Pos2::new(2.0, 12.5), Pos2::new(23.0, 12.5)], 17928.8),
        (vec![Pos2::new(6.25, 2.0), Pos2::new(6.25, 23.0)], 17348.5),
        (vec![Pos2::new(12.5, 7.5), Pos2::new(12.5, 17.5)], 17307.1),
        (vec![Pos2::new(17.5, 2.0), Pos2::new(20.0, 2.0)], 20526.6),
        (vec![Pos2::new(12.5, 11.5), Pos2::new(12.5, 13.5)], 18599.8),
    ];
    
    // Example distance data for the same configurations (in meters)
    let suboptimal_configs_distance = vec![
        (vec![Pos2::new(2.0, 2.0), Pos2::new(23.0, 23.0)], 47268.2),
        (vec![Pos2::new(2.0, 12.5), Pos2::new(23.0, 12.5)], 48364.0),
        (vec![Pos2::new(6.25, 2.0), Pos2::new(6.25, 23.0)], 46135.0),
        (vec![Pos2::new(12.5, 7.5), Pos2::new(12.5, 17.5)], 46683.8),
        (vec![Pos2::new(17.5, 2.0), Pos2::new(20.0, 2.0)], 46872.1),
        (vec![Pos2::new(12.5, 11.5), Pos2::new(12.5, 13.5)], 48109.0),
    ];
    
    // Generate energy-focused plot
    generate_multi_station_plot(
        &optimal_stations,
        optimal_energy,
        &suboptimal_configs_energy,
        &obstacles,
        field_bounds,
    );
    
    // Generate distance-focused plot
    generate_multi_station_distance_plot(
        &optimal_stations,
        optimal_distance,
        &suboptimal_configs_distance,
        &obstacles,
        field_bounds,
    );
    
    // Generate energy comparison plot with all configurations
    let mut all_configs_energy = vec![(optimal_stations.clone(), optimal_energy)];
    all_configs_energy.extend(suboptimal_configs_energy);
    
    println!("Demo plots generated successfully!");
    println!("Energy plot: Optimal configuration 2 stations at ({:.1}, {:.1}) and ({:.1}, {:.1}) with {:.1} Wh energy consumption",
        optimal_stations[0].x, optimal_stations[0].y, optimal_stations[1].x, optimal_stations[1].y, optimal_energy);
    println!("Distance plot: Same stations with {:.1} m total traveling distance", optimal_distance);
    println!("Field boundaries: ({:.1}, {:.1}) to ({:.1}, {:.1})", 
        field_bounds.0, field_bounds.2, field_bounds.1, field_bounds.3);
    println!("Loaded {} obstacles from configuration", obstacles.len());
}
