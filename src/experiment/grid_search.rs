use crate::cfg::{
    DEFAULT_SCENE_CONFIG_PATH, DEFAULT_AGENT_CONFIG_PATH,
};
use crate::tool_module::experiment_tool::{SingleEvaluation, TerminationCondition};
use crate::environment::{
    datetime::DateTimeConfig,
    env_module::env_config::EnvConfig,
    scene_config::SceneConfig,
    field_config::FieldConfig,
};
use crate::task_module::strategies::{ChargingStrategy, ChooseStationStrategy};
use crate::units::{energy::Energy, duration::Duration};
// use crate::experiment::visualization::generate_grid_search_plots;
use crate::experiment::geometry::generate_valid_grid_points;
use crate::experiment::results::save_grid_search_results;
use egui::Pos2;
use rand;

fn separator() {
    println!("{}", "-".repeat(70));
}

/// Grid search experiment with optional optimization minimum point and value for visualization
pub fn run_grid_search_experiment(
    grid_resolution: usize, 
    optimization_minimum: Option<(Pos2, f64)>,
    output_dir: &str
    ) {

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

    //separator();
    println!("Progress | Position |  Energy | Total dist | Charging dist | Time\n");
    
    // Store results for analysis
    let mut results: Vec<(Pos2, f64, f64, f64, f64)> = Vec::new(); // (position, energy, total_distance, charging_distance, time)
    let total_points = grid_points.len();
    
    // Run experiment for each grid point
    for (i, grid_point) in grid_points.iter().enumerate() {
     
        let start = std::time::Instant::now();

        // Update scene config with new station position
        let (energy_consumption, total_distance, charging_distance) = run_single_grid_experiment(*grid_point, &scene_config);

        let elapsed = start.elapsed().as_secs_f64();

        results.push((
            *grid_point, 
            energy_consumption, 
            total_distance, 
            charging_distance, 
            elapsed
        ));
        
        let progress = ((i + 1) as f64 / total_points as f64) * 100.0;

        let progress_str = if progress >= 100.0 {
            format!("{:>4.0}", progress)  // no decimals
        } else {
            format!("{:>4.1}", progress)  // one decimal
        };

        //let width = total_points.to_string().len(); // number of digits in total_points

        println!(
            "[{}%] | ({:>5.2},{:>5.2}) | {:.2} kWh | {:.2} km | {:.2} km | {:.2}s",
            progress_str, // progress
            // i + 1,
            // total_points, //({:>width$}/{})
            grid_point.x,
            grid_point.y,
            energy_consumption/1000.0,
            total_distance/1000.0,
            charging_distance/1000.0,
            elapsed
        );

    }
    
    // Save results to file
    let results_file = format!(
        "{}/grid_search_{}x{}_results.json",
        output_dir,
        grid_resolution, 
        grid_resolution);

    std::fs::create_dir_all(output_dir)
    .expect("Failed to create output directory");

    save_grid_search_results(&results, &results_file, optimization_minimum, &field_config, grid_resolution);
    
    // Generate plots
    // generate_grid_search_plots(&results, &obstacles, grid_resolution, optimization_minimum);
    
    // Find and report best position (based on energy consumption)
    let best_result = results.iter()
        .min_by(|a, b| a.1.partial_cmp(&b.1).unwrap())
        .unwrap();
    
    separator();
    println!("Summary");
    println!("\nValid pts/Total pts : {}/{}", 
             grid_points.len(), grid_resolution * grid_resolution);
    println!("Best position       : ({:.2}, {:.2})", best_result.0.x, best_result.0.y);
    println!("Energy              : {:.2} kWh", best_result.1/1000.0);
    println!("Distance            : {:.2} km", best_result.2/1000.0);
    println!("Charging dist       : {:.2} km", best_result.3/1000.0);
    println!("Results saved as    : {}", results_file);
    // println!("Output directory: {}", output_dir);
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
    let mut runner = SingleEvaluation {
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