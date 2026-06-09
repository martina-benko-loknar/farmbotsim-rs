use crate::cfg::{DEFAULT_SCENE_CONFIG_PATH, DEFAULT_AGENT_CONFIG_PATH};
use crate::environment::{
    datetime::DateTimeConfig, 
    env_module::env_config::EnvConfig,
    scene_config::SceneConfig,
    station_module::station_config::StationConfig,
    field_config::FieldConfig,
    obstacle::Obstacle
};
use crate::tool_module::experiment_tool::{SingleEvaluation, TerminationCondition};
use crate::units::{energy::Energy, duration::Duration};
use crate::utilities::utils::load_json_or_panic;
use crate::task_module::strategies::{ChargingStrategy, ChooseStationStrategy};
use crate::optimization::geometry::round_to_centimeters;
use crate::optimization::geometry::is_position_valid;
use crate::optimization::constants::*;
use crate::environment::geometry::FieldBounds;

use ndarray::{ArrayView2};
use rand::Rng;
use std::fmt;
use egui::Pos2;

// Define the parameters for station position optimization 
#[derive(Clone)]
pub struct StationPositions {
    pub station_positions: Vec<Pos2>,
    pub obstacles: Vec<Obstacle>, // Keep obstacles for validation
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
    pub fn from_optimization_vector(
        x: &ArrayView2<f64>, 
        obstacles: &[Obstacle], 
        n_stations: usize
    ) -> Self {
        let mut station_positions = Vec::with_capacity(n_stations);

        // -------------------------------------------------
        // Compute real field bounds
        // -------------------------------------------------
        let scene_config: SceneConfig =
                load_json_or_panic(DEFAULT_SCENE_CONFIG_PATH.to_string());

        let field_config: FieldConfig =
            load_json_or_panic(scene_config.field_config_path.clone());

        let field_bounds = FieldBounds::from_field_config(&field_config);
        
        // -------------------------------------------------
        // Extract positions
        // -------------------------------------------------
        for i in 0..n_stations {

            let x_coord = x[[0, i * 2]] as f32;
            let y_coord = x[[0, i * 2 + 1]] as f32;

            // Clamp to REAL field boundaries
            let x_clamped = x_coord.clamp(
                field_bounds.min_x + STATION_MARGIN,
                field_bounds.max_x - STATION_MARGIN,
            );

            let y_clamped = y_coord.clamp(
                field_bounds.min_y + STATION_MARGIN,
                field_bounds.max_y - STATION_MARGIN,
            );

            station_positions.push(
                round_to_centimeters(
                    Pos2::new(x_clamped, y_clamped)
                )
            );
        }
        
        Self {
            station_positions,
            obstacles: obstacles.to_vec(),
        }
    }

    // Generate a valid position that doesn't intersect with obstacles
    fn generate_valid_position(obstacles: &[Obstacle], rng: &mut impl Rng) -> Pos2 {
        let max_attempts = 100; // Prevent infinite loops

        // -------------------------------------------------
        // Compute real field bounds
        // -------------------------------------------------
        let scene_config: SceneConfig =
                load_json_or_panic(DEFAULT_SCENE_CONFIG_PATH.to_string());

        let field_config: FieldConfig =
            load_json_or_panic(scene_config.field_config_path.clone());

        let field_bounds = FieldBounds::from_field_config(&field_config);
        
        // -------------------------------------------------
        // Try random valid positions
        // -------------------------------------------------
        for _ in 0..max_attempts {

            let x = rng.random_range(
                (field_bounds.min_x + STATION_MARGIN)
                ..
                (field_bounds.max_x - STATION_MARGIN)
            );

            let y = rng.random_range(
                (field_bounds.min_y + STATION_MARGIN)
                ..
                (field_bounds.max_y - STATION_MARGIN)
            );

            let candidate =
                round_to_centimeters(Pos2::new(x, y));

            if is_position_valid(candidate, obstacles) {
                return candidate;
            }
        }
        

        // -------------------------------------------------
        // Fallback
        // -------------------------------------------------
        println!(
            "Warning: Could not find valid position after {} attempts, using field center",
            max_attempts
        );

        round_to_centimeters(Pos2::new(
            (field_bounds.min_x + field_bounds.max_x) / 2.0,
            (field_bounds.min_y + field_bounds.max_y) / 2.0,
        ))
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
        let mut runner = SingleEvaluation {
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
            if !is_position_valid(*position, &self.obstacles) {
                penalty += 1000000000.0; // Heavy penalty for invalid positions
            }
        }
        
        // Return the energy consumption plus penalty
        runner.total_energy_consumed.value as f64 + penalty
    }
    
    // Helper method to create station configs from optimized positions
    pub fn create_station_configs(&self, original_scene: &SceneConfig) -> Vec<StationConfig> {
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
    pub fn generate_initial_population(obstacles: &[Obstacle], n_stations: usize, population_size: usize, rng: &mut impl Rng) -> Vec<StationPositions> {
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
