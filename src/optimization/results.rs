// Save the optimal station configuration
use crate::cfg::{DEFAULT_SCENE_CONFIG_PATH};
use crate::environment::{
    scene_config::SceneConfig,
};

use crate::utilities::utils::load_json_or_panic;
use crate::optimization::station_positions::StationPositions;
use crate::optimization::geometry::is_position_valid;
use crate::optimization::constants::*;
use crate::environment::field_config::FieldConfig;
use crate::environment::geometry::FieldBounds;
use crate::experiment::models::EvaluationRecord;

//use egui::output;
use serde::{Deserialize, Serialize};
use std::fs;

// Serializable structure for convergence history
#[derive(Serialize, Deserialize, Clone, Debug)]
pub struct ConvergenceRecord {
    pub iteration: usize,
    pub best_energy: f64,
    pub best_positions: Vec<(f32, f32)>,
}

#[derive(Serialize)]
pub struct EvaluatedSample {
    pub positions: Vec<(f32, f32)>,
    pub energy: f64,
}

#[derive(Serialize)]
pub struct SerializableObstacle {
    pub points: Vec<(f32, f32)>,
}

// ============================================================
// JSON structure
// ============================================================
#[derive(Serialize, Deserialize, Clone, Debug)]
pub struct OptimizationResults {
    pub timestamp: String,
    pub max_iterations: usize,
    pub total_evaluations: usize,
    pub optimization_time_seconds: f64,
    pub final_best_energy: f64,
    pub evaluations: Vec<EvaluationRecord>,

    pub field_boundaries: (f32, f32, f32, f32), // (min_x, max_x, min_y, max_y)
    pub station_margin: f32,
    pub obstacle_margin: f32,
}

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
        let valid = is_position_valid(*pos, &positions.obstacles);
        println!("  Station {}: ({:.1}, {:.1}) - {}", 
                i + 1, pos.x, pos.y, if valid { "✓ Valid" } else { "✗ Invalid" });
    }
    
    path
}

// ============================================================
// Save function
// ============================================================
pub fn save_results(
    evaluations: &Vec<EvaluationRecord>,
    max_iterations: usize,
    total_evaluations: usize,
    optimization_time: std::time::Duration,
    output_dir: &str,
) -> String {
    let timestamp = chrono::Utc::now().format("%H%M%S");

    let final_best_energy = evaluations
        .last()
        .map(|e| e.best_energy)
        .unwrap_or(f64::INFINITY);

    // ---------------------------------------------------------
    // Load scene + field config
    // ---------------------------------------------------------
    let scene_config: SceneConfig =
        load_json_or_panic(DEFAULT_SCENE_CONFIG_PATH.to_string());

    let field_config: FieldConfig =
        load_json_or_panic(scene_config.field_config_path);

    // ---------------------------------------------------------
    // Compute REAL field bounds
    // ---------------------------------------------------------
    
    let field_bounds = FieldBounds::from_field_config(&field_config);

    // ---------------------------------------------------------
    // Build results
    // ---------------------------------------------------------
    let results = OptimizationResults {
        timestamp: timestamp.to_string(),
        max_iterations,
        total_evaluations,
        optimization_time_seconds: optimization_time.as_secs_f64(),
        final_best_energy,
        evaluations: evaluations.clone(),

        field_boundaries: (
            field_bounds.min_x,
            field_bounds.max_x,
            field_bounds.min_y,
            field_bounds.max_y,
        ),

        station_margin: STATION_MARGIN,
        obstacle_margin: OBSTACLE_MARGIN,
    };

    fs::create_dir_all(output_dir).unwrap();

    let filename = format!(
        "{}/single_station_{}_ego_trace.json",
        output_dir, timestamp
    );

    match serde_json::to_string_pretty(&results) {
        Ok(json) => {
            if let Err(e) = fs::write(&filename, json) {
                eprintln!("Failed to write results: {}", e);
            } else {
                println!("Results saved to : {}", filename);
            }
        }
        Err(e) => eprintln!("Serialization failed: {}", e),
    }

    filename
}