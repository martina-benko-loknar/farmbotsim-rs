// Save the optimal station configuration
use crate::cfg::{DEFAULT_SCENE_CONFIG_PATH};
use crate::environment::{
    scene_config::SceneConfig,
};
use crate::environment::obstacle::Obstacle;

use crate::utilities::utils::load_json_or_panic;
use crate::optimization::station_positions::StationPositions;
use crate::optimization::geometry::is_position_valid;
use crate::optimization::constants::*;

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
    // pub evaluated_positions: Vec<EvaluatedSample>, // all sampled station configs
    // pub obstacles: Vec<SerializableObstacle>,           // obstacle polygons
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

/// Save convergence history to JSON file
pub fn save_convergence_history(
    convergence_history: &[(usize, f64, Vec<(f32, f32)>)],
    max_iterations: usize,
    total_evaluations: usize,
    optimization_time: std::time::Duration,
    output_dir: &str,
    evaluated_positions: &Vec<(Vec<(f32, f32)>, f64)>,
    obstacles: &Vec<Obstacle>
) -> String {
    let timestamp = chrono::Utc::now().format("%H%M%S");
    
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
    
    // let serializable_obstacles: Vec<SerializableObstacle> = obstacles
    //     .iter()
    //     .map(|obs| SerializableObstacle {
    //         points: obs.points.iter().map(|p| (p.x, p.y)).collect(),
    //     })
    //     .collect();

    // let evaluated_samples: Vec<EvaluatedSample> = evaluated_positions
    //     .iter()
    //     .map(|(pos, energy)| EvaluatedSample {
    //         positions: pos.clone(),
    //         energy: *energy,
    //     })
    //     .collect();

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
        // evaluated_positions: evaluated_samples,
        // obstacles: serializable_obstacles,
    };
    
    
    fs::create_dir_all(output_dir).unwrap();

    let filename = format!(
        "{}/optimization_results_{}.json", 
        output_dir, 
        timestamp);

    match serde_json::to_string_pretty(&optimization_results) {
        Ok(json_string) => {
            match std::fs::write(&filename, json_string) {
                Ok(_) => {
                    println!("Results saved to : {}", filename);
                    // println!("Results summary:");
                    // println!("  Total iterations: {}", convergence_history.len());
                    // println!("  Total evaluations: {}", total_evaluations);
                    // println!("  Optimization time: {:.2} seconds", optimization_time.as_secs_f64());
                    // println!("  Final best energy: {:.2} Wh", final_best_energy);
                },
                Err(e) => eprintln!("Failed to write optimization results to file: {}", e),
            }
        },
        Err(e) => eprintln!("Failed to serialize optimization results: {}", e),
    }

    filename
}