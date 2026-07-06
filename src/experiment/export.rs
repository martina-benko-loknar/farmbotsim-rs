use crate::cfg::DEFAULT_SCENE_CONFIG_PATH;
use std::fs;

use crate::environment::{
    field_config::FieldConfig,
    scene_config::SceneConfig,
};
use crate::experiment::models::{
    EgoOptimizationResults, 
    GridSearchPoint, 
    GridSearchResults, 
    MultiStationExperimentResults, 
    SingleStationExperimentResults
};
use serde_json::{json, Value};

pub fn save_grid_search_results(
    results: &GridSearchResults,
    filename: &str,
    output_dir: &str,
    timestamp: &str,
) -> String {

    // ---------------------------------------------------------
    // Output file
    // ---------------------------------------------------------

    let absolute_path = format!(
        "{}/grid_search_{}_{}.json",
        output_dir,
        timestamp,
        filename,
    );

    // ---------------------------------------------------------
    // Convert grid search points
    // ---------------------------------------------------------

    let points_json: Vec<Value> = results
        .points
        .iter()
        .map(|point: &GridSearchPoint| {
            json!({
                "x": point.position.x,
                "y": point.position.y,
                "energy_consumption": point.energy,
                "total_distance": point.total_distance,
                "charging_distance": point.charging_distance,
                "runtime_sec": point.runtime_sec
            })
        })
        .collect();

    // ---------------------------------------------------------
    // Timing statistics
    // ---------------------------------------------------------

    let runtimes: Vec<f64> = results
        .points
        .iter()
        .map(|p| p.runtime_sec)
        .collect();

    let (avg_time, max_time, min_time, total_time) =
        if runtimes.is_empty() {
            (0.0, 0.0, 0.0, 0.0)
        } else {
            let total: f64 = runtimes.iter().sum();

            (
                total / runtimes.len() as f64,
                runtimes.iter().cloned().fold(f64::MIN, f64::max),
                runtimes.iter().cloned().fold(f64::MAX, f64::min),
                total,
            )
        };

    // ---------------------------------------------------------
    // Load field metadata
    // ---------------------------------------------------------

    let scene_config: SceneConfig =
        crate::utilities::utils::load_json_or_panic(
            DEFAULT_SCENE_CONFIG_PATH.to_string(),
        );

    let field_config_path =
        scene_config.field_config_path.clone();

    let field_config: FieldConfig =
        crate::utilities::utils::load_json_or_panic(
            field_config_path.clone(),
        );

    let raw_field_json =
        std::fs::read_to_string(&field_config_path).ok();

    // Export obstacle polygons 
    let obstacles_json: Vec<Value> = field_config
        .get_obstacles()
        .iter()
        .map(|obs| {

            let points: Vec<Value> = obs
                .points
                .iter()
                .map(|p| {
                    json!({
                        "x": p.x,
                        "y": p.y
                    })
                })
                .collect();

            Value::Array(points)
        })
        .collect();

    // ---------------------------------------------------------
    // Final JSON
    // ---------------------------------------------------------

    let output = json!({

        "grid_search": {

            "grid_resolution":
                results.grid_resolution,

            "valid_points":
                results.valid_points,

            "total_points":
                results.total_points,

            "best_point": {
                "x": results.best_point.position.x,
                "y": results.best_point.position.y,
                "energy_consumption":
                    results.best_point.energy,
                "total_distance":
                    results.best_point.total_distance,
                "charging_distance":
                    results.best_point.charging_distance,
                "runtime_sec":
                    results.best_point.runtime_sec
            },
            
            "experiment": {
                "field_size": results.experiment_configs.field_config_path,
                "fleet_size": results.experiment_configs.n_agents,
                "battery_capacity_wh": results.experiment_configs.battery_capacity_wh,
                "soc_threshold_percent": results.experiment_configs.soc_threshold_percent,
                "critical_soc_percent": results.experiment_configs.critical_soc_percent,
            },

            "points": points_json
        },

        "timing": {
            "average_evaluation_time_sec": avg_time,
            "max_evaluation_time_sec": max_time,
            "min_evaluation_time_sec": min_time,
            "total_grid_search_time_sec": total_time
        },

        "field": {
            "field_config_path": field_config_path,

            "raw_field_config": raw_field_json,

            "num_obstacles":
                field_config.get_obstacles().len(),

            "num_field_configs":
                field_config.configs.len(),

            // Obstacle polygons exported from Rust
            "obstacles": obstacles_json
        },

        "generated_at":
            chrono::Utc::now().to_rfc3339()
    });

    // ---------------------------------------------------------
    // Save
    // ---------------------------------------------------------

    match std::fs::write(
        &absolute_path,
        serde_json::to_string_pretty(&output).unwrap(),
    ) {
        Ok(_) => {
            println!("Results saved to    : {absolute_path}");
        }

        Err(e) => {
            eprintln!("Failed to save results: {}", e);
        }
    }

    absolute_path
}

pub fn save_single_station_results(
    results: &SingleStationExperimentResults,
    timestamp: &str,
    output_dir: &str,
) -> String {
    // // ---------------------------------------------------------
    // // Create output directory
    // // ---------------------------------------------------------

    // if let Err(e) = std::fs::create_dir_all(output_dir) {
    //     eprintln!("Failed to create output directory: {}", e);
    //     return;
    // }

    // ---------------------------------------------------------
    // Output file
    // ---------------------------------------------------------

    let filename = format!(
        "{}/single_station_{}_results_gr{}.json",
        output_dir,
        timestamp,
        results.grid_search.grid_resolution,
    );

    // ---------------------------------------------------------
    // Convert grid search points
    // ---------------------------------------------------------

    let points_json: Vec<Value> = results
        .grid_search
        .points
        .iter()
        .map(|point: &GridSearchPoint| {
            json!({
                "x": point.position.x,
                "y": point.position.y,
                "energy_consumption": point.energy,
                "total_distance": point.total_distance,
                "charging_distance": point.charging_distance,
                "runtime_sec": point.runtime_sec
            })
        })
        .collect();

    // ---------------------------------------------------------
    // Timing statistics
    // ---------------------------------------------------------

    let runtimes: Vec<f64> = results
        .grid_search
        .points
        .iter()
        .map(|p| p.runtime_sec)
        .collect();

    let (avg_time, max_time, min_time, total_time) =
        if runtimes.is_empty() {
            (0.0, 0.0, 0.0, 0.0)
        } else {
            let total: f64 = runtimes.iter().sum();

            (
                total / runtimes.len() as f64,
                runtimes.iter().cloned().fold(f64::MIN, f64::max),
                runtimes.iter().cloned().fold(f64::MAX, f64::min),
                total,
            )
        };

    // ---------------------------------------------------------
    // Load field metadata
    // ---------------------------------------------------------

    let scene_config: SceneConfig =
        crate::utilities::utils::load_json_or_panic(
            DEFAULT_SCENE_CONFIG_PATH.to_string(),
        );

    let field_config_path =
        scene_config.field_config_path.clone();

    let field_config: FieldConfig =
        crate::utilities::utils::load_json_or_panic(
            field_config_path.clone(),
        );

    let raw_field_json =
        std::fs::read_to_string(&field_config_path).ok();

    // Export obstacle polygons 
    let obstacles_json: Vec<Value> = field_config
        .get_obstacles()
        .iter()
        .map(|obs| {

            let points: Vec<Value> = obs
                .points
                .iter()
                .map(|p| {
                    json!({
                        "x": p.x,
                        "y": p.y
                    })
                })
                .collect();

            Value::Array(points)
        })
        .collect();

    // ---------------------------------------------------------
    // Final JSON
    // ---------------------------------------------------------

    let output = json!({

        "experiment": {
            "type": "single_station"
        },

        "ego_summary": {
            "optimal_positions": results.ego.summary.optimal_position
            .iter()
            .map(|p| json!({
                "x": p.x,
                "y": p.y
            }))
            .collect::<Vec<_>>(),


            "optimal_energy_consumption":
                results.ego.summary.optimal_energy,
            "optimization_time_sec": results.ego.summary.optimization_time_sec,
            "total_evaluations": results.ego.summary.total_evaluations
        },

        "grid_search": {

            "grid_resolution":
                results.grid_search.grid_resolution,

            "valid_points":
                results.grid_search.valid_points,

            "total_points":
                results.grid_search.total_points,

            "best_point": {
                "x": results.grid_search.best_point.position.x,
                "y": results.grid_search.best_point.position.y,
                "energy_consumption":
                    results.grid_search.best_point.energy,
                "total_distance":
                    results.grid_search.best_point.total_distance,
                "charging_distance":
                    results.grid_search.best_point.charging_distance,
                "runtime_sec":
                    results.grid_search.best_point.runtime_sec
            },

            "points": points_json
        },

        "timing": {
            "average_evaluation_time_sec": avg_time,
            "max_evaluation_time_sec": max_time,
            "min_evaluation_time_sec": min_time,
            "total_grid_search_time_sec": total_time
        },

        "field": {
            "field_config_path": field_config_path,

            "raw_field_config": raw_field_json,

            "num_obstacles":
                field_config.get_obstacles().len(),

            "num_field_configs":
                field_config.configs.len(),

            "obstacles": obstacles_json
        },

        "generated_at":
            chrono::Utc::now().to_rfc3339()
    });

    // ---------------------------------------------------------
    // Save
    // ---------------------------------------------------------

    match std::fs::write(
        &filename,
        serde_json::to_string_pretty(&output).unwrap(),
    ) {
        Ok(_) => {
            println!("Saved experiment results:");
            println!("{}", filename);
        }

        Err(e) => {
            eprintln!("Failed to save results: {}", e);
        }
    }

    filename
}

pub fn save_ego_results(
    results: &EgoOptimizationResults,
    output_dir: &str,
    experiment_type: &str,
    timestamp: &str,
) -> String {
    use serde_json::json;
    use std::fs;

    let output = json!({
        "experiment": "ego_optimization",
        "timestamp": timestamp,

        "ego_summary": {
            "optimal_energy_consumption": results.summary.optimal_energy,
            "optimal_positions": results.summary.optimal_position
                    .iter()
                    .map(|p| json!({
                        "x": p.x,
                        "y": p.y
                    }))
                    .collect::<Vec<_>>(),
                    },

        "optimization_time_sec": results.summary.optimization_time_sec,
        "total_evaluations": results.summary.total_evaluations,
    });

    let filename = format!(
        "{}/{}_{}_trace.json",
        output_dir,
        experiment_type,
        timestamp
    );

    fs::write(&filename, serde_json::to_string_pretty(&output).unwrap())
        .unwrap();

    filename
}

pub fn save_ego_trace_results(
    ego_results: &EgoOptimizationResults,
    experiment_type: &str,
    output_dir: &str,
    timestamp: &str,
) -> String {

    let trace = &ego_results.trace;
    let summary = &ego_results.summary;

    let final_best_energy = trace
        .evaluation_history
        .last()
        .map(|e| e.best_energy)
        .unwrap_or(f64::INFINITY);

    let results = json!({
        "timestamp": timestamp,
        
        "ego_summary": {
            "optimal_positions": summary.optimal_position
                .iter()
                .map(|p| json!({
                    "x": p.x,
                    "y": p.y
                }))
                .collect::<Vec<_>>(),
            "optimal_energy": summary.optimal_energy,
            "optimization_time_sec": summary.optimization_time_sec,
            "total_evaluations": summary.total_evaluations
        },

        "trace": {
            "evaluations": trace.evaluation_history
        },

        "final_best_energy": final_best_energy,
    });

    fs::create_dir_all(output_dir).unwrap();

    let filename = format!(
        "{}/{}_{}_ego_trace.json",
        output_dir,
        experiment_type,
        timestamp
    );

    match serde_json::to_string_pretty(&results) {
        Ok(json) => {
            if let Err(e) = fs::write(&filename, json) {
                eprintln!("Failed to write ego trace results: {}", e);
            } else {
                println!("EGO trace saved to: {}", filename);
            }
        }
        Err(e) => eprintln!("Serialization failed: {}", e),
    }

    filename
}

pub fn save_multi_station_results(
    results: &MultiStationExperimentResults,
    output_dir: &str,
    timestamp: &str,
) -> String {

    let summary = &results.ego.summary;
    let specialist_results = &results.specialist;

    let results = json!({
        "timestamp": timestamp,
        
        "ego_summary": {
            "optimal_positions": summary.optimal_position
                .iter()
                .map(|p| json!({
                    "x": p.x,
                    "y": p.y
                }))
                .collect::<Vec<_>>(),
            "optimal_energy": summary.optimal_energy,
            "optimization_time_sec": summary.optimization_time_sec,
            "total_evaluations": summary.total_evaluations
        },

        "specialist_layout_evaluations": {
            "evaluations": specialist_results,
        },


    });

    fs::create_dir_all(output_dir).unwrap();

    let filename = format!(
        "{}/multi_station_{}_results.json",
        output_dir,
        timestamp
    );

    match serde_json::to_string_pretty(&results) {
        Ok(json) => {
            if let Err(e) = fs::write(&filename, json) {
                eprintln!("Failed to write multi-station results: {}", e);
            } else {
                println!("Multi-station results saved to: {}", filename);
            }
        }
        Err(e) => eprintln!("Serialization failed: {}", e),
    }

    filename
}