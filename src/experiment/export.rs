use crate::experiment::config::ExperimentConfig;
use crate::experiment::models::EgoExport;
use std::fs;
use crate::experiment::models::TimingStatistics;
use crate::experiment::models::FieldExport;
use crate::experiment::models::GridSearchExport;
use crate::experiment::models::ExperimentMetadata;
use egui::Pos2;
use crate::experiment::models::SingleStationExport;
use crate::experiment::models::MultiStationExport;

use crate::environment::field_config::FieldConfig;

use crate::experiment::models::{
    EgoOptimizationResults, 
    GridSearchResults, 
    MultiStationExperimentResults, 
    SingleStationExperimentResults,
    ExperimentInfo
};

// Helper for field extraction

fn export_field(config: &ExperimentConfig) -> FieldExport {

    let field_config: FieldConfig = config.load_field_config();

    let obstacles: Vec<Vec<Pos2>> = field_config
        .get_obstacles()
        .iter()
        .map(|obs| obs.points.clone())
        .collect();

    FieldExport {
        config_path: config.field_config_path.clone(),
        obstacle_count: field_config.get_obstacles().len(),
        obstacles,
    }
}

pub fn save_grid_search_results(
    results: &GridSearchResults,
    config: &ExperimentConfig,
    info: &ExperimentInfo,
    filename: &str,
    output_dir: &str,
) -> String {

    // ---------------------------------------------------------
    // Output file
    // ---------------------------------------------------------

    let absolute_path = format!(
        "{}/{}_{}.json",
        output_dir,
        info.timestamp,
        filename,
    );

    // ---------------------------------------------------------
    // Timing statistics
    // ---------------------------------------------------------

    let runtimes: Vec<f64> = results
        .trace
        .points
        .iter()
        .map(|p| p.metrics.evaluation_time_sec)
        .collect();

    let timing = if runtimes.is_empty() {
        TimingStatistics {
            average_evaluation_time_sec: 0.0,
            minimum_evaluation_time_sec: 0.0,
            maximum_evaluation_time_sec: 0.0,
            total_evaluation_time_sec: 0.0,
        }
    } else {
        let total: f64 = runtimes.iter().sum();

        TimingStatistics {
            average_evaluation_time_sec: total / runtimes.len() as f64,
            minimum_evaluation_time_sec: runtimes.iter().cloned().fold(f64::MAX, f64::min),
            maximum_evaluation_time_sec: runtimes.iter().cloned().fold(f64::MIN, f64::max),
            total_evaluation_time_sec: total,
        }
    };

    // ---------------------------------------------------------
    // Load field 
    // ---------------------------------------------------------

    let field = export_field(config);

    // ---------------------------------------------------------
    // Metadata
    // ---------------------------------------------------------

    let metadata = ExperimentMetadata {
        experiment_type: info.experiment_type.clone(),
        config: config.clone(),
        timestamp: info.timestamp.clone(),
        seed: config.seed,
    };

    // ---------------------------------------------------------
    // Final JSON: assemble export
    // ---------------------------------------------------------

    let export = GridSearchExport {
        metadata,
        grid_search: results.clone(),
        timing,
        field,
    };

    // ---------------------------------------------------------
    // Save
    // ---------------------------------------------------------

    match std::fs::write(
        &absolute_path,
        serde_json::to_string_pretty(&export).unwrap(),
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
    config: &ExperimentConfig,
    info: &ExperimentInfo,
    filename: &str,
    output_dir: &str,
) -> String {

    // ---------------------------------------------------------
    // Output file
    // ---------------------------------------------------------

    let absolute_path = format!(
        "{}/{}_{}.json",
        output_dir,
        info.timestamp,
        filename,
    );

    // ---------------------------------------------------------
    // Timing statistics
    // ---------------------------------------------------------

    let mut runtimes = Vec::new();

    runtimes.extend(
        results
            .grid_search
            .trace
            .points
            .iter()
            .map(|p| p.metrics.evaluation_time_sec)
    );

    runtimes.extend(
        results
            .ego
            .trace
            .evaluation_history
            .iter()
            .map(|e| e.metrics.evaluation_time_sec)
    );


    let timing = if runtimes.is_empty() {
        TimingStatistics {
            average_evaluation_time_sec: 0.0,
            minimum_evaluation_time_sec: 0.0,
            maximum_evaluation_time_sec: 0.0,
            total_evaluation_time_sec: 0.0,
        }
    } else {
        let total: f64 = runtimes.iter().sum();

        TimingStatistics {
            average_evaluation_time_sec: total / runtimes.len() as f64,
            minimum_evaluation_time_sec: runtimes.iter().cloned().fold(f64::MAX, f64::min),
            maximum_evaluation_time_sec: runtimes.iter().cloned().fold(f64::MIN, f64::max),
            total_evaluation_time_sec: total,
        }
    };

    // ---------------------------------------------------------
    // Load field 
    // ---------------------------------------------------------

    let field = export_field(config);

    // ---------------------------------------------------------
    // Metadata
    // ---------------------------------------------------------

    let metadata = ExperimentMetadata {
        experiment_type: info.experiment_type.clone(),
        config: config.clone(),
        timestamp: info.timestamp.clone(),
        seed: config.seed, 
    };

    // ---------------------------------------------------------
    // Assemble export
    // ---------------------------------------------------------

    let export = SingleStationExport {
        metadata,
        ego: results.ego.clone(),
        grid_search: results.grid_search.clone(),
        timing,
        field,
    };
  
    // ---------------------------------------------------------
    // Save
    // ---------------------------------------------------------

    match fs::write(
        &absolute_path,
        serde_json::to_string_pretty(&export).unwrap(),
    ) {

        Ok(_) => {
            println!("Single-station results saved:");
            println!("{}", absolute_path);
        }

        Err(e) => {
            eprintln!("Failed saving results: {}", e);
        }
    }

    absolute_path
}

pub fn save_ego_results(
    results: &EgoOptimizationResults,
    config: &ExperimentConfig,
    info: &ExperimentInfo,
    filename: &str,
    output_dir: &str,
) -> String {

    // ---------------------------------------------------------
    // Output file
    // ---------------------------------------------------------

    let absolute_path = format!(
        "{}/{}_{}.json",
        output_dir,
        info.timestamp,
        filename,
    );

    // ---------------------------------------------------------
    // Timing statistics
    // ---------------------------------------------------------

    let runtimes: Vec<f64> = results
        .trace
        .evaluation_history
        .iter()
        .map(|e| e.metrics.evaluation_time_sec)
        .collect();

    let timing = if runtimes.is_empty() {
        TimingStatistics {
            average_evaluation_time_sec: 0.0,
            minimum_evaluation_time_sec: 0.0,
            maximum_evaluation_time_sec: 0.0,
            total_evaluation_time_sec: 0.0,
        }
    } else {
        let total: f64 = runtimes.iter().sum();

        TimingStatistics {
            average_evaluation_time_sec: total / runtimes.len() as f64,
            minimum_evaluation_time_sec: runtimes.iter().cloned().fold(f64::MAX, f64::min),
            maximum_evaluation_time_sec: runtimes.iter().cloned().fold(f64::MIN, f64::max),
            total_evaluation_time_sec: total,
        }
    };

    // ---------------------------------------------------------
    // Load field
    // ---------------------------------------------------------

    let field = export_field(config);

    // ---------------------------------------------------------
    // Metadata
    // ---------------------------------------------------------

    let metadata = ExperimentMetadata {
        experiment_type: info.experiment_type.clone(),
        config: config.clone(),
        timestamp: info.timestamp.clone(),
        seed: config.seed, 
    };

    // ---------------------------------------------------------
    // Final JSON: assemble export
    // ---------------------------------------------------------

    let export = EgoExport {
        metadata,
        ego: results.clone(),
        timing,
        field,
    };

    // ---------------------------------------------------------
    // Save
    // ---------------------------------------------------------

    match std::fs::write(
        &absolute_path,
        serde_json::to_string_pretty(&export).unwrap(),
    ) {
        Ok(_) => {
            println!("Results saved to    : {}", absolute_path);
        }

        Err(e) => {
            eprintln!("Failed to save results: {}", e);
        }
    }

    absolute_path
}


pub fn save_multi_station_results(
    results: &MultiStationExperimentResults,
    config: &ExperimentConfig,
    info: &ExperimentInfo,
    filename: &str,
    output_dir: &str,
) -> String {

    // ---------------------------------------------------------
    // Output file
    // ---------------------------------------------------------

    let absolute_path = format!(
        "{}/{}_{}.json",
        output_dir,
        info.timestamp,
        filename,
    );

    // ---------------------------------------------------------
    // Timing statistics
    // ---------------------------------------------------------

    let runtimes: Vec<f64> = results
        .ego
        .trace
        .evaluation_history
        .iter()
        .map(|e| e.metrics.evaluation_time_sec)
        .collect();


    let timing = if runtimes.is_empty() {

        TimingStatistics {
            average_evaluation_time_sec: 0.0,
            minimum_evaluation_time_sec: 0.0,
            maximum_evaluation_time_sec: 0.0,
            total_evaluation_time_sec: 0.0,
        }

    } else {

        let total: f64 = runtimes.iter().sum();

        TimingStatistics {
            average_evaluation_time_sec: total / runtimes.len() as f64,
            minimum_evaluation_time_sec: runtimes.iter().cloned().fold(f64::MAX, f64::min),
            maximum_evaluation_time_sec: runtimes.iter().cloned().fold(f64::MIN, f64::max),
            total_evaluation_time_sec: total,
        }
    };

    // ---------------------------------------------------------
    // Load field 
    // ---------------------------------------------------------

    let field = export_field(config);

    // ---------------------------------------------------------
    // Metadata
    // ---------------------------------------------------------

    let metadata = ExperimentMetadata {
        experiment_type: info.experiment_type.clone(),
        config: config.clone(),
        timestamp: info.timestamp.clone(),
        seed: config.seed, 
    };

    // ---------------------------------------------------------
    // Final JSON: assemble export
    // ---------------------------------------------------------

    let export = MultiStationExport {
        metadata,
        ego: results.ego.clone(),
        specialist: results.specialist.clone(),
        timing,
        field,
    };

    // ---------------------------------------------------------
    // Save
    // ---------------------------------------------------------

    match fs::write(
        &absolute_path,
        serde_json::to_string_pretty(&export).unwrap(),
    ) {

        Ok(_) => {
            println!("Multi-station results saved:");
            println!("{}", absolute_path);
        }

        Err(e) => {
            eprintln!("Failed saving results: {}", e);
        }
    }

    absolute_path
}