//use plotly::layout::LayoutScene;

use crate::experiment::grid_search::grid_search_experiment;
use crate::experiment::export::{
    save_grid_search_results,
    save_single_station_results, 
    save_ego_trace_results,
    save_multi_station_results};
use crate::experiment::station_layouts::{
    specialist_layouts,
    evaluate_station_layouts};
use crate::optimization::ego::optimize_station_positions_ego;
use crate::experiment::models::{
    SingleStationExperimentResults,
    MultiStationExperimentResults,
};

use crate::environment::{
    field_config::FieldConfig,
    scene_config::SceneConfig,
};

use crate::cfg::DEFAULT_SCENE_CONFIG_PATH;

use crate::utilities::utils::load_json_or_panic;

pub fn run_ego_experiment(
    n_stations: usize,
    max_iterations: usize,
    output_dir: &str,
) {
    println!("Running EGO optimization...");
    let timestamp = chrono::Utc::now()
    .format("%H%M%S")
    .to_string(); 

    // ---------------------------------------------------------
    // EGO optimization
    // ---------------------------------------------------------

    let ego_results =
        optimize_station_positions_ego(
            n_stations,
            max_iterations,
            output_dir);

    // ---------------------------------------------------------
    // Save experiment results
    // ---------------------------------------------------------

    save_ego_trace_results(
        &ego_results,
        "ego_optimization",
        output_dir,
        &timestamp,
    );

    println!("EGO optimization experiment completed.");
}

pub fn run_grid_search_experiment(
    resolution: usize,
    output_dir: &str,
) {
    println!("Running grid search experiment...");
    let timestamp = chrono::Utc::now()
    .format("%H%M%S")
    .to_string(); 

    // ---------------------------------------------------------
    // Grid search
    // ---------------------------------------------------------

    let grid_results = grid_search_experiment(
        resolution
    );
    // ---------------------------------------------------------
    // Save experiment results
    // ---------------------------------------------------------

    save_grid_search_results(
        &grid_results,
        output_dir,
        &timestamp,
    );

    println!("Grid search experiment completed.");
}

pub fn run_single_station_experiment(
    resolution: usize,
    output_dir: &str,
) {
    println!("Running single-station experiment...");
    let timestamp = chrono::Utc::now()
    .format("%H%M%S")
    .to_string();

    // ---------------------------------------------------------
    // Phase 1: EGO optimization
    // ---------------------------------------------------------

    let ego_results =
        optimize_station_positions_ego(
            1,
            10,
            output_dir);

    // println!(
    //     "EGO minimum: ({:.3}, {:.3}) -> {:.2} Wh",
    //     ego_results.summary.optimal_position.x,
    //     ego_results.summary.optimal_position.y,
    //     ego_results.summary.optimal_energy
    // );

    // ---------------------------------------------------------
    // Phase 2: Grid search
    // ---------------------------------------------------------

    let grid_results = grid_search_experiment(
        resolution
    );

    // ---------------------------------------------------------
    // Save & aggregate experiment results
    // ---------------------------------------------------------

    save_ego_trace_results(
        &ego_results,
        "single_station",
        output_dir,
        &timestamp,
    );

    let experiment_results =
        SingleStationExperimentResults {
            ego: ego_results,
            grid_search: grid_results,
        };

    save_single_station_results(
        &experiment_results,
        &timestamp, 
        output_dir,
    );

    println!("Single-station experiment completed.");
}

pub fn run_multi_station_experiment(
    max_iterations: usize,
    output_dir: &str,
) {
    println!("Running multi-station experiment...");
    let timestamp = chrono::Utc::now()
    .format("%H%M%S")
    .to_string();

    let scene_config: SceneConfig =
    load_json_or_panic(
        DEFAULT_SCENE_CONFIG_PATH.to_string()
    );

    let field_config: FieldConfig =
        load_json_or_panic(
            scene_config.field_config_path.clone()
        );

    // ---------------------------------------------------------
    // Phase 1: EGO optimization
    // ---------------------------------------------------------

    let ego_results =
        optimize_station_positions_ego(
            2,
            max_iterations,
            output_dir);

    // ---------------------------------------------------------
    // Phase 2: Specialist layouts
    // ---------------------------------------------------------
    
    let layouts = 
        specialist_layouts(&field_config);

    let specialist_results = 
        evaluate_station_layouts(&layouts);

    // ---------------------------------------------------------
    // Save & aggregate experiment results
    // ---------------------------------------------------------

    save_ego_trace_results(
        &ego_results,
        "multi_station",
        output_dir,
        &timestamp,
    );

    let experiment_results =
        MultiStationExperimentResults {
            ego: ego_results,
            specialist: specialist_results,
        };

    save_multi_station_results(
        &experiment_results,
        output_dir,
        &timestamp, 
    );

    println!("Multi-station experiment completed.");
}