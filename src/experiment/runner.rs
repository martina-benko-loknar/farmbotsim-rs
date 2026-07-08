//use plotly::layout::LayoutScene;

use crate::experiment::config::ExperimentConfig;
use crate::experiment::grid_search::grid_search_experiment;
use crate::experiment::export::{
    save_grid_search_results,
    save_single_station_results, 
    save_ego_results,
    save_multi_station_results};
use crate::experiment::station_layouts::{
    specialist_layouts,
    evaluate_station_layouts};
use crate::optimization::ego::optimize_station_positions_ego;
use crate::experiment::models::{
    SingleStationExperimentResults,
    MultiStationExperimentResults,
    ExperimentRun,
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
    filename: &str,
    output_dir: &str,
    exp: ExperimentConfig,
) -> ExperimentRun {

    println!("Experiment type: EGO ({} stations, {} iterations max)\n", 
            n_stations, 
            max_iterations);

    let timestamp = chrono::Utc::now()
    .format("%H%M%S")
    .to_string(); 

    // ---------------------------------------------------------
    // EGO optimization
    // ---------------------------------------------------------

    let ego_results = optimize_station_positions_ego(
            n_stations,
            max_iterations,
            &exp);

    // ---------------------------------------------------------
    // Save experiment results
    // ---------------------------------------------------------

    let results_path = save_ego_results(
        &ego_results,
        &exp,
        filename,
        output_dir,
        &timestamp,
    );

    println!("EGO optimization experiment completed.");

    ExperimentRun{
        timestamp: timestamp,
        output_dir: output_dir.to_string(),
        results_path}

}

pub fn run_grid_search_experiment(
    resolution: usize,
    filename: &str,
    output_dir: &str,
    exp: ExperimentConfig,
) -> ExperimentRun {

    println!("Experiment type: GRID SEARCH ({}×{} resolution)\n", 
            resolution, 
            resolution);

    let timestamp = chrono::Utc::now()
    .format("%H%M%S")
    .to_string(); 

    // ---------------------------------------------------------
    // Grid search
    // ---------------------------------------------------------
    let grid_results = grid_search_experiment(
        resolution,
        &exp
    );
    // ---------------------------------------------------------
    // Save experiment results
    // ---------------------------------------------------------

    let results_path = save_grid_search_results(
        &grid_results,
        &exp, 
        filename,
        output_dir,
        &timestamp,
    );

    ExperimentRun{
        timestamp: timestamp,
        output_dir: output_dir.to_string(),
        results_path}
}

pub fn run_single_station_experiment(
    resolution: usize,
    filename: &str,
    output_dir: &str,
    exp: ExperimentConfig,
) -> ExperimentRun {

    println!("Experiment type: SINGLE-STATION experiment");
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
            &exp);

    // ---------------------------------------------------------
    // Phase 2: Grid search
    // ---------------------------------------------------------

    let grid_results = grid_search_experiment(
        resolution, 
        &exp
    );

    // ---------------------------------------------------------
    // Save & aggregate experiment results
    // ---------------------------------------------------------

    let experiment_results =
        SingleStationExperimentResults {
            ego: ego_results,
            grid_search: grid_results,
        };

    let results_path = save_single_station_results(
        &experiment_results,
        &exp, 
        filename,
        output_dir,
        &timestamp, 
        
    );

    ExperimentRun{
        timestamp: timestamp,
        output_dir: output_dir.to_string(),
        results_path}

}

pub fn run_multi_station_experiment(
    max_iterations: usize,
    filename: &str,
    output_dir: &str,
    exp: ExperimentConfig,
)-> ExperimentRun {

    println!("Experiment type: MULTI-STATION experiment");
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
            &exp
        );

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

    let experiment_results =
        MultiStationExperimentResults {
            ego: ego_results,
            specialist: specialist_results,
        };

    let results_path = save_multi_station_results(
        &experiment_results,
        &exp,
        filename,
        output_dir,
        &timestamp, 
    );

    ExperimentRun{
        timestamp: timestamp,
        output_dir: output_dir.to_string(),
        results_path}
}