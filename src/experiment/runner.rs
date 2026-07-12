//use plotly::layout::LayoutScene;

use crate::experiment::config::ExperimentConfig;
use crate::experiment::grid_search::grid_search_experiment;
use crate::experiment::evaluation::evaluate_station_layout;
use crate::experiment::export::{
    save_grid_search_results,
    save_single_station_results,
    save_ego_results,
    save_multi_station_results,
    save_single_evaluation_results};
use crate::experiment::station_layouts::{
    specialist_layouts,
    evaluate_station_layouts};
use crate::optimization::ego::optimize_station_positions_ego;
use crate::experiment::models::{
    SingleStationExperimentResults,
    MultiStationExperimentResults,
    ExperimentRun,
    ExperimentInfo
};

use egui::Pos2;

pub fn run_ego_experiment(
    n_stations: usize,
    max_iterations: usize,
    filename: &str,
    output_dir: &str,
    exp: ExperimentConfig,
    info: ExperimentInfo,
) -> ExperimentRun {

    println!("Experiment type: EGO ({} stations, {} iterations max)\n", 
            n_stations, 
            max_iterations);

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
        &info,
        filename,
        output_dir,
    );

    println!("EGO optimization experiment completed.");

    ExperimentRun{
        timestamp: info.timestamp,
        output_dir: output_dir.to_string(),
        results_path}

}

pub fn run_grid_search_experiment(
    resolution: usize,
    filename: &str,
    output_dir: &str,
    exp: ExperimentConfig,
    info: ExperimentInfo,
) -> ExperimentRun {

    println!("Experiment type: GRID SEARCH ({}×{} resolution)\n", 
            resolution, 
            resolution);

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
        &info,
        filename,
        output_dir,
    );

    ExperimentRun{
        timestamp: info.timestamp,
        output_dir: output_dir.to_string(),
        results_path}
}

pub fn run_single_station_experiment(
    resolution: usize,
    filename: &str,
    output_dir: &str,
    exp: ExperimentConfig,
    info: ExperimentInfo,
) -> (ExperimentRun, Pos2) {

    println!("Experiment type: SINGLE-STATION (grid search + EGO)");

    // ---------------------------------------------------------
    // Phase 1: EGO optimization
    // ---------------------------------------------------------

    println!("\n===========  EGO optimization ===========");
    let ego_results =
        optimize_station_positions_ego(
            1,
            30,
            &exp);

    // ---------------------------------------------------------
    // Phase 2: Grid search
    // ---------------------------------------------------------
    println!("\n=========== Grid search ===========");
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

    let best_position = experiment_results.best_position();

    let results_path = save_single_station_results(
        &experiment_results,
        &exp,
        &info,
        filename,
        output_dir,
    );

    (ExperimentRun{
        timestamp: info.timestamp,
        output_dir: output_dir.to_string(),
        results_path}, best_position)

}

pub fn run_multi_station_experiment(
    max_iterations: usize,
    filename: &str,
    output_dir: &str,
    exp: ExperimentConfig,
    info: ExperimentInfo,
)-> ExperimentRun {

    println!("Experiment type: MULTI-STATION (EGO + specialized layouts)");

    // ------------------------------
    // Load config
    // ------------------------------
    let scene_config = exp.load_scene_config();
    let field_config = exp.load_field_config();

    // ---------------------------------------------------------
    // Phase 1: EGO optimization
    // ---------------------------------------------------------
    println!("\n===========  EGO optimization ===========");
    let ego_results =
        optimize_station_positions_ego(
            2,
            max_iterations,
            &exp
        );

    // ---------------------------------------------------------
    // Phase 2: Specialist layouts
    // ---------------------------------------------------------
    println!("\n===========  Specialist layouts ===========");
    let layouts = 
        specialist_layouts(&field_config);

    let specialist_results = 
        evaluate_station_layouts(
            &layouts, 
            &scene_config, 
            &exp);

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
        &info,
        filename,
        output_dir,
    );

    ExperimentRun{
        timestamp: info.timestamp,
        output_dir: output_dir.to_string(),
        results_path}
}

pub fn run_single_evaluation(
    station_position: Pos2,
    filename: &str,
    output_dir: &str,
    exp: ExperimentConfig,
    info: ExperimentInfo,
) -> ExperimentRun {

    println!("Experiment type: SINGLE EVALUATION");

    let scene_config = exp.load_scene_config();
    let stations = vec![station_position];

    // ---------------------------------------------------------
    // Single evaluation
    // ---------------------------------------------------------

    let metrics = evaluate_station_layout(
        &stations,
        &scene_config,
        &exp,
    );

    // ---------------------------------------------------------
    // Save experiment results
    // ---------------------------------------------------------

    let results_path = save_single_evaluation_results(
        &metrics,
        &exp,
        &info,
        filename,
        output_dir,
    );

    println!("Single evaluation experiment completed.");

    ExperimentRun{
        timestamp: info.timestamp,
        output_dir: output_dir.to_string(),
        results_path}
}