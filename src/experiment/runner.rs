//use plotly::layout::LayoutScene;

use crate::experiment::config::ExperimentConfig;
use crate::experiment::grid_search::grid_search_experiment;
use crate::experiment::evaluation::evaluate_station_layout;
use crate::experiment::export::{
    save_grid_search_results,
    save_single_station_results,
    save_ego_results,
    save_multi_station_results,
    save_single_evaluation_results,
    save_soc_threshold_results};
use crate::experiment::station_layouts::{
    specialist_layouts,
    evaluate_station_layouts};
use crate::optimization::ego::{optimize_station_positions_ego, optimize_soc_threshold_ego};
use crate::experiment::models::{
    SingleStationExperimentResults,
    MultiStationExperimentResults,
    ExperimentRun,
    ExperimentInfo,
    ExperimentType,
};

use egui::Pos2;

pub fn run_ego_experiment(
    n_stations: usize,
    n_initial: usize,
    max_iterations: usize,
    filename: &str,
    output_dir: &str,
    exp: ExperimentConfig,
    info: ExperimentInfo,
) -> ExperimentRun {

    println!("Experiment type: EGO ({} stations, {} initial + {} BO iterations max)\n",
            n_stations,
            n_initial,
            max_iterations);

    // ---------------------------------------------------------
    // EGO optimization
    // ---------------------------------------------------------

    let ego_results = optimize_station_positions_ego(
            n_stations,
            n_initial,
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
    n_initial: usize,
    max_iterations: usize,
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
            n_initial,
            max_iterations,
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
    n_initial: usize,
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
            n_initial,
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

pub fn run_soc_threshold_experiment(
    max_iterations: usize,
    filename: &str,
    output_dir: &str,
    exp: ExperimentConfig,
    info: ExperimentInfo,
) -> ExperimentRun {

    println!("Experiment type: SOC-THRESHOLD OPTIMIZATION (Level II, {} BO iterations max)\n",
            max_iterations);

    // ---------------------------------------------------------
    // Phase 1: find the Level-I-optimal station position (grid search +
    // EGO), held fixed while optimizing the SoC threshold.
    // ---------------------------------------------------------
    println!("\n=========== Station placement (grid search + EGO) ===========");
    let (_, station_position) = run_single_station_experiment(
        exp.field_resolution,
        crate::optimization::constants::DEFAULT_EGO_INITIAL_SAMPLES,
        crate::optimization::constants::DEFAULT_EGO_MAX_ITERATIONS,
        &format!("{filename}_placement"),
        output_dir,
        exp.clone(),
        ExperimentInfo {
            experiment_type: ExperimentType::SingleStation,
            timestamp: info.timestamp.clone(),
        },
    );

    // ---------------------------------------------------------
    // Phase 2: SoC-threshold optimization at the fixed position
    // ---------------------------------------------------------
    println!("\n=========== SoC-threshold optimization ===========");
    let soc_results = optimize_soc_threshold_ego(
        max_iterations,
        &[station_position],
        &exp,
    );

    // ---------------------------------------------------------
    // Save experiment results
    // ---------------------------------------------------------

    let results_path = save_soc_threshold_results(
        &soc_results,
        &exp,
        &info,
        filename,
        output_dir,
    );

    println!("SoC-threshold optimization experiment completed.");

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