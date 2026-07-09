use crate::cfg::DEFAULT_SCENE_CONFIG_PATH;
use crate::environment::{
    field_config::FieldConfig,
    scene_config::SceneConfig,
};
use crate::optimization::constants::*;
use crate::optimization::geometry::{is_position_valid, round_to_centimeters};
use crate::optimization::objective::{station_objective_function, OptimizationContext};
use crate::optimization::station_positions::StationPositions;
use crate::utilities::utils::load_json_or_panic;
use crate::experiment::models::{EgoOptimizationResults, EgoSummary, EgoTrace, ExperimentMetrics};
use crate::experiment::config::ExperimentConfig;
use crate::experiment::models::EvaluatedCandidate;
use crate::experiment::models::EvaluationRecord;


use egobox_ego::EgorBuilder;
use egui::Pos2;
use ndarray::{Array2, ArrayView2};
use std::sync::{Arc, RwLock};
use std::time::Instant;

use crate::environment::geometry::FieldBounds;

fn separator() {
    println!("{}", "-".repeat(70));
}

// ============================================================
// Phase enum
// ============================================================
#[derive(Clone, Copy)]
enum EvalPhase {
    Init,
    BO,
}

// ============================================================
// Logging + Record creation
// ============================================================
fn log_evaluation(
    phase: EvalPhase,
    eval_id: usize,
    phase_iter: usize,
    candidate: &EvaluatedCandidate,
    best_energy: &Arc<RwLock<f64>>,
    best_positions: &Arc<RwLock<Vec<(f32, f32)>>>,
) -> EvaluationRecord {

    let stations = &candidate.positions;
    let energy = candidate.metrics.energy_wh;

    let mut best_e = best_energy.write().unwrap();
    let mut best_pos = best_positions.write().unwrap();

    let is_new_best = energy < *best_e;

    if is_new_best {
        *best_e = energy;
        *best_pos = stations
            .iter()
            .map(|(x, y)| (*x as f32, *y as f32))
            .collect();
    }

    let best_energy_now = *best_e;
    let best_positions_now = best_pos.clone();

    // ---------------- PRINT ----------------

    let position_string = if stations.len() == 1 {
        format!("({:.2}, {:.2})", stations[0].0, stations[0].1)
    } else {
        stations
            .iter()
            .enumerate()
            .map(|(i, (x, y))| {
                format!("S{}({:.2},{:.2})", i + 1, x, y)
            })
            .collect::<Vec<_>>()
            .join(" ")
    };

    println!(
        "[{:>2}] | {} | {:.2} kWh | {:.2} km | {:.2} km | {:.2} s{}",
        phase_iter,
        position_string,
        candidate.metrics.energy_wh / 1000.0,
        candidate.metrics.total_distance_m / 1000.0,
        candidate.metrics.charging_distance_m / 1000.0,
        candidate.metrics.evaluation_time_sec,
        if is_new_best { "  <-- New best" } else { "" },
    );

    // ---------------- RECORD ----------------
    EvaluationRecord {
        evaluation: eval_id,
        phase: match phase {
            EvalPhase::Init => "init".to_string(),
            EvalPhase::BO => "ego".to_string(),
        },
        phase_iteration: phase_iter,
        metrics: candidate.metrics.clone(),
        best_energy: best_energy_now,
        is_new_best,
        positions: stations
            .iter()
            .map(|(x, y)| (*x as f32, *y as f32))
            .collect(),
        best_positions: best_positions_now,
    }
}

// ============================================================
// Main optimizer
// ============================================================
pub fn optimize_station_positions_ego(
    n_stations: usize,
    max_iterations: usize,
    exp: &ExperimentConfig,
) -> EgoOptimizationResults 
    {
    let start_time = Instant::now();

    // ------------------------------
    // Load config
    // ------------------------------
    let scene_config = exp.load_scene_config();
    let field_config = exp.load_field_config();

    let obstacles = field_config.get_obstacles();

    // ------------------------------
    // Bounds
    // ------------------------------
    let mut bounds_vec = Vec::with_capacity(n_stations * 4);

    let field_config: FieldConfig =
        load_json_or_panic(scene_config.field_config_path.clone());

    let field_bounds = FieldBounds::from_field_config(&field_config);

    for _ in 0..n_stations {

        bounds_vec.push(
            field_bounds.min_x as f64 + STATION_MARGIN as f64
        );

        bounds_vec.push(
            field_bounds.max_x as f64 - STATION_MARGIN as f64
        );

        bounds_vec.push(
            field_bounds.min_y as f64 + STATION_MARGIN as f64
        );

        bounds_vec.push(
            field_bounds.max_y as f64 - STATION_MARGIN as f64
        );
    }

    let bounds_array =
        Array2::from_shape_vec((n_stations * 2, 2), bounds_vec).unwrap();

    // ------------------------------
    // Initial sampling (DOE)
    // ------------------------------
    let mut rng = rand::rng();

    let initial_positions = StationPositions::generate_initial_population(
        &obstacles,
        n_stations,
        20,
        &mut rng,
    );

    let initial_x: Array2<f64> = Array2::from_shape_vec(
        (initial_positions.len(), n_stations * 2),
        initial_positions
            .iter()
            .flat_map(|pos| {
                pos.station_positions
                    .iter()
                    .flat_map(|p| vec![p.x as f64, p.y as f64])
            })
            .collect(),
    )
    .unwrap();

    // ------------------------------
    // Shared state
    // ------------------------------
    let eval_counter = Arc::new(RwLock::new(0usize));
    let best_energy = Arc::new(RwLock::new(f64::INFINITY));
    let best_positions = Arc::new(RwLock::new(Vec::<(f32, f32)>::new()));
    let evaluation_history = Arc::new(RwLock::new(Vec::<EvaluationRecord>::new()));
    let evaluated_candidates = Arc::new(RwLock::new(Vec::<EvaluatedCandidate>::new()));

    // ------------------------------
    // Objective + candidate storage
    // ------------------------------
    let context = OptimizationContext {
        obstacles: obstacles.clone(),
        n_stations,
        max_iterations,
        scene_config: scene_config.clone(),
        experiment_config: exp.clone(),
    };

    // let evaluated_positions_obj = Arc::clone(&evaluated_positions);

    let evaluated_candidates_for_objective = Arc::clone(&evaluated_candidates);

    let objective_fn = move |x: &ArrayView2<f64>| {
        let mut records = evaluated_candidates_for_objective
            .write()
            .unwrap();

        station_objective_function(
            x,
            &context,
            &mut records,
        )
    };

    // ------------------------------
    // WRAPPER (logging layer)
    // ------------------------------
    let eval_counter_wrapped = Arc::clone(&eval_counter);
    let best_energy_wrapped = Arc::clone(&best_energy);
    let best_positions_wrapped = Arc::clone(&best_positions);
    let history_wrapped = Arc::clone(&evaluation_history);
    let evaluated_candidates_wrapped = Arc::clone(&evaluated_candidates);

    let n_initial = initial_x.nrows();

    let objective_wrapped = move |x: &ArrayView2<f64>| {
               
        let start_idx = {
            evaluated_candidates_wrapped
            .read()
            .unwrap()
            .len()
        };
        let y = objective_fn(x);

        for (i, _row) in x.rows().into_iter().enumerate() {
            let mut counter = eval_counter_wrapped.write().unwrap();
            *counter += 1;
            let eval_id = *counter;
            drop(counter);

            let candidate = {
                let candidates = evaluated_candidates_wrapped
                    .read()
                    .unwrap();

                candidates
                    .get(start_idx + i)
                    .expect("Missing evaluated candidate")
                    .clone()
            };

            let (phase, phase_iter) = if eval_id <= n_initial {
                if eval_id == 1 {
                    println!("\nPhase 1 : initial sampling");
                    // println!("Evaluating {} initial samples...\n", n_initial);
                    println!("Eval | Position |  Energy | Total dist | Charging dist | Time");
                }
                (EvalPhase::Init, eval_id)
            } else {
                if eval_id - n_initial == 1 {
                    println!("\nPhase 2 : bayesian optimization");
                    // println!("Starting optimization...\n");
                    println!("Iter | Position |  Energy | Total dist | Charging dist | Time");
                }
                (EvalPhase::BO, eval_id - n_initial)
            };

            let record = log_evaluation(
                phase,
                eval_id,
                phase_iter,
                &candidate,
                &best_energy_wrapped,
                &best_positions_wrapped,
            );

            history_wrapped.write().unwrap().push(record);
        }

        y
    };
    
    let result = EgorBuilder::optimize(objective_wrapped)
        .configure(|config| config.max_iters(max_iterations).doe(&initial_x))
        .subject_to(vec![|x: &[f64], g: Option<&mut [f64]>, _u| {

            if let Some(g) = g {
                g[0] = 0.0;
            }

            let scene_config: SceneConfig =
                load_json_or_panic(DEFAULT_SCENE_CONFIG_PATH.to_string());

            let n_stations = scene_config.station_configs.len();

            let field_config: FieldConfig =
                load_json_or_panic(scene_config.field_config_path);

            let obstacles = field_config.get_obstacles();

            // -------------------------------------------------
            // Compute field bounds from obstacles
            // -------------------------------------------------

            let field_bounds = FieldBounds::from_field_config(&field_config);

            let mut violation = 0.0;

            if x.len() != n_stations * 2 {
                return 1.0;
            }

            for i in 0..n_stations {

                let x_coord = x[i * 2] as f32;
                let y_coord = x[i * 2 + 1] as f32;

                // ---------------------------------------------
                // Clamp using computed bounds
                // ---------------------------------------------
                let x_clamped = x_coord.clamp(
                    field_bounds.min_x + STATION_MARGIN,
                    field_bounds.max_x - STATION_MARGIN,
                );

                let y_clamped = y_coord.clamp(
                    field_bounds.min_y + STATION_MARGIN,
                    field_bounds.max_y - STATION_MARGIN,
                );

                let position =
                    round_to_centimeters(Pos2::new(
                        x_clamped,
                        y_clamped,
                    ));

                if !is_position_valid(position, &obstacles) {
                    violation += 1.0;
                }
            }

            violation
        }])
        .min_within(&bounds_array)
        .run();

    let elapsed = start_time.elapsed();

    // ------------------------------
    // Results
    // ------------------------------
    match result {
        Ok(_opt) => {
            //println!("\n====== RESULTS ========================================================");

            let candidates = evaluated_candidates.read().unwrap();

            let best_candidate = candidates
                .iter()
                .min_by(|a, b| {
                    a.metrics
                        .energy_wh
                        .partial_cmp(&b.metrics.energy_wh)
                        .unwrap()
                })
                .expect("No evaluated candidates found");

            let best_positions: Vec<Pos2> = best_candidate.positions
                .iter()
                .map(|(x, y)| Pos2::new(*x, *y))
                .collect();

            let history = evaluation_history.read().unwrap();
            let m = &best_candidate.metrics;

            separator();
            println!("Summary: ");
            println!("Evaluations: "); 
            println!("  - initial samples  : {}", n_initial);
            println!("  - BO iterations    : {}", max_iterations);
            println!("Best point: "); 
            println!("  - stations         : {}", best_positions.len());
            println!("  - station positions:");
            for (i, p) in best_positions.iter().enumerate() {
                println!("      S{}: ({:.2}, {:.2})", i + 1, p.x, p.y);
            }
            println!("  - energy           : {:.2} kWh", m.energy_wh / 1000.0);
            println!("  - distance         : {:.2} km", m.total_distance_m/1000.0); 
            println!("  - charging dist    : {:.2} km", m.charging_distance_m/1000.0); 
            println!("  - mission time     : {:.2} h", m.simulation_time_sec/3600.0); 
            println!("  - charging events  : {}", m.charging_events); 

            println!("Experiment time      : {:.2?} s",  elapsed);

            let summary = EgoSummary {
                best_metrics: best_candidate.metrics.clone(),
                optimal_position: best_positions.clone(),
                optimization_time_sec: elapsed.as_secs_f64(),
                total_evaluations: history.len(),
            };

            let trace = EgoTrace {
                evaluation_history: history.clone(),
                max_iterations,
            };

            EgoOptimizationResults { summary, trace }
        }

        Err(e) => {
            eprintln!("EGO optimization failed: {:?}", e);

            let history = evaluation_history.read().unwrap();
            let candidates = evaluated_candidates.read().unwrap();

            // ------------------------------------------------------------
            // Case 1: Use best already evaluated candidate
            // ------------------------------------------------------------
            if let Some(best_candidate) = candidates
                .iter()
                .min_by(|a, b| {
                    a.metrics
                        .energy_wh
                        .partial_cmp(&b.metrics.energy_wh)
                        .unwrap()
                })
            {
                println!("\nEGO failed. Returning best evaluated candidate.");

                let summary = EgoSummary {
                    best_metrics: best_candidate.metrics.clone(),
                    optimal_position: best_candidate.positions
                        .iter()
                        .map(|(x, y)| Pos2::new(*x, *y))
                        .collect(),
                    optimization_time_sec: elapsed.as_secs_f64(),
                    total_evaluations: history.len(),
                };

                return EgoOptimizationResults {
                    summary,
                    trace: EgoTrace {
                        evaluation_history: history.clone(),
                        max_iterations,
                    },
                };
            }

            // ------------------------------------------------------------
            // Case 2: No evaluations happened - generate fallback
            // ------------------------------------------------------------
            let mut rng = rand::rng();

            let fallback = StationPositions::generate_initial_population(
                &obstacles,
                n_stations,
                100,
                &mut rng,
            )
            .into_iter()
            .next()
            .unwrap();

            eprintln!("No evaluated candidates available. Using random fallback.");

            EgoOptimizationResults {
                summary: EgoSummary {
                    best_metrics: ExperimentMetrics {
                        energy_wh: f64::INFINITY,
                        total_distance_m: 0.0,
                        charging_distance_m: 0.0,
                        simulation_time_sec: 0.0,
                        evaluation_time_sec: 0.0,
                        charging_events: 0,
                        charge_attempts: 0,
                        failed_charge_attempts: 0,
                        completed_tasks: 0,
                    },
                    optimal_position: fallback.station_positions.clone(),
                    optimization_time_sec: elapsed.as_secs_f64(),
                    total_evaluations: 0,
                },

                trace: EgoTrace {
                    evaluation_history: vec![],
                    max_iterations,
                },
            }
        }
    }
}