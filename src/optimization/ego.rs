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

use egobox_ego::EgorBuilder;
use egui::Pos2;
use ndarray::{Array2, ArrayView2};
use std::sync::{Arc, RwLock};
use std::time::Instant;

use crate::optimization::results::{self, EvaluationRecord};

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
    x: &ndarray::ArrayView1<f64>,
    energy: f64,
    best_energy: &Arc<RwLock<f64>>,
    best_positions: &Arc<RwLock<Vec<(f32, f32)>>>,
) -> EvaluationRecord {
    let stations: Vec<(f64, f64)> = x
        .iter()
        .cloned()
        .collect::<Vec<_>>()
        .chunks(2)
        .map(|c| (c[0], c[1]))
        .collect();

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
    let phase_label = match phase {
        EvalPhase::Init => "INIT | Eval",
        EvalPhase::BO => "EGO  | Iter",
    };

    print!(
        "[{} {:>3}]   E = {:>7.2} kWh | ",
        phase_label,
        phase_iter,
        energy / 1000.0
    );

    for (i, (sx, sy)) in stations.iter().enumerate() {
        print!("S{}({:>5.2},{:>5.2}) ", i + 1, sx, sy);
    }

    if is_new_best {
        print!(" <-- New best: {:.2} kWh", energy / 1000.0);
    }

    println!();

    // ---------------- RECORD ----------------
    EvaluationRecord {
        evaluation: eval_id,
        phase: match phase {
            EvalPhase::Init => "init".to_string(),
            EvalPhase::BO => "ego".to_string(),
        },
        phase_iteration: phase_iter,
        energy,
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
    max_iterations: usize,
    output_dir: &str,
) -> (StationPositions, String) {
    let start_time = Instant::now();

    // ------------------------------
    // Load config
    // ------------------------------
    let scene_config: SceneConfig =
        load_json_or_panic(DEFAULT_SCENE_CONFIG_PATH.to_string());

    let n_stations = scene_config.station_configs.len();

    let field_config: FieldConfig =
        load_json_or_panic(scene_config.field_config_path);

    let obstacles = field_config.get_obstacles();

    // ------------------------------
    // Bounds
    // ------------------------------
    let mut bounds_vec = Vec::with_capacity(n_stations * 4);

    for _ in 0..n_stations {
        bounds_vec.push(FIELD_MIN_X as f64 + STATION_MARGIN as f64);
        bounds_vec.push(FIELD_MAX_X as f64 - STATION_MARGIN as f64);
        bounds_vec.push(FIELD_MIN_Y as f64 + STATION_MARGIN as f64);
        bounds_vec.push(FIELD_MAX_Y as f64 - STATION_MARGIN as f64);
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
        100,
        &mut rng,
    );

    println!("Stations         : {}", n_stations);
    println!("Max iterations   : {}", max_iterations);
    println!("Initial samples  : {}", initial_positions.len());

    // println!("\n====== Phase 1 : INITIAL SAMPLING ====================================");

    // for (i, pos) in initial_positions.iter().take(3).enumerate() {
    //     println!("  Sample {}: {:?}", i + 1, pos.station_positions);
    // }

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

    // ------------------------------
    // Objective (PURE)
    // ------------------------------
    let context = OptimizationContext {
        obstacles: obstacles.clone(),
        n_stations,
        max_iterations,
    };

    // let evaluated_positions_obj = Arc::clone(&evaluated_positions);

    let objective_fn = move |x: &ArrayView2<f64>| {
        station_objective_function(x, &context, &mut Vec::new())
    };

    // ------------------------------
    // WRAPPER (logging layer)
    // ------------------------------
    let eval_counter_wrapped = Arc::clone(&eval_counter);
    let best_energy_wrapped = Arc::clone(&best_energy);
    let best_positions_wrapped = Arc::clone(&best_positions);
    let history_wrapped = Arc::clone(&evaluation_history);

    let n_initial = initial_x.nrows();

    let objective_wrapped = move |x: &ArrayView2<f64>| {
        let y = objective_fn(x);

        for (i, row) in x.rows().into_iter().enumerate() {
            let mut counter = eval_counter_wrapped.write().unwrap();
            *counter += 1;
            let eval_id = *counter;
            drop(counter);

            let energy = y[[i, 0]];

            let (phase, phase_iter) = if eval_id <= n_initial {
                if eval_id == 1 {
                    println!("\n====== Phase 1 : INITIAL SAMPLING ====================================");
                    println!("Evaluating {} initial samples...\n", n_initial);
                }
                (EvalPhase::Init, eval_id)
            } else {
                if eval_id - n_initial == 1 {
                    println!("\n====== Phase 2 : BAYESIAN OPTIMIZATION ================================");
                    println!("Starting optimization...\n");
                }
                (EvalPhase::BO, eval_id - n_initial)
            };

            let record = log_evaluation(
                phase,
                eval_id,
                phase_iter,
                &row,
                energy,
                &best_energy_wrapped,
                &best_positions_wrapped,
            );

            history_wrapped.write().unwrap().push(record);
        }

        y
    };

    // ------------------------------
    // Run optimization
    // ------------------------------
    // println!("\n====== Phase 2 : BAYESIAN OPTIMIZATION ================================");
    // println!("Starting optimization...\n");

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

            let mut violation = 0.0;

            if x.len() != n_stations * 2 {
                return 1.0;
            }

            for i in 0..n_stations {
                let x_coord = x[i * 2] as f32;
                let y_coord = x[i * 2 + 1] as f32;

                let x_clamped =
                    x_coord.clamp(FIELD_MIN_X + STATION_MARGIN, FIELD_MAX_X - STATION_MARGIN);
                let y_clamped =
                    y_coord.clamp(FIELD_MIN_Y + STATION_MARGIN, FIELD_MAX_Y - STATION_MARGIN);

                let position =
                    round_to_centimeters(Pos2::new(x_clamped, y_clamped));

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
        Ok(opt) => {
            println!("\n====== RESULTS ========================================================");

            let best_positions = StationPositions::from_optimization_vector(
                &opt.x_opt.view().insert_axis(ndarray::Axis(0)),
                &obstacles,
                n_stations,
            );

            println!(
                "Optimization time: {:.2?}",
                elapsed
            );

            println!("Best energy      : {:.3} kWh", opt.y_opt[0] / 1000.0);

            for (i, p) in best_positions.station_positions.iter().enumerate() {
                println!("  S{} → ({:.2}, {:.2})", i + 1, p.x, p.y);
            }

            // let evaluated = evaluated_positions.read().unwrap();

            // let mut best = f64::INFINITY;
            // let mut convergence = Vec::new();

            // for (i, (_pos, energy)) in evaluated.iter().enumerate() {
            //     if *energy < best {
            //         best = *energy;
            //     }

            //     convergence.push((i + 1, best, _pos.clone()));
            // }
            let history = evaluation_history.read().unwrap();

            let json_path = results::save_results(
                &history,
                max_iterations,
                history.len(),
                elapsed,
                output_dir,
            );

            (best_positions, json_path)
        }

        Err(e) => {
            eprintln!("EGO optimization failed: {:?}", e);

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

            (fallback, String::from(""))
        }
    }
}