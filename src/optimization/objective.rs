
use crate::environment::scene_config::SceneConfig;
use crate::experiment::evaluation::evaluate_station_layout;
use ndarray::{Array2, ArrayView2};
use egui::Pos2;

use crate::optimization::station_positions::StationPositions;
use crate::experiment::config::ExperimentConfig;
use crate::experiment::models::EvaluatedCandidate;
use crate::experiment::models::SocEvaluatedCandidate;
use crate::experiment::search_domain::SearchDomain;

// Context struct to hold optimization parameters
#[derive(Clone)]
pub struct OptimizationContext {
    pub n_stations: usize,
    pub max_iterations: usize,
    pub scene_config: SceneConfig,
    pub experiment_config: ExperimentConfig,
    pub domain: SearchDomain,
}

// Objective function for EGO optimization 
pub fn station_objective_function(
    x: &ArrayView2<f64>,
    context: &OptimizationContext,
    evaluated_candidates: &mut Vec<EvaluatedCandidate>,
    //convergence_history: &mut Vec<(usize, f64, Vec<(f32, f32)>)>, // Track (iteration, best_energy, best_positions)
) -> Array2<f64> {

    let mut y: Array2<f64> = Array2::zeros((x.nrows(), 1));

    for (i, xi) in x.rows().into_iter().enumerate() {

        let positions = StationPositions::from_optimization_vector(
            &xi.insert_axis(ndarray::Axis(0)),
            context.n_stations,
            &context.domain,
        );

        let metrics = evaluate_station_layout(
            &positions.station_positions, 
            &context.scene_config,
            &context.experiment_config,
        );

        let coords = positions
            .station_positions
            .iter()
            .map(|p| (p.x, p.y))
            .collect::<Vec<(f32, f32)>>();

        evaluated_candidates.push(EvaluatedCandidate {
            positions: coords,
            metrics: metrics.clone(),
        });

        y[[i, 0]] = metrics.energy_wh;
    }

    y
}

// ============================================================
// SoC-threshold optimization (Level II): station position is held fixed
// at the Level-I-optimal layout; the decision variable is the SoC
// threshold that triggers opportunistic charging.
// ============================================================
#[derive(Clone)]
pub struct SocOptimizationContext {
    pub max_iterations: usize,
    pub scene_config: SceneConfig,
    pub experiment_config: ExperimentConfig,
    pub fixed_stations: Vec<Pos2>,
}

pub fn soc_threshold_objective_function(
    x: &ArrayView2<f64>,
    context: &SocOptimizationContext,
    evaluated_candidates: &mut Vec<SocEvaluatedCandidate>,
) -> Array2<f64> {

    let mut y: Array2<f64> = Array2::zeros((x.nrows(), 1));

    for (i, xi) in x.rows().into_iter().enumerate() {

        let threshold_percent = xi[0] as f32;

        let mut exp = context.experiment_config.clone();
        exp.soc_threshold_percent = threshold_percent;

        let metrics = evaluate_station_layout(
            &context.fixed_stations,
            &context.scene_config,
            &exp,
        );

        evaluated_candidates.push(SocEvaluatedCandidate {
            threshold_percent,
            metrics: metrics.clone(),
        });

        y[[i, 0]] = metrics.energy_wh;
    }

    y
}