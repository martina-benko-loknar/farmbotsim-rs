
use crate::environment::{
    obstacle::Obstacle, 
    scene_config::SceneConfig,
    };
use crate::experiment::evaluation::evaluate_station_layout;
use ndarray::{Array2, ArrayView2};

use crate::optimization::station_positions::StationPositions;
use crate::experiment::config::ExperimentConfig;
use crate::experiment::models::EvaluatedCandidate;

// Context struct to hold optimization parameters
#[derive(Clone)]
pub struct OptimizationContext {
    pub obstacles: Vec<Obstacle>,
    pub n_stations: usize,
    pub max_iterations: usize,   
    pub scene_config: SceneConfig,
    pub experiment_config: ExperimentConfig,
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
            &context.obstacles,
            context.n_stations,
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