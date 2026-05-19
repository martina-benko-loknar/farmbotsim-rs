
use crate::environment::{
    obstacle::Obstacle, 
    scene_config::SceneConfig,
    };
use crate::experiment::evaluation::evaluate_station_layout;
use ndarray::{Array2, ArrayView2};

use crate::optimization::station_positions::StationPositions;

// Context struct to hold optimization parameters
#[derive(Clone)]
pub struct OptimizationContext {
    pub obstacles: Vec<Obstacle>,
    pub n_stations: usize,
    pub max_iterations: usize,   
    pub scene_config: SceneConfig,
}

// Objective function for EGO optimization 
pub fn station_objective_function(
    x: &ArrayView2<f64>,
    context: &OptimizationContext,
    evaluated_positions: &mut Vec<(Vec<(f32, f32)>, f64)>,
    //convergence_history: &mut Vec<(usize, f64, Vec<(f32, f32)>)>, // Track (iteration, best_energy, best_positions)
) -> Array2<f64> {

    let mut y: Array2<f64> = Array2::zeros((x.nrows(), 1));

    let mut current_best = f64::INFINITY;
    let mut _current_best_positions = Vec::new();

    for (i, xi) in x.rows().into_iter().enumerate() {

        let positions = StationPositions::from_optimization_vector(
            &xi.insert_axis(ndarray::Axis(0)),
            &context.obstacles,
            context.n_stations,
        );

        let evaluation = evaluate_station_layout(
            &positions.station_positions, 
            &context.scene_config);

        let energy = evaluation.energy;

        // Update current best for this batch
        if energy < current_best {
            current_best = energy;
            _current_best_positions = positions
                .station_positions
                .iter()
                .map(|p| (p.x, p.y))
                .collect();
        }

        // let total = x.nrows();

        // if i % 10 == 0 {
        //         println!("evaluating candidate {}/{}", i, total);
        //     }
        // Log the evaluation
        //println!("Evaluated positions: {} -> Energy: {:.2} Wh", positions, energy);

        // Store the evaluated positions and energy
        evaluated_positions.push((
            positions
                .station_positions
                .iter()
                .map(|p| (p.x, p.y))
                .collect(),
            energy,
        ));

        y[[i, 0]] = energy;
    }

    y
}