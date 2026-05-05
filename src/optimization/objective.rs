
use crate::environment::{
    obstacle::Obstacle
};
use ndarray::{Array2, ArrayView2};

use crate::optimization::station_positions::StationPositions;

// Context struct to hold optimization parameters
#[derive(Clone)]
pub struct OptimizationContext {
    pub obstacles: Vec<Obstacle>,
    pub n_stations: usize,
    pub max_iterations: usize,   
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
        let energy = positions.evaluate();

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

    // println!(
    //     "Batch done: {} candidates | best in batch = {:.2} Wh",
    //     x.nrows(),
    //     current_best
    // );

    // Update convergence history with the best value found in this evaluation batch
    // let current_iteration = convergence_history.len() + 1;
    
    // // Get the global best so far (including previous iterations)
    // let (global_best_energy, global_best_positions) = if let Some((_, prev_best_energy, prev_best_positions)) = convergence_history.last() {
    //     if current_best < *prev_best_energy {
    //         (current_best, current_best_positions)
    //     } else {
    //         (*prev_best_energy, prev_best_positions.clone())
    //     }
    // } else {
    //     (current_best, current_best_positions)
    // };
    
    // convergence_history.push((current_iteration, global_best_energy, global_best_positions));


    // println!("Iteration {}: Best energy so far: {:.2} Wh", current_iteration, global_best_energy);

    // let progress = (current_iteration as f32 / context.max_iterations as f32) * 100.0;

    // println!(
    //     "[{:>3}%] ({}/{}) | global best energy = {:.2} Wh",
    //     progress.round() as i32,
    //     current_iteration,
    //     context.max_iterations,
    //     global_best_energy
    // );

    // if current_iteration == 1 {
    //     println!("====== Phase 2 : BAYESIAN OPTIMIZATION ================================");
    // }
    // //
    // println!(
    //     "[{:>3}% | iter {}/{}] best = {:.2} Wh | batch best = {:.2} Wh | candidates  = {}",
    //     progress.round() as i32,
    //     current_iteration,
    //     context.max_iterations,
    //     global_best_energy,
    //     current_best,
    //     y.nrows()
    // );

    y
}