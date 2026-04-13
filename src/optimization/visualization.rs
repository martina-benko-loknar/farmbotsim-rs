use crate::cfg::{OPTIMIZATION_RESULTS_PATH};
use crate::environment::{
    obstacle::Obstacle
};

use plotly::{Plot, Scatter3D, ImageFormat};
use plotly::layout::{Layout, Axis};
use plotly::common::{Marker, Mode};

pub fn visualize_optimization_results(
    evaluated_positions: &[(Vec<(f32, f32)>, f64)],
    obstacles: &[Obstacle],
) {
    let n_stations = evaluated_positions[0].0.len(); // Number of charging stations

    // Calculate the minimum energy and define an outlier threshold
    let min_energy = evaluated_positions
        .iter()
        .map(|(_, energy)| *energy)
        .fold(f64::INFINITY, f64::min);
    let threshold = min_energy * 1.5; 

    // Filter out outlier points
    let filtered_positions: Vec<_> = evaluated_positions
        .iter()
        .filter(|(_, energy)| *energy <= threshold)
        .cloned()
        .collect();


    let mut station_traces = Vec::new();

    // Separate positions and energies for each station
    let mut station_x: Vec<Vec<f32>> = vec![Vec::new(); n_stations];
    let mut station_y: Vec<Vec<f32>> = vec![Vec::new(); n_stations];
    let mut station_z: Vec<Vec<f64>> = vec![Vec::new(); n_stations];

    for (positions, energy) in filtered_positions.iter() {
        for (station_idx, (px, py)) in positions.iter().enumerate() {
            station_x[station_idx].push(*px);
            station_y[station_idx].push(*py);
            station_z[station_idx].push(*energy);
        }
    }

    // Assign a distinct color to each station
    let colors = vec!["blue", "green", "orange", "purple", "cyan", "magenta"];
    for station_idx in 0..n_stations {
        let trace = Scatter3D::new(
            station_x[station_idx].clone(),
            station_y[station_idx].clone(),
            station_z[station_idx].clone(),
        )
        .mode(Mode::Markers)
        .marker(
            Marker::new()
                .size(5)
                .color(colors[station_idx % colors.len()]), // Cycle through colors
        )
        .name(format!("Station {}", station_idx + 1)); // Add legend entry for the station
        station_traces.push(trace);
    }

    // Find the optimal point (minimum energy)
    let (optimal_positions, optimal_energy) = evaluated_positions
        .iter()
        .min_by(|a, b| a.1.partial_cmp(&b.1).unwrap())
        .unwrap();

    let optimal_x: Vec<f32> = optimal_positions.iter().map(|(px, _)| *px).collect();
    let optimal_y: Vec<f32> = optimal_positions.iter().map(|(_, py)| *py).collect();
    let optimal_z: Vec<f64> = vec![*optimal_energy; optimal_positions.len()];

    // Create a 3D scatter plot for the optimal point
    let trace_optimal = Scatter3D::new(optimal_x, optimal_y, optimal_z)
        .mode(Mode::Markers)
        .marker(
            Marker::new()
                .size(10) // Larger size for the optimal point
                .color("red"), // Use a distinct color for the optimal point
        )
        .name("Optimal Point"); // Add a legend entry for the optimal point

    // Create traces for obstacles
    let mut obstacle_traces = Vec::new();
    for obstacle in obstacles {
        let mut obstacle_x = Vec::new();
        let mut obstacle_y = Vec::new();
        let mut obstacle_z = Vec::new();

        // Add each edge of the obstacle
        for point in &obstacle.points {
            obstacle_x.push(point.x);
            obstacle_y.push(point.y);
            obstacle_z.push(*optimal_energy); // Set z to the optimal energy value
        }

        // Close the polygon by connecting the last point to the first
        if let Some(first_point) = obstacle.points.first() {
            obstacle_x.push(first_point.x);
            obstacle_y.push(first_point.y);
            obstacle_z.push(*optimal_energy);
        }

        // Create a trace for the obstacle
        let trace_obstacle = Scatter3D::new(obstacle_x, obstacle_y, obstacle_z)
            .mode(Mode::Lines)
            .line(plotly::common::Line::new().color("black").width(2.0))
            .show_legend(false)
            .name("Obstacle");

        obstacle_traces.push(trace_obstacle);
    }

    // Create the plot
    let mut plot = Plot::new();
    for trace in station_traces {
        plot.add_trace(trace);
    }
    plot.add_trace(trace_optimal);
    for trace in obstacle_traces {
        plot.add_trace(trace);
    }

    // Set plot layout with increased size
    plot.set_layout(
        Layout::new()
            .title("Charging Station Position Optimization with Obstacles")
            .width(1200)
            .height(800)
            .x_axis(Axis::new().title("X Coordinate [m]"))
            .y_axis(Axis::new().title("Y Coordinate [m]"))
            .z_axis(Axis::new().title("Energy Consumption [Wh]")),
    );

    // Save the plot to an HTML file => concat timestamp with path
    let timestamp = chrono::Utc::now().format("%Y%m%d_%H%M%S");
    let filename_html = format!("{}{}.html", OPTIMIZATION_RESULTS_PATH, timestamp);
    let filename_svg = format!("{}{}.svg", OPTIMIZATION_RESULTS_PATH, timestamp);
    
    plot.write_html(&filename_html);
    if let Err(e) = plot.write_image(&filename_svg, ImageFormat::SVG, 860, 800, 1.0) {
        eprintln!("Failed to write SVG file: {}", e);
        println!("Optimization plot saved to: {}", filename_html);
    } else {
        println!("Optimization plot saved to: {} and {}", filename_html, filename_svg);
    }
}

/// Generate convergence analysis plot
pub fn generate_convergence_plot(convergence_history: &[(usize, f64, Vec<(f32, f32)>)]) {
    if convergence_history.is_empty() {
        println!("No convergence data available for plotting");
        return;
    }

    let iterations: Vec<f64> = convergence_history.iter().map(|(iter, _, _)| *iter as f64).collect();
    let best_energies: Vec<f64> = convergence_history.iter().map(|(_, energy, _)| *energy).collect();
    
    // Create convergence plot showing best solution over iterations
    let trace_convergence = plotly::Scatter::new(iterations.clone(), best_energies.clone())
        .mode(plotly::common::Mode::LinesMarkers)
        .line(plotly::common::Line::new().color("#ff0000").width(3.0))
        .marker(plotly::common::Marker::new().size(6).color("#ff0000"))
        .name("Best Solution Convergence");
    
    let mut plot = Plot::new();
    plot.add_trace(trace_convergence);
    
    let layout = Layout::new()
        .title("EGO Optimization Convergence Analysis")
        .width(1000)
        .height(600)
        .font(plotly::common::Font::new().size(14).family("Arial"))
        .x_axis(plotly::layout::Axis::new()
            .title("Iteration Number")
            .tick_font(plotly::common::Font::new().size(12)))
        .y_axis(plotly::layout::Axis::new()
            .title("Best Energy Consumption (Wh)")
            .tick_font(plotly::common::Font::new().size(12)))
        .legend(
            plotly::layout::Legend::new()
                .font(plotly::common::Font::new().size(12))
        );
    
    plot.set_layout(layout);
    
    let timestamp = chrono::Utc::now().format("%Y%m%d_%H%M%S");
    let filename_html = format!("{}{}_convergence.html", OPTIMIZATION_RESULTS_PATH, timestamp);
    let filename_svg = format!("{}{}_convergence.svg", OPTIMIZATION_RESULTS_PATH, timestamp);
    
    plot.write_html(&filename_html);
    if let Err(e) = plot.write_image(&filename_svg, ImageFormat::SVG, 860, 800, 1.0) {
        eprintln!("Failed to write SVG file: {}", e);
        println!("Convergence plot saved to: {}", filename_html);
    } else {
        println!("Convergence plot saved to: {} and {}", filename_html, filename_svg);
    }
    println!("Convergence summary:");
    println!("  Initial best: {:.2} Wh", best_energies.first().unwrap_or(&0.0));
    println!("  Final best: {:.2} Wh", best_energies.last().unwrap_or(&0.0));
    if let (Some(initial), Some(final_val)) = (best_energies.first(), best_energies.last()) {
        let improvement = ((initial - final_val) / initial) * 100.0;
        println!("  Improvement: {:.1}%", improvement);
    }
}