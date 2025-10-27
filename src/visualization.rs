use crate::environment::obstacle::Obstacle;
use egui::Pos2;
use plotly::{Plot, Scatter, Scatter3D, HeatMap, Layout, common::{Marker, Mode}, ImageFormat};

/// Data structure to hold grid search results for visualization
#[derive(Debug, Clone)]
pub struct GridSearchResults {
    pub results: Vec<(Pos2, f64, f64, f64)>, // (position, energy, total_distance, charging_distance)
    pub grid_resolution: usize,
    pub obstacles: Vec<Obstacle>,
    pub field_bounds: (f32, f32, f32, f32), // (min_x, max_x, min_y, max_y)
}

/// Data structure for multi-station configuration results
#[derive(Debug, Clone)]
pub struct MultiStationResults {
    pub optimal_stations: Vec<Pos2>,
    pub optimal_energy: f64,
    pub optimal_distance: f64,
    pub suboptimal_configs_energy: Vec<(Vec<Pos2>, f64)>,
    pub suboptimal_configs_distance: Vec<(Vec<Pos2>, f64)>,
    pub obstacles: Vec<Obstacle>,
    pub field_bounds: (f32, f32, f32, f32),
}

// ============================================================================
// LaTeX-style Font Helper Functions
// ============================================================================

/// Create a LaTeX-style font with specified size
fn latex_font(size: usize) -> plotly::common::Font {
    plotly::common::Font::new()
        .size(size)
        .family("Latin Modern Roman")
        .color("black")
}

// ============================================================================

/// Generate all plots for grid search results
pub fn generate_all_grid_plots(
    results: &GridSearchResults,
    optimization_minimum: Option<(Pos2, f64)>
) {
    generate_3d_plot(&results.results, &results.obstacles, results.grid_resolution, optimization_minimum);
    generate_energy_heatmap_plot(&results.results, &results.obstacles, results.grid_resolution, optimization_minimum);
    generate_distance_heatmap_plot(&results.results, &results.obstacles, results.grid_resolution, optimization_minimum);
    generate_charging_distance_heatmap_plot(&results.results, &results.obstacles, results.grid_resolution, optimization_minimum);
}

/// Generate all plots for multi-station results
pub fn generate_all_multi_station_plots(results: &MultiStationResults) {
    generate_multi_station_plot(
        &results.optimal_stations,
        results.optimal_energy,
        &results.suboptimal_configs_energy,
        &results.obstacles,
        results.field_bounds,
    );
    
    generate_multi_station_distance_plot(
        &results.optimal_stations,
        results.optimal_distance,
        &results.suboptimal_configs_distance,
        &results.obstacles,
        results.field_bounds,
    );
}

/// Generate 3D scatter plot (similar to optimization.rs)
pub fn generate_3d_plot(
    results: &[(Pos2, f64, f64, f64)], 
    obstacles: &[Obstacle], 
    grid_resolution: usize, 
    optimization_minimum: Option<(Pos2, f64)>
) {
    let x_coords: Vec<f64> = results.iter().map(|(pos, _, _, _)| pos.x as f64).collect();
    let y_coords: Vec<f64> = results.iter().map(|(pos, _, _, _)| pos.y as f64).collect();
    let energy_values: Vec<f64> = results.iter().map(|(_, energy, _, _)| *energy).collect();
    
    // Find the optimal (minimum) energy value for obstacle z-level
    let min_energy = energy_values.iter().fold(f64::INFINITY, |a, &b| a.min(b));
    
    let trace = Scatter3D::new(x_coords, y_coords, energy_values)
        .mode(Mode::Markers)
        .marker(Marker::new().size(4).color("blue"));
    
    let mut plot = Plot::new();
    plot.add_trace(trace);
    
    // Add optimization minimum point if provided
    if let Some((opt_pos, opt_value)) = optimization_minimum {
        // Add white padding cross first (larger, underneath)
        let padding_trace = Scatter3D::new(
            vec![opt_pos.x as f64], 
            vec![opt_pos.y as f64], 
            vec![opt_value]
        )
        .mode(Mode::Markers)
        .name("Minimum Padding")
        .show_legend(false)
        .marker(
            Marker::new()
                .size(20)
                .color("white")
                .symbol(plotly::common::MarkerSymbol::X)
                .line(plotly::common::Line::new().width(3.0).color("white"))
        );
        
        plot.add_trace(padding_trace);
        
        // Add black cross on top (thinner)
        let opt_trace = Scatter3D::new(
            vec![opt_pos.x as f64], 
            vec![opt_pos.y as f64], 
            vec![opt_value]
        )
        .mode(Mode::Markers)
        .name("Optimization Minimum")
        .marker(
            Marker::new()
                .size(15)
                .color("black")
                .symbol(plotly::common::MarkerSymbol::X)
                .line(plotly::common::Line::new().width(2.0).color("black"))
        );
        
        plot.add_trace(opt_trace);
    }
    
    // Add obstacle boundaries as 3D traces
    add_obstacles_to_3d_plot(&mut plot, obstacles, min_energy);
    
    let layout = Layout::new()
        .title(&format!("Grid Search Results - 3D View ({}x{})", grid_resolution, grid_resolution))
        .width(1200)
        .height(800)
        .x_axis(plotly::layout::Axis::new()
            .title("x (m)")
            .tick_font(latex_font(16)))
        .y_axis(plotly::layout::Axis::new()
            .title("y (m)")
            .tick_font(latex_font(16)))
        .legend(plotly::layout::Legend::new()
            .font(latex_font(16)));
    
    plot.set_layout(layout);
    
    save_plot(&plot, &format!("grid_search_{}x{}_3d", grid_resolution, grid_resolution));
}

/// Generate 2D heatmap plot for energy consumption with interpolation
pub fn generate_energy_heatmap_plot(
    results: &[(Pos2, f64, f64, f64)], 
    obstacles: &[Obstacle], 
    grid_resolution: usize, 
    optimization_minimum: Option<(Pos2, f64)>
) {
    let energy_results: Vec<(Pos2, f64)> = results.iter()
        .map(|(pos, energy, _, _)| (*pos, *energy))
        .collect();
    
    let interp_resolution = 1000;
    let (x_grid, y_grid, z_grid) = interpolate_results(&energy_results, obstacles, interp_resolution);
    
    let heatmap = HeatMap::new(x_grid, y_grid, z_grid)
        .color_bar(
            plotly::common::ColorBar::new()
                // .title("Total<br>Energy<br>Consumption<br>(Wh)")
                .title("E<sub>tot</sub> (W h)")
                .tick_font(latex_font(18))
        );
    
    let mut plot = Plot::new();
    plot.add_trace(heatmap);
    
    add_obstacles_to_2d_plot(&mut plot, obstacles);
    add_optimization_minimum_to_2d_plot(&mut plot, optimization_minimum);
    
    // Create layout with optional annotation
    let mut annotations = Vec::new();
    if let Some(annotation) = create_minimum_annotation(optimization_minimum, "minimum", 1.0, 18) {
        annotations.push(annotation);
    }
    
    let layout = create_2d_layout_with_annotations(
        &format!("Grid Search Results - Energy Consumption Heatmap ({}x{})", grid_resolution, grid_resolution),
        860,
        annotations
    );
    plot.set_layout(layout);
    
    save_plot(&plot, &format!("grid_search_{}x{}_energy_heatmap", grid_resolution, grid_resolution));
}

/// Generate 2D heatmap plot for total distance driven with interpolation
pub fn generate_distance_heatmap_plot(
    results: &[(Pos2, f64, f64, f64)], 
    obstacles: &[Obstacle], 
    grid_resolution: usize, 
    optimization_minimum: Option<(Pos2, f64)>
) {
    let distance_results: Vec<(Pos2, f64)> = results.iter()
        .map(|(pos, _, total_distance, _)| (*pos, *total_distance))
        .collect();
    
    let interp_resolution = 1000;
    let (x_grid, y_grid, z_grid) = interpolate_results(&distance_results, obstacles, interp_resolution);
    
    let heatmap = HeatMap::new(x_grid, y_grid, z_grid)
        .color_bar(
            plotly::common::ColorBar::new()
                //.title("Total<br>Distance<br>Travelled<br>(m)")
                .title("d (m)")
                .tick_font(latex_font(20))
        );
    
    let mut plot = Plot::new();
    plot.add_trace(heatmap);
    
    add_obstacles_to_2d_plot(&mut plot, obstacles);
    add_optimization_minimum_to_2d_plot(&mut plot, optimization_minimum);
    
    // Create layout with optional annotation
    let mut annotations = Vec::new();
    if let Some(annotation) = create_minimum_annotation(optimization_minimum, "minimum", 1.0, 18) {
        annotations.push(annotation);
    }
    
    let layout = create_2d_layout_with_annotations(
        &format!("Grid Search Results - Total Distance Heatmap ({}x{})", grid_resolution, grid_resolution),
        820,
        annotations
    );
    plot.set_layout(layout);
    
    save_plot(&plot, &format!("grid_search_{}x{}_distance_heatmap", grid_resolution, grid_resolution));
}

/// Generate 2D heatmap plot for charging distance with interpolation
pub fn generate_charging_distance_heatmap_plot(
    results: &[(Pos2, f64, f64, f64)], 
    obstacles: &[Obstacle], 
    grid_resolution: usize, 
    optimization_minimum: Option<(Pos2, f64)>
) {
    let charging_distance_results: Vec<(Pos2, f64)> = results.iter()
        .map(|(pos, _, _, charging_distance)| (*pos, *charging_distance))
        .collect();
    
    let interp_resolution = 1000;
    let (x_grid, y_grid, z_grid) = interpolate_results(&charging_distance_results, obstacles, interp_resolution);
    
    let heatmap = HeatMap::new(x_grid, y_grid, z_grid)
        .color_bar(
            plotly::common::ColorBar::new()
                //.title("Total<br>Charging<br>Distance<br>(m)")
                .title("d<sup>ch</sup> (m)")
                .tick_font(latex_font(20))
        );
    
    let mut plot = Plot::new();
    plot.add_trace(heatmap);
    
    add_obstacles_to_2d_plot(&mut plot, obstacles);
    add_optimization_minimum_to_2d_plot(&mut plot, optimization_minimum);
    
    // Create layout with optional annotation
    let mut annotations = Vec::new();
    if let Some(annotation) = create_minimum_annotation(optimization_minimum, "minimum", 1.0, 18) {
        annotations.push(annotation);
    }
    
    let layout = create_2d_layout_with_annotations(
        &format!("Grid Search Results - Charging Distance Heatmap ({}x{})", grid_resolution, grid_resolution),
        820,
        annotations
    );
    plot.set_layout(layout);
    
    save_plot(&plot, &format!("grid_search_{}x{}_charging_distance_heatmap", grid_resolution, grid_resolution));
}

/// Generate 2D multi-station visualization with heatmap-style appearance
pub fn generate_multi_station_plot(
    optimal_stations: &[Pos2],
    optimal_energy: f64,
    suboptimal_configs: &[(Vec<Pos2>, f64)],
    obstacles: &[Obstacle],
    field_bounds: (f32, f32, f32, f32),
) {
    let mut plot = Plot::new();
    
    let mut all_configs = vec![(optimal_stations.to_vec(), optimal_energy)];
    for (stations, energy) in suboptimal_configs {
        all_configs.push((stations.clone(), *energy));
    }
    
    let mut energy_rank: Vec<(usize, f64)> = all_configs.iter().enumerate()
        .map(|(i, (_, energy))| (i, *energy))
        .collect();
    energy_rank.sort_by(|a, b| a.1.partial_cmp(&b.1).unwrap());
    
    let colors = [
        "#321decff", "#5754f7ff", "#adb3ffff", "#f0c49cff", "#f3a172ff", "#f3584dff", "#ee2e2eff"
    ];
    
    // Add optimal configuration
    let opt_x: Vec<f64> = optimal_stations.iter().map(|p| p.x as f64).collect();
    let opt_y: Vec<f64> = optimal_stations.iter().map(|p| p.y as f64).collect();
    let optimal_rank = energy_rank.iter().position(|(i, _)| *i == 0).unwrap_or(0);
    let optimal_color_idx = optimal_rank.min(colors.len() - 1);
    
    let optimal_trace = Scatter::new(opt_x.clone(), opt_y.clone())
        .mode(Mode::Markers)
        .name(&format!("Opt. ({:.1} W h)", optimal_energy))
        .marker(Marker::new()
            .size(16)
            .color(colors[optimal_color_idx])
            .line(plotly::common::Line::new().width(3.0).color("black"))
            .symbol(plotly::common::MarkerSymbol::Star));
    
    plot.add_trace(optimal_trace);
    
    // Add suboptimal configurations
    for (i, (stations, energy)) in suboptimal_configs.iter().enumerate() {
        let x_coords: Vec<f64> = stations.iter().map(|p| p.x as f64).collect();
        let y_coords: Vec<f64> = stations.iter().map(|p| p.y as f64).collect();
        
        let config_rank = energy_rank.iter().position(|(idx, _)| *idx == i + 1).unwrap_or(colors.len() - 1);
        let color_idx = config_rank.min(colors.len() - 1);
        
        let marker_symbol = match i {
            0 => plotly::common::MarkerSymbol::Circle,
            1 => plotly::common::MarkerSymbol::Square,
            2 => plotly::common::MarkerSymbol::Diamond,
            3 => plotly::common::MarkerSymbol::Cross,
            4 => plotly::common::MarkerSymbol::X,
            _ => plotly::common::MarkerSymbol::Hexagon,
        };
        
        let trace = Scatter::new(x_coords.clone(), y_coords.clone())
            .mode(Mode::Markers)
            .name(&format!("Cfg. {} ({:.1} W h)", i + 1, energy))
            .marker(Marker::new()
                .size(16)
                .color(colors[color_idx])
                .line(plotly::common::Line::new().width(2.0).color("black"))
                .symbol(marker_symbol));
        
        plot.add_trace(trace);
    }
    
    add_obstacles_to_2d_plot(&mut plot, obstacles);
    add_field_boundaries_to_plot(&mut plot, field_bounds);
    
    let layout = create_multi_station_layout("Charging Station Position Optimization Results", 890);
    plot.set_layout(layout);
    
    let timestamp = chrono::Utc::now().format("%Y%m%d_%H%M%S");
    save_plot(&plot, &format!("multi_station_optimization_energy_{}", timestamp));
}

/// Generate 2D multi-station visualization focused on total traveling distance
pub fn generate_multi_station_distance_plot(
    optimal_stations: &[Pos2],
    optimal_distance: f64,
    suboptimal_configs: &[(Vec<Pos2>, f64)],
    obstacles: &[Obstacle],
    field_bounds: (f32, f32, f32, f32),
) {
    let mut plot = Plot::new();
    
    let mut all_configs = vec![(optimal_stations.to_vec(), optimal_distance)];
    for (stations, distance) in suboptimal_configs {
        all_configs.push((stations.clone(), *distance));
    }
    
    let mut distance_rank: Vec<(usize, f64)> = all_configs.iter().enumerate()
        .map(|(i, (_, distance))| (i, *distance))
        .collect();
    distance_rank.sort_by(|a, b| a.1.partial_cmp(&b.1).unwrap());
    
    let colors = [
        "#321decff", "#5754f7ff", "#adb3ffff", "#f0c49cff", "#f3a172ff", "#f3584dff", "#ee2e2eff"
    ];
    
    // Add optimal configuration
    let opt_x: Vec<f64> = optimal_stations.iter().map(|p| p.x as f64).collect();
    let opt_y: Vec<f64> = optimal_stations.iter().map(|p| p.y as f64).collect();
    let optimal_rank = distance_rank.iter().position(|(i, _)| *i == 0).unwrap_or(0);
    let optimal_color_idx = optimal_rank.min(colors.len() - 1);
    
    let optimal_trace = Scatter::new(opt_x.clone(), opt_y.clone())
        .mode(Mode::Markers)
        .name(&format!("Opt. ({:.1} m)", optimal_distance))
        .marker(Marker::new()
            .size(16)
            .color(colors[optimal_color_idx])
            .line(plotly::common::Line::new().width(3.0).color("black"))
            .symbol(plotly::common::MarkerSymbol::Star));
    
    plot.add_trace(optimal_trace);
    
    // Add suboptimal configurations
    for (i, (stations, distance)) in suboptimal_configs.iter().enumerate() {
        let x_coords: Vec<f64> = stations.iter().map(|p| p.x as f64).collect();
        let y_coords: Vec<f64> = stations.iter().map(|p| p.y as f64).collect();
        
        let config_rank = distance_rank.iter().position(|(idx, _)| *idx == i + 1).unwrap_or(colors.len() - 1);
        let color_idx = config_rank.min(colors.len() - 1);
        
        let marker_symbol = match i {
            0 => plotly::common::MarkerSymbol::Circle,
            1 => plotly::common::MarkerSymbol::Square,
            2 => plotly::common::MarkerSymbol::Diamond,
            3 => plotly::common::MarkerSymbol::Cross,
            4 => plotly::common::MarkerSymbol::X,
            _ => plotly::common::MarkerSymbol::Hexagon,
        };
        
        let trace = Scatter::new(x_coords.clone(), y_coords.clone())
            .mode(Mode::Markers)
            .name(&format!("Cfg. {} ({:.1} m)", i + 1, distance))
            .marker(Marker::new()
                .size(16)
                .color(colors[color_idx])
                .line(plotly::common::Line::new().width(2.0).color("black"))
                .symbol(marker_symbol));
        
        plot.add_trace(trace);
    }
    
    add_obstacles_to_2d_plot(&mut plot, obstacles);
    add_field_boundaries_to_plot(&mut plot, field_bounds);
    
    let layout = create_multi_station_layout("Charging Station Position Optimization Results - Total Traveling Distance", 890);
    plot.set_layout(layout);
    
    let timestamp = chrono::Utc::now().format("%Y%m%d_%H%M%S");
    save_plot(&plot, &format!("multi_station_distance_{}", timestamp));
}

// Helper functions

/// Interpolate results to create smooth heatmap
fn interpolate_results(
    results: &[(Pos2, f64)], 
    obstacles: &[Obstacle], 
    resolution: usize
) -> (Vec<f64>, Vec<f64>, Vec<Vec<f64>>) {
    let min_x = results.iter().map(|(pos, _)| pos.x).fold(f32::INFINITY, f32::min);
    let max_x = results.iter().map(|(pos, _)| pos.x).fold(f32::NEG_INFINITY, f32::max);
    let min_y = results.iter().map(|(pos, _)| pos.y).fold(f32::INFINITY, f32::min);
    let max_y = results.iter().map(|(pos, _)| pos.y).fold(f32::NEG_INFINITY, f32::max);
    
    let step_x = (max_x - min_x) / (resolution - 1) as f32;
    let step_y = (max_y - min_y) / (resolution - 1) as f32;
    
    let mut x_grid = Vec::new();
    let mut y_grid = Vec::new();
    let mut z_grid = vec![vec![f64::NAN; resolution]; resolution];
    
    for i in 0..resolution {
        x_grid.push((min_x + i as f32 * step_x) as f64);
    }
    for j in 0..resolution {
        y_grid.push((min_y + j as f32 * step_y) as f64);
    }
    
    for (i, &x) in x_grid.iter().enumerate() {
        for (j, &y) in y_grid.iter().enumerate() {
            let point = Pos2::new(x as f32, y as f32);
            
            if !is_position_valid(point, obstacles, 0.4) {
                z_grid[j][i] = f64::NAN;
                continue;
            }
            
            let mut weighted_sum = 0.0;
            let mut weight_sum = 0.0;
            
            for (result_pos, energy) in results {
                let distance = ((point.x - result_pos.x).powi(2) + (point.y - result_pos.y).powi(2)).sqrt();
                
                if distance < 0.001 {
                    z_grid[j][i] = *energy;
                    break;
                } else {
                    let weight = 1.0 / (distance as f64).powi(4);
                    weighted_sum += weight * energy;
                    weight_sum += weight;
                }
            }
            
            if weight_sum > 0.0 && z_grid[j][i].is_nan() {
                z_grid[j][i] = weighted_sum / weight_sum;
            }
        }
    }
    
    (x_grid, y_grid, z_grid)
}

fn add_obstacles_to_3d_plot(plot: &mut Plot, obstacles: &[Obstacle], z_level: f64) {
    for (i, obstacle) in obstacles.iter().enumerate() {
        let mut obstacle_x = Vec::new();
        let mut obstacle_y = Vec::new();
        let mut obstacle_z = Vec::new();

        for point in &obstacle.points {
            obstacle_x.push(point.x as f64);
            obstacle_y.push(point.y as f64);
            obstacle_z.push(z_level);
        }

        if let Some(first_point) = obstacle.points.first() {
            obstacle_x.push(first_point.x as f64);
            obstacle_y.push(first_point.y as f64);
            obstacle_z.push(z_level);
        }

        let obstacle_trace = Scatter3D::new(obstacle_x, obstacle_y, obstacle_z)
            .mode(Mode::Lines)
            .name(&format!("Obstacle {}", i + 1))
            .show_legend(false)
            .line(plotly::common::Line::new().color("black").width(2.0));
        
        plot.add_trace(obstacle_trace);
    }
}

fn add_obstacles_to_2d_plot(plot: &mut Plot, obstacles: &[Obstacle]) {
    for (i, obstacle) in obstacles.iter().enumerate() {
        let mut obstacle_x = Vec::new();
        let mut obstacle_y = Vec::new();

        for point in &obstacle.points {
            obstacle_x.push(point.x as f64);
            obstacle_y.push(point.y as f64);
        }

        if let Some(first_point) = obstacle.points.first() {
            obstacle_x.push(first_point.x as f64);
            obstacle_y.push(first_point.y as f64);
        }

        let obstacle_trace = Scatter::new(obstacle_x, obstacle_y)
            .mode(Mode::Lines)
            .name(&format!("Obstacle {}", i + 1))
            .show_legend(false)
            .line(plotly::common::Line::new().color("black").width(2.0));
        
        plot.add_trace(obstacle_trace);
    }
}

/// Add optimization minimum marker to 2D plot
/// 
/// # Arguments
/// * `plot` - The plot to add the marker to
/// * `optimization_minimum` - Optional tuple of (position, value)
fn add_optimization_minimum_to_2d_plot(plot: &mut Plot, optimization_minimum: Option<(Pos2, f64)>) {
    if let Some((opt_pos, _opt_value)) = optimization_minimum {
        let x = opt_pos.x as f64;
        let y = opt_pos.y as f64;
        let cross_size = 0.4; // Size of the cross in plot units
        
        // Draw white padding cross (thicker lines, underneath)
        // Diagonal line 1 (top-left to bottom-right)
        let padding_line1 = Scatter::new(
            vec![x - cross_size, x + cross_size],
            vec![y + cross_size, y - cross_size]
        )
        .mode(Mode::Lines)
        .show_legend(false)
        .line(plotly::common::Line::new().width(10.0).color("white"));
        
        // Diagonal line 2 (top-right to bottom-left)
        let padding_line2 = Scatter::new(
            vec![x - cross_size, x + cross_size],
            vec![y - cross_size, y + cross_size]
        )
        .mode(Mode::Lines)
        .show_legend(false)
        .line(plotly::common::Line::new().width(10.0).color("white"));
        
        plot.add_trace(padding_line1);
        plot.add_trace(padding_line2);
        
        // Draw black cross on top (thinner lines)
        // Diagonal line 1 (top-left to bottom-right)
        let black_line1 = Scatter::new(
            vec![x - cross_size, x + cross_size],
            vec![y + cross_size, y - cross_size]
        )
        .mode(Mode::Lines)
        .name("Optimization Minimum")
        .show_legend(false)
        .line(plotly::common::Line::new().width(4.0).color("black"));
        
        // Diagonal line 2 (top-right to bottom-left)
        let black_line2 = Scatter::new(
            vec![x - cross_size, x + cross_size],
            vec![y - cross_size, y + cross_size]
        )
        .mode(Mode::Lines)
        .show_legend(false)
        .line(plotly::common::Line::new().width(4.0).color("black"));
        
        plot.add_trace(black_line1);
        plot.add_trace(black_line2);
    }
}

/// Add optimization minimum marker to 2D plot with custom styling
/// 
/// # Arguments
/// * `plot` - The plot to add the marker to
/// * `optimization_minimum` - Optional tuple of (position, value)
/// * `marker_size` - Size of the marker
/// * `marker_color` - Color of the marker as a string
/// * `marker_symbol` - Symbol for the marker (e.g., "circle", "x", "cross", "diamond")
fn add_optimization_minimum_to_2d_plot_custom(
    plot: &mut Plot, 
    optimization_minimum: Option<(Pos2, f64)>,
    marker_size: usize,
    marker_color: &str,
    marker_symbol: &str
) {
    if let Some((opt_pos, _opt_value)) = optimization_minimum {
        let opt_trace = Scatter::new(vec![opt_pos.x as f64], vec![opt_pos.y as f64])
            .mode(Mode::Markers)
            .name("Optimization Minimum")
            .show_legend(false)
            .marker(
                Marker::new()
                    .size(marker_size)
                    .color(marker_color.to_string())
                    .symbol(plotly::common::MarkerSymbol::X)
            );
        
        plot.add_trace(opt_trace);
    }
}

/// Add optimization minimum annotation to a layout
/// 
/// Creates a text annotation above the minimum point with customizable styling.
/// 
/// # Arguments
/// * `optimization_minimum` - Optional tuple of (position, value) for the minimum
/// * `label` - Text to display in the annotation (e.g., "Minimum")
/// * `y_offset` - Vertical offset from the marker position (default: 1.0)
/// * `font_size` - Font size for the annotation text (default: 18)
/// 
/// # Returns
/// * `Option<plotly::layout::Annotation>` - The annotation if a minimum exists, None otherwise
fn create_minimum_annotation(
    optimization_minimum: Option<(Pos2, f64)>,
    label: &str,
    y_offset: f64,
    font_size: usize
) -> Option<plotly::layout::Annotation> {
    optimization_minimum.map(|(opt_pos, _opt_value)| {
        plotly::layout::Annotation::new()
            .x(opt_pos.x as f64)
            .y(opt_pos.y as f64 + y_offset)
            .text(label)
            .show_arrow(false)
            .font(latex_font(font_size))
            .background_color("white")
            .border_color("black")
            .border_width(1.0)
    })
}

fn add_field_boundaries_to_plot(plot: &mut Plot, field_bounds: (f32, f32, f32, f32)) {
    let boundary_x = vec![
        field_bounds.0 as f64, field_bounds.1 as f64, field_bounds.1 as f64, 
        field_bounds.0 as f64, field_bounds.0 as f64
    ];
    let boundary_y = vec![
        field_bounds.2 as f64, field_bounds.2 as f64, field_bounds.3 as f64, 
        field_bounds.3 as f64, field_bounds.2 as f64
    ];
    
    let boundary_trace = Scatter::new(boundary_x, boundary_y)
        .mode(Mode::Lines)
        .name("Field Boundary")
        .show_legend(false)
        .line(plotly::common::Line::new().color("black").width(2.0));
    
    plot.add_trace(boundary_trace);
}

fn create_2d_layout(title: &str, width: usize) -> Layout {
    create_2d_layout_with_annotations(title, width, vec![])
}

fn create_2d_layout_with_annotations(
    title: &str, 
    width: usize, 
    annotations: Vec<plotly::layout::Annotation>
) -> Layout {
    let mut layout = Layout::new()
        .title(title)
        .width(width)
        .height(800)
        .font(latex_font(20))
        .x_axis(plotly::layout::Axis::new()
            .title("x (m)")
            .tick_font(latex_font(20)))
        .y_axis(plotly::layout::Axis::new()
            .title("y (m)")
            .tick_font(latex_font(20)));
    
    if !annotations.is_empty() {
        layout = layout.annotations(annotations);
    }
    
    layout
}

fn create_multi_station_layout(title: &str, width: usize) -> Layout {
    Layout::new()
        .title(title)
        .width(width)
        .height(800)
        .font(latex_font(20))
        .x_axis(plotly::layout::Axis::new()
            .title("x (m)")
            .tick_font(latex_font(20))
            .show_grid(false)
            .show_line(true)
            .line_color("black")
            .line_width(1)
            .zero_line(false)
            .ticks(plotly::layout::TicksDirection::Outside)
            .tick_length(5)
            .tick_width(1)
            .tick_color("black"))
        .y_axis(plotly::layout::Axis::new()
            .title("y (m)")
            .tick_font(latex_font(20))
            .show_grid(false)
            .show_line(true)
            .line_color("black")
            .line_width(1)
            .zero_line(false)
            .ticks(plotly::layout::TicksDirection::Outside)
            .tick_length(5)
            .tick_width(1)
            .tick_color("black"))
        .legend(plotly::layout::Legend::new()
            .x(1.02)
            .y(1.0)
            .font(latex_font(20)))
        .plot_background_color("white")
        .paper_background_color("white")
        .show_legend(true)
}

fn save_plot(plot: &Plot, filename_base: &str) {
    let filename_html = format!("results/{}.html", filename_base);
    let filename_svg = format!("results/{}.svg", filename_base);
    
    plot.write_html(&filename_html);
    if let Err(e) = plot.write_image(&filename_svg, ImageFormat::SVG, 860, 800, 1.0) {
        eprintln!("Failed to write SVG file: {}", e);
        println!("Plot saved to: {}", filename_html);
    } else {
        println!("Plot saved to: {} and {}", filename_html, filename_svg);
    }
}

// Geometric helper functions

fn is_position_valid(position: Pos2, obstacles: &[Obstacle], margin: f32) -> bool {
    for obstacle in obstacles {
        if is_point_inside_polygon(position, &obstacle.points) {
            return false;
        }
        
        for window in obstacle.points.windows(2) {
            if let [p1, p2] = window {
                let distance = point_to_line_distance(position, *p1, *p2);
                if distance < margin {
                    return false;
                }
            }
        }
        
        if obstacle.points.len() >= 2 {
            let first = obstacle.points[0];
            let last = obstacle.points[obstacle.points.len() - 1];
            let distance = point_to_line_distance(position, last, first);
            if distance < margin {
                return false;
            }
        }
    }
    true
}

fn is_point_inside_polygon(point: Pos2, polygon: &[Pos2]) -> bool {
    if polygon.len() < 3 {
        return false;
    }
    
    let mut inside = false;
    let mut j = polygon.len() - 1;
    
    for i in 0..polygon.len() {
        let pi = polygon[i];
        let pj = polygon[j];
        
        if ((pi.y > point.y) != (pj.y > point.y)) &&
           (point.x < (pj.x - pi.x) * (point.y - pi.y) / (pj.y - pi.y) + pi.x) {
            inside = !inside;
        }
        j = i;
    }
    
    inside
}

fn point_to_line_distance(point: Pos2, line_start: Pos2, line_end: Pos2) -> f32 {
    let line_vec = Pos2::new(line_end.x - line_start.x, line_end.y - line_start.y);
    let point_vec = Pos2::new(point.x - line_start.x, point.y - line_start.y);
    
    let line_len_sq = line_vec.x * line_vec.x + line_vec.y * line_vec.y;
    if line_len_sq == 0.0 {
        return (point_vec.x * point_vec.x + point_vec.y * point_vec.y).sqrt();
    }
    
    let t = (point_vec.x * line_vec.x + point_vec.y * line_vec.y) / line_len_sq;
    let t = t.clamp(0.0, 1.0);
    
    let projection = Pos2::new(
        line_start.x + t * line_vec.x,
        line_start.y + t * line_vec.y
    );
    
    let dist_vec = Pos2::new(point.x - projection.x, point.y - projection.y);
    (dist_vec.x * dist_vec.x + dist_vec.y * dist_vec.y).sqrt()
}