use crate::cfg::{
    DEFAULT_SCENE_CONFIG_PATH
};
use crate::environment::{
    scene_config::SceneConfig,
    field_config::FieldConfig,
    obstacle::Obstacle,
};

use egui::Pos2;
use plotly::{Plot, Scatter, Scatter3D, HeatMap, Layout, common::{Marker, Mode}, ImageFormat};
use crate::experiment::geometry::interpolate_results;

/// Generate visualization plots for grid search results
pub fn generate_grid_search_plots(results: &[(Pos2, f64, f64, f64, f64)], obstacles: &[Obstacle], grid_resolution: usize, optimization_minimum: Option<(Pos2, f64)>) {
    generate_3d_plot(results, obstacles, grid_resolution, optimization_minimum);
    generate_energy_heatmap_plot(results, obstacles, grid_resolution, optimization_minimum);
    generate_distance_heatmap_plot(results, obstacles, grid_resolution, optimization_minimum);
    generate_charging_distance_heatmap_plot(results, obstacles, grid_resolution, optimization_minimum);
}

/// Generate 3D scatter plot (similar to optimization.rs)
fn generate_3d_plot(results: &[(Pos2, f64, f64, f64, f64)], obstacles: &[Obstacle], grid_resolution: usize, optimization_minimum: Option<(Pos2, f64)>) {
    let x_coords: Vec<f64> = results.iter().map(|(pos, _, _, _, _)| pos.x as f64).collect();
    let y_coords: Vec<f64> = results.iter().map(|(pos, _, _, _, _)| pos.y as f64).collect();
    let energy_values: Vec<f64> = results.iter().map(|(_, energy, _, _, _)| *energy).collect();
    
    // Find the optimal (minimum) energy value for obstacle z-level
    let min_energy = energy_values.iter().fold(f64::INFINITY, |a, &b| a.min(b));
    
    let trace = Scatter3D::new(x_coords, y_coords, energy_values)
        .mode(Mode::Markers)
        .marker(Marker::new().size(4).color("blue"));
    
    let mut plot = Plot::new();
    plot.add_trace(trace);
    
    // Add optimization minimum point if provided (using the actual energy value from optimization)
    if let Some((opt_pos, opt_value)) = optimization_minimum {
        let opt_trace = Scatter3D::new(
            vec![opt_pos.x as f64], 
            vec![opt_pos.y as f64], 
            vec![opt_value]
        )
        .mode(Mode::Markers)
        .name("Optimization Minimum")
        .marker(Marker::new().size(12).color("red"));
        
        plot.add_trace(opt_trace);
    }
    
    // Add obstacle boundaries as 3D traces
    for (i, obstacle) in obstacles.iter().enumerate() {
        let mut obstacle_x = Vec::new();
        let mut obstacle_y = Vec::new();
        let mut obstacle_z = Vec::new();

        // Add each edge of the obstacle
        for point in &obstacle.points {
            obstacle_x.push(point.x as f64);
            obstacle_y.push(point.y as f64);
            obstacle_z.push(min_energy); // Set z to the minimum energy value
        }

        // Close the polygon by connecting the last point to the first
        if let Some(first_point) = obstacle.points.first() {
            obstacle_x.push(first_point.x as f64);
            obstacle_y.push(first_point.y as f64);
            obstacle_z.push(min_energy);
        }

        // Create a trace for the obstacle
        let obstacle_trace = Scatter3D::new(obstacle_x, obstacle_y, obstacle_z)
            .mode(Mode::Lines)
            .name(&format!("Obstacle {}", i + 1))
            .show_legend(false)
            .line(plotly::common::Line::new().color("black").width(2.0));
        
        plot.add_trace(obstacle_trace);
    }
    
    let layout = Layout::new()
        .title(&format!("Grid Search Results - 3D View ({}x{})", grid_resolution, grid_resolution))
        .width(1200)
        .height(800)
        .x_axis(plotly::layout::Axis::new()
            .title("X Position (m)")
            .tick_font(plotly::common::Font::new().size(14)))
        .y_axis(plotly::layout::Axis::new()
            .title("Y Position (m)")
            .tick_font(plotly::common::Font::new().size(14)))
        .legend(plotly::layout::Legend::new()
            .font(plotly::common::Font::new().size(14)));
    
    plot.set_layout(layout);
    
    let filename_html = format!("results/grid_search_{}x{}_3d.html", grid_resolution, grid_resolution);
    let filename_svg = format!("results/grid_search_{}x{}_3d.svg", grid_resolution, grid_resolution);
    
    plot.write_html(&filename_html);
    if let Err(e) = plot.write_image(&filename_svg, ImageFormat::SVG, 800, 600, 1.0) {
        eprintln!("Failed to write SVG file: {}", e);
        println!("3D plot saved to: {}", filename_html);
    } else {
        println!("3D plot saved to: {} and {}", filename_html, filename_svg);
    }
}

/// Generate 2D heatmap plot for energy consumption with interpolation
fn generate_energy_heatmap_plot(results: &[(Pos2, f64, f64, f64, f64)], obstacles: &[Obstacle], grid_resolution: usize, optimization_minimum: Option<(Pos2, f64)>) {
    // Extract energy data for heatmap
    let energy_results: Vec<(Pos2, f64)> = results.iter()
        .map(|(pos, energy, _, _, _)| (*pos, *energy))
        .collect();
    
    // Create interpolated grid for smooth heatmap
    let interp_resolution = 1000; // Higher resolution for smooth interpolation
    let (x_grid, y_grid, z_grid) = interpolate_results(&energy_results, obstacles, interp_resolution);
    
    let heatmap = HeatMap::new(x_grid, y_grid, z_grid)
        .color_bar(
            plotly::common::ColorBar::new()
                .title("Energy<br>Consumption<br>(Wh)")
                .tick_font(plotly::common::Font::new().size(14))
        );
    
    let mut plot = Plot::new();
    plot.add_trace(heatmap);
    
    // Add obstacle boundaries as traces
    for (i, obstacle) in obstacles.iter().enumerate() {
        let mut obstacle_x = Vec::new();
        let mut obstacle_y = Vec::new();

        // Add each edge of the obstacle
        for point in &obstacle.points {
            obstacle_x.push(point.x as f64);
            obstacle_y.push(point.y as f64);
        }

        // Close the polygon by connecting the last point to the first
        if let Some(first_point) = obstacle.points.first() {
            obstacle_x.push(first_point.x as f64);
            obstacle_y.push(first_point.y as f64);
        }

        // Create a trace for the obstacle
        let obstacle_trace = Scatter::new(obstacle_x, obstacle_y)
            .mode(Mode::Lines)
            .name(&format!("Obstacle {}", i + 1))
            .show_legend(false)
            .line(plotly::common::Line::new().color("black").width(2.0));
        
        plot.add_trace(obstacle_trace);
    }
    
    // Add optimization minimum point if provided (using black marker for better visibility)
    if let Some((opt_pos, _opt_value)) = optimization_minimum {
        let opt_trace = Scatter::new(vec![opt_pos.x as f64], vec![opt_pos.y as f64])
            .mode(Mode::Markers)
            .name("Optimization Minimum")
            .show_legend(false)  // Don't show in legend
            .marker(Marker::new().size(15).color("black"));
        
        plot.add_trace(opt_trace);
        
        // Add text annotation above the marker
        let annotation = plotly::layout::Annotation::new()
            .x(opt_pos.x as f64)
            .y(opt_pos.y as f64 + 1.0) 
            .text(&format!("Minimum"))
            .show_arrow(false)
            .font(plotly::common::Font::new().size(18).color("black"))
            .background_color("white")
            .border_color("black")
            .border_width(1.0);

        let layout = Layout::new()
            .title(&format!("Grid Search Results - Energy Consumption Heatmap ({}x{})", grid_resolution, grid_resolution))
            .width(860)
            .height(800)
            .font(plotly::common::Font::new().size(16).color("black"))
            .x_axis(plotly::layout::Axis::new()
                .title("X Position (m)")
                .tick_font(plotly::common::Font::new().size(16)))
            .y_axis(plotly::layout::Axis::new()
                .title("Y Position (m)")
                .tick_font(plotly::common::Font::new().size(16)))
            .annotations(vec![annotation]);
        
        plot.set_layout(layout);

    } else {
        let layout = Layout::new()
                .title(&format!("Grid Search Results - Energy Consumption Heatmap ({}x{})", grid_resolution, grid_resolution))
                .width(860)
                .height(800)
                .font(plotly::common::Font::new().size(16).color("black"))
                .x_axis(plotly::layout::Axis::new()
                    .title("X Position (m)")
                    .tick_font(plotly::common::Font::new().size(16)))
                .y_axis(plotly::layout::Axis::new()
                    .title("Y Position (m)")
                    .tick_font(plotly::common::Font::new().size(16)));
            
            plot.set_layout(layout);
    }    
    
    let filename_html = format!("results/grid_search_{}x{}_energy_heatmap.html", grid_resolution, grid_resolution);
    let filename_svg = format!("results/grid_search_{}x{}_energy_heatmap.svg", grid_resolution, grid_resolution);
    
    plot.write_html(&filename_html);
    if let Err(e) = plot.write_image(&filename_svg, ImageFormat::SVG, 860, 800, 1.0) {
        eprintln!("Failed to write SVG file: {}", e);
        println!("Energy heatmap saved to: {}", filename_html);
    } else {
        println!("Energy heatmap saved to: {} and {}", filename_html, filename_svg);
    }
}

/// Generate 2D heatmap plot for total distance driven with interpolation
fn generate_distance_heatmap_plot(results: &[(Pos2, f64, f64, f64, f64)], obstacles: &[Obstacle], grid_resolution: usize, optimization_minimum: Option<(Pos2, f64)>) {
    // Extract distance data for heatmap
    let distance_results: Vec<(Pos2, f64)> = results.iter()
        .map(|(pos, _, total_distance, _, _)| (*pos, *total_distance))
        .collect();
    
    // Create interpolated grid for smooth heatmap
    let interp_resolution = 1000; // Higher resolution for smooth interpolation
    let (x_grid, y_grid, z_grid) = interpolate_results(&distance_results, obstacles, interp_resolution);
    
    let heatmap = HeatMap::new(x_grid, y_grid, z_grid)
        .color_bar(
            plotly::common::ColorBar::new()
                .title("Total<br>Distance<br>Driven<br>(m)")
                .tick_font(plotly::common::Font::new().size(14))
        );
    
    let mut plot = Plot::new();
    plot.add_trace(heatmap);
    
    // Add obstacle boundaries as traces
    for (i, obstacle) in obstacles.iter().enumerate() {
        let mut obstacle_x = Vec::new();
        let mut obstacle_y = Vec::new();

        // Add each edge of the obstacle
        for point in &obstacle.points {
            obstacle_x.push(point.x as f64);
            obstacle_y.push(point.y as f64);
        }

        // Close the polygon by connecting the last point to the first
        if let Some(first_point) = obstacle.points.first() {
            obstacle_x.push(first_point.x as f64);
            obstacle_y.push(first_point.y as f64);
        }

        // Create a trace for the obstacle
        let obstacle_trace = Scatter::new(obstacle_x, obstacle_y)
            .mode(Mode::Lines)
            .name(&format!("Obstacle {}", i + 1))
            .show_legend(false)
            .line(plotly::common::Line::new().color("black").width(2.0));
        
        plot.add_trace(obstacle_trace);
    }
    
    // Add optimization minimum point if provided (using black marker for better visibility)
    if let Some((opt_pos, _opt_value)) = optimization_minimum {
        let opt_trace = Scatter::new(vec![opt_pos.x as f64], vec![opt_pos.y as f64])
            .mode(Mode::Markers)
            .name("Energy Consumption Optimization Minimum")
            .show_legend(false)  // Don't show in legend
            .marker(Marker::new().size(15).color("black"));
        
        plot.add_trace(opt_trace);
        
        // Add text annotation above the marker
        let annotation = plotly::layout::Annotation::new()
            .x(opt_pos.x as f64)
            .y(opt_pos.y as f64 + 1.0)  // Position text slightly above the marker
            .text(&format!("Minimum"))
            .show_arrow(false)
            .font(plotly::common::Font::new().size(18).color("black"))
            .background_color("white")
            .border_color("black")
            .border_width(1.0);

        let layout = Layout::new()
            .title(&format!("Grid Search Results - Total Distance Heatmap ({}x{})", grid_resolution, grid_resolution))
            .width(820)
            .height(800)
            .font(plotly::common::Font::new().size(16).color("black"))
            .x_axis(plotly::layout::Axis::new()
                .title("X Position (m)")
                .tick_font(plotly::common::Font::new().size(16)))
            .y_axis(plotly::layout::Axis::new()
                .title("Y Position (m)")
                .tick_font(plotly::common::Font::new().size(16)))
            .annotations(vec![annotation]);
        
        plot.set_layout(layout);

    } else {
        let layout = Layout::new()
                .title(&format!("Grid Search Results - Total Distance Heatmap ({}x{})", grid_resolution, grid_resolution))
                .width(820)
                .height(800)
                .font(plotly::common::Font::new().size(16).color("black"))
                .x_axis(plotly::layout::Axis::new()
                    .title("X Position (m)")
                    .tick_font(plotly::common::Font::new().size(16)))
                .y_axis(plotly::layout::Axis::new()
                    .title("Y Position (m)")
                    .tick_font(plotly::common::Font::new().size(16)));
            
            plot.set_layout(layout);
    }    
    
    let filename_html = format!("results/grid_search_{}x{}_distance_heatmap.html", grid_resolution, grid_resolution);
    let filename_svg = format!("results/grid_search_{}x{}_distance_heatmap.svg", grid_resolution, grid_resolution);
    
    plot.write_html(&filename_html);
    if let Err(e) = plot.write_image(&filename_svg, ImageFormat::SVG, 860, 800, 1.0) {
        eprintln!("Failed to write SVG file: {}", e);
        println!("Total distance heatmap saved to: {}", filename_html);
    } else {
        println!("Total distance heatmap saved to: {} and {}", filename_html, filename_svg);
    }
}

/// Generate 2D heatmap plot for charging distance with interpolation
fn generate_charging_distance_heatmap_plot(results: &[(Pos2, f64, f64, f64, f64)], obstacles: &[Obstacle], grid_resolution: usize, optimization_minimum: Option<(Pos2, f64)>) {
    // Extract charging distance data for heatmap
    let charging_distance_results: Vec<(Pos2, f64)> = results.iter()
        .map(|(pos, _, _, charging_distance, _)| (*pos, *charging_distance))
        .collect();
    
    // Create interpolated grid for smooth heatmap
    let interp_resolution = 1000; // Higher resolution for smooth interpolation
    let (x_grid, y_grid, z_grid) = interpolate_results(&charging_distance_results, obstacles, interp_resolution);
    
    let heatmap = HeatMap::new(x_grid, y_grid, z_grid)
        .color_bar(
            plotly::common::ColorBar::new()
                .title("Total<br>Charging<br>Distance<br>(m)")
                .tick_font(plotly::common::Font::new().size(14))
        );
    
    let mut plot = Plot::new();
    plot.add_trace(heatmap);
    
    // Add obstacle boundaries as traces
    for (i, obstacle) in obstacles.iter().enumerate() {
        let mut obstacle_x = Vec::new();
        let mut obstacle_y = Vec::new();

        // Add each edge of the obstacle
        for point in &obstacle.points {
            obstacle_x.push(point.x as f64);
            obstacle_y.push(point.y as f64);
        }

        // Close the polygon by connecting the last point to the first
        if let Some(first_point) = obstacle.points.first() {
            obstacle_x.push(first_point.x as f64);
            obstacle_y.push(first_point.y as f64);
        }

        // Create a trace for the obstacle
        let obstacle_trace = Scatter::new(obstacle_x, obstacle_y)
            .mode(Mode::Lines)
            .name(&format!("Obstacle {}", i + 1))
            .show_legend(false)
            .line(plotly::common::Line::new().color("black").width(2.0));
        
        plot.add_trace(obstacle_trace);
    }
    
    // Add optimization minimum point if provided (using black marker for better visibility)
    if let Some((opt_pos, _opt_value)) = optimization_minimum {
        let opt_trace = Scatter::new(vec![opt_pos.x as f64], vec![opt_pos.y as f64])
            .mode(Mode::Markers)
            .name("Energy Consumption Optimization Minimum")
            .show_legend(false)  // Don't show in legend
            .marker(Marker::new().size(15).color("black"));
        
        plot.add_trace(opt_trace);
        
        // Add text annotation above the marker
        let annotation = plotly::layout::Annotation::new()
            .x(opt_pos.x as f64)
            .y(opt_pos.y as f64 + 1.0)  // Position text slightly above the marker
            .text(&format!("Minimum"))
            .show_arrow(false)
            .font(plotly::common::Font::new().size(18).color("black"))
            .background_color("white")
            .border_color("black")
            .border_width(1.0);

        let layout = Layout::new()
            .title(&format!("Grid Search Results - Charging Distance Heatmap ({}x{})", grid_resolution, grid_resolution))
            .width(820)
            .height(800)
            .font(plotly::common::Font::new().size(16).color("black"))
            .x_axis(plotly::layout::Axis::new()
                .title("X Position (m)")
                .tick_font(plotly::common::Font::new().size(16)))
            .y_axis(plotly::layout::Axis::new()
                .title("Y Position (m)")
                .tick_font(plotly::common::Font::new().size(16)))
            .annotations(vec![annotation]);
        
        plot.set_layout(layout);

    } else {
        let layout = Layout::new()
                .title(&format!("Grid Search Results - Charging Distance Heatmap ({}x{})", grid_resolution, grid_resolution))
                .width(820)
                .height(800)
                .font(plotly::common::Font::new().size(16).color("black"))
                .x_axis(plotly::layout::Axis::new()
                    .title("X Position (m)")
                    .tick_font(plotly::common::Font::new().size(16)))
                .y_axis(plotly::layout::Axis::new()
                    .title("Y Position (m)")
                    .tick_font(plotly::common::Font::new().size(16)));
            
            plot.set_layout(layout);
    }    
    
    let filename_html = format!("results/grid_search_{}x{}_charging_distance_heatmap.html", grid_resolution, grid_resolution);
    let filename_svg = format!("results/grid_search_{}x{}_charging_distance_heatmap.svg", grid_resolution, grid_resolution);
    
    plot.write_html(&filename_html);
    if let Err(e) = plot.write_image(&filename_svg, ImageFormat::SVG, 860, 800, 1.0) {
        eprintln!("Failed to write SVG file: {}", e);
        println!("Charging distance heatmap saved to: {}", filename_html);
    } else {
        println!("Charging distance heatmap saved to: {} and {}", filename_html, filename_svg);
    }
}


/// Generate 2D multi-station visualization with heatmap-style appearance
pub fn generate_multi_station_plot(
    optimal_stations: &[Pos2],
    optimal_energy: f64,
    suboptimal_configs: &[(Vec<Pos2>, f64)], // Up to 4-5 suboptimal configurations
    obstacles: &[Obstacle],
    field_bounds: (f32, f32, f32, f32), // (min_x, max_x, min_y, max_y)
) {
    let mut plot = Plot::new();
    
    // Collect all energy values for ranking
    let mut all_configs = vec![(optimal_stations.to_vec(), optimal_energy)];
    for (stations, energy) in suboptimal_configs {
        all_configs.push((stations.clone(), *energy));
    }
    
    // Sort by energy (ascending) to determine color ranking
    let mut energy_rank: Vec<(usize, f64)> = all_configs.iter().enumerate()
        .map(|(i, (_, energy))| (i, *energy))
        .collect();
    energy_rank.sort_by(|a, b| a.1.partial_cmp(&b.1).unwrap());
    
    // Heatmap-style color scheme (blue to red)
    let colors = [
        "#321decff",  "#5754f7ff", "#adb3ffff", "#f0c49cff", "#f3a172ff", "#f3584dff", "#ee2e2eff"
    ];
    
    // Add optimal configuration (first in sorted list)
    let opt_x: Vec<f64> = optimal_stations.iter().map(|p| p.x as f64).collect();
    let opt_y: Vec<f64> = optimal_stations.iter().map(|p| p.y as f64).collect();
    let optimal_rank = energy_rank.iter().position(|(i, _)| *i == 0).unwrap_or(0);
    let optimal_color_idx = optimal_rank.min(colors.len() - 1);
    
    let optimal_trace = Scatter::new(opt_x.clone(), opt_y.clone())
        .mode(Mode::Markers)
        .name(&format!("Optimal ({:.1} Wh)", optimal_energy))
        .marker(Marker::new()
            .size(18)
            .color(colors[optimal_color_idx])
            .line(plotly::common::Line::new().width(3.0).color("black"))
            .symbol(plotly::common::MarkerSymbol::Star)); // Always use star for optimal
    
    plot.add_trace(optimal_trace);
    
    
    // Add suboptimal configurations
    for (i, (stations, energy)) in suboptimal_configs.iter().enumerate() {
        let x_coords: Vec<f64> = stations.iter().map(|p| p.x as f64).collect();
        let y_coords: Vec<f64> = stations.iter().map(|p| p.y as f64).collect();
        
        let config_rank = energy_rank.iter().position(|(idx, _)| *idx == i + 1).unwrap_or(colors.len() - 1);
        let color_idx = config_rank.min(colors.len() - 1);
        
        // Use different marker shape for each config (cycle through available shapes)
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
            .name(&format!("Config {} ({:.1} Wh)", i + 1, energy))
            .marker(Marker::new()
                .size(14)
                .color(colors[color_idx])
                .line(plotly::common::Line::new().width(2.0).color("black"))
                .symbol(marker_symbol));
        
        plot.add_trace(trace);
    }
    
    // Add obstacles
    for (i, obstacle) in obstacles.iter().enumerate() {
        let mut obstacle_x = Vec::new();
        let mut obstacle_y = Vec::new();

        for point in &obstacle.points {
            obstacle_x.push(point.x as f64);
            obstacle_y.push(point.y as f64);
        }

        // Close the polygon
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
    
    // Add field boundaries as visible lines
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
    
    // Create layout matching heatmap style
    let layout = Layout::new()
        .title("Charging Station Position Optimization Results")
        .width(890)  // Match heatmap size
        .height(800)
        .font(plotly::common::Font::new().size(16).color("black"))
        .x_axis(plotly::layout::Axis::new()
            .title("X Position (m)")
            .range(vec![field_bounds.0 as f64, field_bounds.1 as f64])
            .auto_range(false)  // Disable auto range to use exact range
            .tick_font(plotly::common::Font::new().size(16))
            .show_grid(false)  // Remove grid
            .show_line(true)   // Show axis line
            .line_color("black")
            .line_width(1)
            .zero_line(false)  // Disable zero line
            .ticks(plotly::layout::TicksDirection::Outside)  // Show tick marks outside
            .tick_length(5)    // Set tick mark length
            .tick_width(1)     // Set tick mark width
            .tick_color("black"))  // Set tick mark color
        .y_axis(plotly::layout::Axis::new()
            .title("Y Position (m)")
            .range(vec![field_bounds.2 as f64, field_bounds.3 as f64])
            .auto_range(false)  // Disable auto range to use exact range
            .tick_font(plotly::common::Font::new().size(16))
            .show_grid(false)  // Remove grid
            .show_line(true)   // Show axis line
            .line_color("black")
            .line_width(1)
            .zero_line(false)  // Disable zero line
            .ticks(plotly::layout::TicksDirection::Outside)  // Show tick marks outside
            .tick_length(5)    // Set tick mark length
            .tick_width(1)     // Set tick mark width
            .tick_color("black"))  // Set tick mark color
        .legend(plotly::layout::Legend::new()
            .x(1.02)
            .y(1.0)
            .font(plotly::common::Font::new().size(16)))
        .plot_background_color("white")
        .paper_background_color("white")
        .show_legend(true);
    
    plot.set_layout(layout);
    
    // Save plot
    let timestamp = chrono::Utc::now().format("%Y%m%d_%H%M%S");
    let filename_html = format!("results/multi_station_optimization_energy_{}.html", timestamp);
    let filename_svg = format!("results/multi_station_optimization_energy_{}.svg", timestamp);

    plot.write_html(&filename_html);
    if let Err(e) = plot.write_image(&filename_svg, ImageFormat::SVG, 890, 800, 1.0) {
        eprintln!("Failed to write SVG file: {}", e);
        println!("Multi-station energy plot saved to: {}", filename_html);
    } else {
        println!("Multi-station energy plot saved to: {} and {}", filename_html, filename_svg);
    }
}

/// Generate 2D multi-station visualization focused on total traveling distance
pub fn generate_multi_station_distance_plot(
    optimal_stations: &[Pos2],
    optimal_distance: f64,
    suboptimal_configs: &[(Vec<Pos2>, f64)], // Station positions with their total distances
    obstacles: &[Obstacle],
    field_bounds: (f32, f32, f32, f32), // (min_x, max_x, min_y, max_y)
) {
    let mut plot = Plot::new();
    
    // Collect all distance values for ranking
    let mut all_configs = vec![(optimal_stations.to_vec(), optimal_distance)];
    for (stations, distance) in suboptimal_configs {
        all_configs.push((stations.clone(), *distance));
    }
    
    // Sort by distance (ascending) to determine color ranking
    let mut distance_rank: Vec<(usize, f64)> = all_configs.iter().enumerate()
        .map(|(i, (_, distance))| (i, *distance))
        .collect();
    distance_rank.sort_by(|a, b| a.1.partial_cmp(&b.1).unwrap());
    
    // Heatmap-style color scheme (blue to red) - blue for lower distances, red for higher
    let colors = [
        "#321decff",  "#5754f7ff", "#adb3ffff", "#f0c49cff", "#f3a172ff", "#f3584dff", "#ee2e2eff"
    ];
    
    // Add optimal configuration (first in sorted list)
    let opt_x: Vec<f64> = optimal_stations.iter().map(|p| p.x as f64).collect();
    let opt_y: Vec<f64> = optimal_stations.iter().map(|p| p.y as f64).collect();
    let optimal_rank = distance_rank.iter().position(|(i, _)| *i == 0).unwrap_or(0);
    let optimal_color_idx = optimal_rank.min(colors.len() - 1);
    
    let optimal_trace = Scatter::new(opt_x.clone(), opt_y.clone())
        .mode(Mode::Markers)
        .name(&format!("Optimal ({:.1} m)", optimal_distance))
        .marker(Marker::new()
            .size(18)
            .color(colors[optimal_color_idx])
            .line(plotly::common::Line::new().width(3.0).color("black"))
            .symbol(plotly::common::MarkerSymbol::Star)); // Always use star for optimal
    
    plot.add_trace(optimal_trace);
    
    // Add suboptimal configurations
    for (i, (stations, distance)) in suboptimal_configs.iter().enumerate() {
        let x_coords: Vec<f64> = stations.iter().map(|p| p.x as f64).collect();
        let y_coords: Vec<f64> = stations.iter().map(|p| p.y as f64).collect();
        
        let config_rank = distance_rank.iter().position(|(idx, _)| *idx == i + 1).unwrap_or(colors.len() - 1);
        let color_idx = config_rank.min(colors.len() - 1);
        
        // Use different marker shape for each config (cycle through available shapes)
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
            .name(&format!("Config {} ({:.1} m)", i + 1, distance))
            .marker(Marker::new()
                .size(14)
                .color(colors[color_idx])
                .line(plotly::common::Line::new().width(2.0).color("black"))
                .symbol(marker_symbol));
        
        plot.add_trace(trace);
    }
    
    // Add obstacles
    for (i, obstacle) in obstacles.iter().enumerate() {
        let mut obstacle_x = Vec::new();
        let mut obstacle_y = Vec::new();

        for point in &obstacle.points {
            obstacle_x.push(point.x as f64);
            obstacle_y.push(point.y as f64);
        }

        // Close the polygon
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
    
    // Add field boundaries as visible lines
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
    
    // Create layout matching heatmap style
    let layout = Layout::new()
        .title("Charging Station Position Optimization Results - Total Traveling Distance")
        .width(890)  // Match heatmap size
        .height(800)
        .font(plotly::common::Font::new().size(16).color("black"))
        .x_axis(plotly::layout::Axis::new()
            .title("X Position (m)")
            .range(vec![field_bounds.0 as f64, field_bounds.1 as f64])
            .auto_range(false)  // Disable auto range to use exact range
            .tick_font(plotly::common::Font::new().size(16))
            .show_grid(false)  // Remove grid
            .show_line(true)   // Show axis line
            .line_color("black")
            .line_width(1)
            .zero_line(false)  // Disable zero line
            .ticks(plotly::layout::TicksDirection::Outside)  // Show tick marks outside
            .tick_length(5)    // Set tick mark length
            .tick_width(1)     // Set tick mark width
            .tick_color("black"))  // Set tick mark color
        .y_axis(plotly::layout::Axis::new()
            .title("Y Position (m)")
            .range(vec![field_bounds.2 as f64, field_bounds.3 as f64])
            .auto_range(false)  // Disable auto range to use exact range
            .tick_font(plotly::common::Font::new().size(16))
            .show_grid(false)  // Remove grid
            .show_line(true)   // Show axis line
            .line_color("black")
            .line_width(1)
            .zero_line(false)  // Disable zero line
            .ticks(plotly::layout::TicksDirection::Outside)  // Show tick marks outside
            .tick_length(5)    // Set tick mark length
            .tick_width(1)     // Set tick mark width
            .tick_color("black"))  // Set tick mark color
        .legend(plotly::layout::Legend::new()
            .x(1.02)
            .y(1.0)
            .font(plotly::common::Font::new().size(16)))
        .plot_background_color("white")
        .paper_background_color("white")
        .show_legend(true);
    
    plot.set_layout(layout);
    
    // Save plot
    let timestamp = chrono::Utc::now().format("%Y%m%d_%H%M%S");
    let filename_html = format!("results/multi_station_distance_{}.html", timestamp);
    let filename_svg = format!("results/multi_station_distance_{}.svg", timestamp);
    
    plot.write_html(&filename_html);
    if let Err(e) = plot.write_image(&filename_svg, ImageFormat::SVG, 890, 800, 1.0) {
        eprintln!("Failed to write SVG file: {}", e);
        println!("Multi-station distance plot saved to: {}", filename_html);
    } else {
        println!("Multi-station distance plot saved to: {} and {}", filename_html, filename_svg);
    }
}

/// Example function demonstrating how to generate plots for multi-station configurations
pub fn multi_station_plot_function() {
    use egui::Pos2;
    
    // Load scene configuration to get field boundaries and obstacles (same as other plot functions)
    let scene_config: SceneConfig = crate::utilities::utils::load_json_or_panic(DEFAULT_SCENE_CONFIG_PATH.to_string());
    let field_config: FieldConfig = crate::utilities::utils::load_json_or_panic(scene_config.field_config_path.clone());
    let obstacles = field_config.get_obstacles();
    
    // Define field boundaries (same as used in grid search experiments)
    const FIELD_MIN_X: f32 = 0.0;
    const FIELD_MAX_X: f32 = 25.0;
    const FIELD_MIN_Y: f32 = 0.0;
    const FIELD_MAX_Y: f32 = 25.0;
    let field_bounds = (FIELD_MIN_X, FIELD_MAX_X, FIELD_MIN_Y, FIELD_MAX_Y);
    
    // Example optimal and suboptimal station configurations (scaled to field dimensions)
    let optimal_stations = vec![Pos2::new(17.11, 1.29), Pos2::new(12.47, 14.34)];
    let optimal_energy = 15049.52;
    let optimal_distance = 46105.30; // Example optimal total traveling distance

    let suboptimal_configs_energy = vec![        
        (vec![Pos2::new(12.5, 7.5), Pos2::new(12.5, 17.5)], 15110.14),
        (vec![Pos2::new(17.5, 1.5), Pos2::new(20.0, 1.5)], 15231.02),
        (vec![Pos2::new(12.5, 11.5), Pos2::new(12.5, 13.5)], 15341.80),
        (vec![Pos2::new(6.25, 1.5), Pos2::new(6.25, 23.5)], 15494.18),        
        (vec![Pos2::new(1.5, 1.5), Pos2::new(23.0, 23.5)], 15497.50),
        (vec![Pos2::new(1.5, 12.5), Pos2::new(23.0, 12.5)], 15726.52),
    ];
    
    // Example distance data for the same configurations (in meters)
    let suboptimal_configs_distance = vec![   
        (vec![Pos2::new(12.5, 7.5), Pos2::new(12.5, 17.5)], 46306.12),
        (vec![Pos2::new(17.5, 1.5), Pos2::new(20.0, 1.5)], 46797.55),
        (vec![Pos2::new(12.5, 11.5), Pos2::new(12.5, 13.5)], 47181.80),
        (vec![Pos2::new(6.25, 1.5), Pos2::new(6.25, 23.5)], 47766.84),
        (vec![Pos2::new(1.5, 1.5), Pos2::new(23.0, 23.5)], 47773.37), 
        (vec![Pos2::new(1.5, 12.5), Pos2::new(23.0, 12.5)], 48678.67),
    ];
    
    // Generate energy-focused plot
    generate_multi_station_plot(
        &optimal_stations,
        optimal_energy,
        &suboptimal_configs_energy,
        &obstacles,
        field_bounds,
    );
    
    // Generate distance-focused plot
    generate_multi_station_distance_plot(
        &optimal_stations,
        optimal_distance,
        &suboptimal_configs_distance,
        &obstacles,
        field_bounds,
    );
    
    // Generate energy comparison plot with all configurations
    let mut all_configs_energy = vec![(optimal_stations.clone(), optimal_energy)];
    all_configs_energy.extend(suboptimal_configs_energy);
    
    println!("Demo plots generated successfully!");
    println!("Energy plot: Optimal configuration 2 stations at ({:.1}, {:.1}) and ({:.1}, {:.1}) with {:.1} Wh energy consumption",
        optimal_stations[0].x, optimal_stations[0].y, optimal_stations[1].x, optimal_stations[1].y, optimal_energy);
    println!("Distance plot: Same stations with {:.1} m total traveling distance", optimal_distance);
    println!("Field boundaries: ({:.1}, {:.1}) to ({:.1}, {:.1})", 
        field_bounds.0, field_bounds.2, field_bounds.1, field_bounds.3);
    println!("Loaded {} obstacles from configuration", obstacles.len());
}
