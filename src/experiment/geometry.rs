use crate::environment::obstacle::Obstacle;
use egui::Pos2;
use crate::experiment::search_domain::SearchDomain;
use crate::environment::geometry::FieldBounds;
use crate::environment::field_config::FieldConfig;
use crate::environment::field_config::VariantFieldConfig;

/// Forbidden strips between every pair of adjacent rows in a field group.
///
/// Uses the same row/spacing-axis convention as `FieldConfig::get_obstacles()`
/// and `FieldBounds::per_group_from_field_config()`: the row itself runs
/// along `angle + 90°`, and the spacing between parallel rows runs along
/// `angle`. Keeping this consistent matters -- a mismatched convention here
/// previously produced gap strips with the wrong orientation, which spanned
/// much further than the real field and rejected valid positions well
/// outside it (see various/field_bounds_angle_convention_bug.txt).
pub fn generate_row_gap_obstacles(
    field_config: &FieldConfig,
    gap_margin: f32,
) -> Vec<Obstacle> {

    let mut obstacles = Vec::new();

    for cfg in &field_config.configs {

        if let VariantFieldConfig::Line(field) = cfg {

            let origin = field.left_top_pos;

            // to_base_unit() already returns radians
            let theta = field.angle.to_base_unit();

            let row_dir_x = -theta.sin();
            let row_dir_y = theta.cos();

            let spacing_dir_x = theta.cos();
            let spacing_dir_y = theta.sin();

            let length = field.length.to_base_unit();

            let spacing = field.line_spacing.to_base_unit();

            let n_lines = field.n_lines;

            // create one forbidden strip between every pair of rows
            for i in 0..(n_lines - 1) {

                let offset = (i as f32 + 1.0) * spacing;

                let half_gap = (spacing * 0.5) - gap_margin;

                let near = offset - half_gap;
                let far = offset + half_gap;

                let p1 = Pos2::new(
                    origin.x + spacing_dir_x * near,
                    origin.y + spacing_dir_y * near,
                );
                let p2 = Pos2::new(
                    origin.x + spacing_dir_x * far,
                    origin.y + spacing_dir_y * far,
                );
                let p3 = Pos2::new(
                    p2.x + row_dir_x * length,
                    p2.y + row_dir_y * length,
                );
                let p4 = Pos2::new(
                    p1.x + row_dir_x * length,
                    p1.y + row_dir_y * length,
                );

                obstacles.push(Obstacle {
                    points: vec![p1, p2, p3, p4],
                });
            }

        }
    }

    obstacles
}

/// Generate valid grid points that don't intersect with obstacles or fall inside any field
pub fn generate_valid_grid_points(
    domain: &SearchDomain,
    resolution: usize,
    obstacles: &[Obstacle],
    obstacle_margin: f32,
    field_bounds: &[FieldBounds],
) -> Vec<Pos2> {
    let mut valid_points = Vec::new();

    let step_x = (domain.max_x - domain.min_x) / (resolution - 1) as f32;
    let step_y = (domain.max_y - domain.min_y) / (resolution - 1) as f32;

    for i in 0..resolution {
        for j in 0..resolution {
            let x = domain.min_x + i as f32 * step_x;
            let y = domain.min_y + j as f32 * step_y;

            let point = round_to_centimeters(Pos2::new(x, y));

            if is_station_position_valid(
                point,
                obstacles,
                obstacle_margin,
                field_bounds,
            ) {
                valid_points.push(point);
            }
        }
    }

    valid_points
}

/// Check if a position is valid (not inside or too close to obstacles)
pub fn is_clear_of_obstacles(position: Pos2, obstacles: &[Obstacle], margin: f32) -> bool {
    for obstacle in obstacles {
        // Check if position is inside obstacle
        if is_point_inside_polygon(position, &obstacle.points) {
            return false;
        }
        
        // Check if position is too close to any edge of the obstacle
        for window in obstacle.points.windows(2) {
            if let [p1, p2] = window {
                let distance = point_to_line_distance(position, *p1, *p2);
                if distance < margin {
                    return false;
                }
            }
        }
        
        // Check the closing edge (last point to first point)
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

pub fn is_inside_field_bounds(
    position: Pos2,
    field_bounds: &[FieldBounds],
) -> bool {

    for bounds in field_bounds {

        if position.x >= bounds.min_x
            && position.x <= bounds.max_x
            && position.y >= bounds.min_y
            && position.y <= bounds.max_y
        {
            return true;
        }
    }

    false
}

// pub fn is_station_position_valid(
//     position: Pos2,
//     obstacles: &[Obstacle],
//     obstacle_margin: f32,
// ) -> bool {

//     // for cfg in &field_config.configs {
//     //     if let VariantFieldConfig::Line(line) = cfg {
//     //         if is_point_in_line_field_band(position, line, row_margin) {
//     //             return false;
//     //         }
//     //     }
//     // }

//     // 2. obstacle constraint
//     if !is_clear_of_obstacles(position, obstacles, obstacle_margin) {
//         return false;
//     }

//     true
// }

pub fn is_station_position_valid(
    position: Pos2,
    obstacles: &[Obstacle],
    obstacle_margin: f32,
    field_bounds: &[FieldBounds],
) -> bool {

    if is_inside_field_bounds(position, field_bounds) {
        return false;
    }

    if !is_clear_of_obstacles(position, obstacles, obstacle_margin) {
        return false;
    }

    true
}

// pub fn is_station_position_valid(
//     position: Pos2,
//     obstacles: &[Obstacle],
//     obstacle_margin: f32,
//     cultivation_bounds: &[FieldBounds],
//     field_configs: &[LineFieldConfig],
//     row_margin: f32,
// ) -> bool {

//     for field in field_configs {
//         if is_in_row_band(position, field, row_margin) {
//             return false;
//         }
//     }

//     // Reject points inside vineyard regions
//     if is_inside_field_bounds(position, cultivation_bounds) {
//         return false;
//     }

//     // Reject obstacle collisions
//     if !is_clear_of_obstacles(
//         position,
//         obstacles,
//         obstacle_margin,
//     ) {
//         return false;
//     }

//     true
// }

/// Ray casting algorithm to determine if point is inside polygon
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

/// Calculate distance from point to line segment
fn point_to_line_distance(point: Pos2, line_start: Pos2, line_end: Pos2) -> f32 {
    let line_vec = Pos2::new(line_end.x - line_start.x, line_end.y - line_start.y);
    let point_vec = Pos2::new(point.x - line_start.x, point.y - line_start.y);
    
    let line_len_sq = line_vec.x * line_vec.x + line_vec.y * line_vec.y;
    if line_len_sq == 0.0 {
        // Line start and end are the same point
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

/// Round coordinates to 2 decimal places (centimeters) to prevent floating-point precision issues
pub fn round_to_centimeters(pos: Pos2) -> Pos2 {
    Pos2::new(
        (pos.x * 100.0).round() / 100.0,
        (pos.y * 100.0).round() / 100.0
    )
}

/// Interpolate results to create smooth heatmap
pub fn interpolate_results(
    results: &[(Pos2, f64)], 
    obstacles: &[Obstacle], 
    resolution: usize
) -> (Vec<f64>, Vec<f64>, Vec<Vec<f64>>) {
    // Find bounds
    let min_x = results.iter().map(|(pos, _)| pos.x).fold(f32::INFINITY, f32::min);
    let max_x = results.iter().map(|(pos, _)| pos.x).fold(f32::NEG_INFINITY, f32::max);
    let min_y = results.iter().map(|(pos, _)| pos.y).fold(f32::INFINITY, f32::min);
    let max_y = results.iter().map(|(pos, _)| pos.y).fold(f32::NEG_INFINITY, f32::max);
    
    let step_x = (max_x - min_x) / (resolution - 1) as f32;
    let step_y = (max_y - min_y) / (resolution - 1) as f32;
    
    let mut x_grid = Vec::new();
    let mut y_grid = Vec::new();
    let mut z_grid = vec![vec![f64::NAN; resolution]; resolution];
    
    // Create grid coordinates
    for i in 0..resolution {
        x_grid.push((min_x + i as f32 * step_x) as f64);
    }
    for j in 0..resolution {
        y_grid.push((min_y + j as f32 * step_y) as f64);
    }
    
    // Interpolate values using inverse distance weighting
    for (i, &x) in x_grid.iter().enumerate() {
        for (j, &y) in y_grid.iter().enumerate() {
            let point = Pos2::new(x as f32, y as f32);
            
            // Skip points that are not valid charging station positions (same validation as grid generation)
            if !is_clear_of_obstacles(point, obstacles, 0.4) {
                z_grid[j][i] = f64::NAN; // Will appear as gap in heatmap
                continue;
            }
            
            // Inverse distance weighting interpolation
            let mut weighted_sum = 0.0;
            let mut weight_sum = 0.0;
            
            for (result_pos, energy) in results {
                let distance = ((point.x - result_pos.x).powi(2) + (point.y - result_pos.y).powi(2)).sqrt();
                
                if distance < 0.001 {
                    // Very close to a data point, use exact value
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
