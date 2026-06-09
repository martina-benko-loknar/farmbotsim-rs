use crate::environment::obstacle::Obstacle;
use egui::Pos2;
use crate::experiment::search_domain::SearchDomain;
use crate::environment::geometry::FieldBounds;
use crate::environment::field_config::LineFieldConfig;
use crate::environment::field_config::FieldConfig;
use crate::environment::field_config::VariantFieldConfig;

pub struct FieldPolygon {
    pub points: Vec<Pos2>, // closed loop
}

pub fn generate_row_gap_obstacles(
    field_config: &FieldConfig,
    gap_margin: f32,
) -> Vec<Obstacle> {

    let mut obstacles = Vec::new();

    for cfg in &field_config.configs {

        if let VariantFieldConfig::Line(field) = cfg {

            let origin = field.left_top_pos;

            let angle_deg = field.angle.to_degrees();

            let length = field.length.to_base_unit();

            let spacing = field.line_spacing.to_base_unit();

            let n_lines = field.n_lines;

            // rows approximately horizontal?
            let x_aligned =
                angle_deg.rem_euclid(180.0) < 45.0
                || angle_deg.rem_euclid(180.0) > 135.0;

                // for i in 0..(n_lines - 1) {

                //     let local_rect = if x_aligned {

                //         let y0 = i as f32 * spacing + gap_margin;
                //         let y1 = (i as f32 + 1.0) * spacing - gap_margin;

                //         vec![
                //             Pos2::new(0.0, y0),
                //             Pos2::new(length, y0),
                //             Pos2::new(length, y1),
                //             Pos2::new(0.0, y1),
                //         ]

                //     } else {

                //         let x0 = i as f32 * spacing + gap_margin;
                //         let x1 = (i as f32 + 1.0) * spacing - gap_margin;

                //         vec![
                //             Pos2::new(x0, 0.0),
                //             Pos2::new(x1, 0.0),
                //             Pos2::new(x1, length),
                //             Pos2::new(x0, length),
                //         ]
                //     };

                //     let world_points =
                //         local_rect
                //             .into_iter()
                //             .map(|p| rotate_local_point(p, origin, angle_deg))
                //             .collect();

                //     obstacles.push(Obstacle {
                //         points: world_points,
                //     });
                // }

            // create one forbidden strip between every pair of rows
            for i in 0..(n_lines - 1) {

                let offset = (i as f32 + 1.0) * spacing;

                let half_gap = (spacing * 0.5) - gap_margin;

                let local_rect = if x_aligned {

                    // rows run along X axis
                    // forbidden strip spans horizontally

                    vec![
                        Pos2::new(0.0, offset - half_gap),
                        Pos2::new(length, offset - half_gap),
                        Pos2::new(length, offset + half_gap),
                        Pos2::new(0.0, offset + half_gap),
                    ]

                } else {

                    // rows run along Y axis
                    // forbidden strip spans vertically

                    vec![
                        Pos2::new(offset - half_gap, 0.0),
                        Pos2::new(offset + half_gap, 0.0),
                        Pos2::new(offset + half_gap, length),
                        Pos2::new(offset - half_gap, length),
                    ]
                };

                // rotate + translate into world coordinates
                let world_points =
                    local_rect
                        .into_iter()
                        .map(|p| rotate_local_point(p, origin, angle_deg))
                        .collect();

                obstacles.push(Obstacle {
                    points: world_points,
                });
            }

        }
    }

    obstacles
}

fn rotate_local_point(
    local: Pos2,
    origin: Pos2,
    angle_deg: f32,
) -> Pos2 {

    let rad = angle_deg.to_radians();

    let cos = rad.cos();
    let sin = rad.sin();

    Pos2::new(
        origin.x + local.x * cos - local.y * sin,
        origin.y + local.x * sin + local.y * cos,
    )
}

pub fn field_polygon(cfg: &LineFieldConfig) -> FieldPolygon {
    let origin = cfg.left_top_pos;
    let angle = cfg.angle.to_degrees().to_radians();

    let length = cfg.length.to_base_unit();
    //let width = cfg.line_spacing.to_base_unit() * cfg.n_lines as f32;
    let width = cfg.line_spacing.to_base_unit() * (cfg.n_lines as f32 - 1.0);

    let cos = angle.cos();
    let sin = angle.sin();

    // local rectangle corners (before rotation)
    let local = [
        Pos2::new(0.0, 0.0),
        Pos2::new(length, 0.0),
        Pos2::new(length, width),
        Pos2::new(0.0, width),
    ];

    let mut world = Vec::with_capacity(4);

    for p in local {
        let rotated = Pos2::new(
            origin.x + p.x * cos - p.y * sin,
            origin.y + p.x * sin + p.y * cos,
        );
        world.push(rotated);
    }

    FieldPolygon { points: world }
}

pub fn is_point_in_polygon(point: Pos2, poly: &FieldPolygon) -> bool {
    let pts = &poly.points;

    if pts.len() < 3 {
        return false;
    }

    let mut inside = false;
    let mut j = pts.len() - 1;

    for i in 0..pts.len() {
        let pi = pts[i];
        let pj = pts[j];

        if ((pi.y > point.y) != (pj.y > point.y)) &&
            (point.x < (pj.x - pi.x) * (point.y - pi.y) / (pj.y - pi.y) + pi.x)
        {
            inside = !inside;
        }

        j = i;
    }

    inside
}

pub fn is_in_vineyard(
    position: Pos2,
    vineyards: &[FieldPolygon],
) -> bool {
    vineyards.iter().any(|v| is_point_in_polygon(position, v))
}

pub fn vineyard_polygons(cfg: &FieldConfig) -> Vec<FieldPolygon> {
    cfg.configs.iter().filter_map(|c| {
        if let VariantFieldConfig::Line(line) = c {
            Some(field_polygon(line))
        } else {
            None
        }
    }).collect()
}

/// Generate valid grid points that don't intersect with obstacles
pub fn generate_valid_grid_points(
    domain: &SearchDomain,
    resolution: usize,
    obstacles: &[Obstacle],
    obstacle_margin: f32,
    vineyards: &[FieldPolygon],
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
                vineyards,
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
    vineyards: &[FieldPolygon],
) -> bool {

    if is_in_vineyard(position, vineyards) {
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
