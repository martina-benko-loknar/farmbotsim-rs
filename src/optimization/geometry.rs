use crate::environment::{
    obstacle::Obstacle
};
use egui::Pos2;

// Define field boundaries for optimization (adjust these based on your actual farm layout)
const OBSTACLE_MARGIN: f32 = 0.4; // Keep stations at least 0.4m from obstacles

/// Round coordinates to 2 decimal places (centimeters) to prevent floating-point precision issues
pub fn round_to_centimeters(pos: Pos2) -> Pos2 {
    Pos2::new(
        (pos.x * 100.0).round() / 100.0,
        (pos.y * 100.0).round() / 100.0
    )
}

// Check if a position is valid (not inside or too close to obstacles)
pub fn is_position_valid(position: Pos2, obstacles: &[Obstacle]) -> bool {
    for obstacle in obstacles {
        // Check if position is inside obstacle or too close to it
        if is_point_near_obstacle(position, obstacle, OBSTACLE_MARGIN) {
            return false;
        }
    }
    true
}

// Check if a point is inside an obstacle or within a certain margin
fn is_point_near_obstacle(point: Pos2, obstacle: &Obstacle, margin: f32) -> bool {
    // First check if point is inside the obstacle polygon
    if is_point_inside_polygon(point, &obstacle.points) {
        return true;
    }
    
    // Check if point is too close to any edge of the obstacle
    for window in obstacle.points.windows(2) {
        if let [p1, p2] = window {
            let distance = point_to_line_distance(point, *p1, *p2);
            if distance < margin {
                return true;
            }
        }
    }
    
    // Also check the closing edge (last point to first point)
    if obstacle.points.len() >= 2 {
        let first = obstacle.points[0];
        let last = obstacle.points[obstacle.points.len() - 1];
        let distance = point_to_line_distance(point, last, first);
        if distance < margin {
            return true;
        }
    }
    
    false
}

// Ray casting algorithm to determine if point is inside polygon
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

// Calculate distance from point to line segment
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