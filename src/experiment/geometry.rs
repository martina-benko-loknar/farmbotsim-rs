use egui::Pos2;
use crate::experiment::search_domain::SearchDomain;
use crate::environment::geometry::FieldBounds;

/// Generate grid points whose centers don't fall inside any field.
pub fn generate_valid_grid_points(
    domain: &SearchDomain,
    resolution: usize,
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

            if is_station_position_valid(point, field_bounds) {
                valid_points.push(point);
            }
        }
    }

    valid_points
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

/// A position is valid for station placement as long as it doesn't fall
/// inside any field's (padded) bounding rectangle.
pub fn is_station_position_valid(
    position: Pos2,
    field_bounds: &[FieldBounds],
) -> bool {
    !is_inside_field_bounds(position, field_bounds)
}

/// Round coordinates to 2 decimal places (centimeters) to prevent floating-point precision issues
pub fn round_to_centimeters(pos: Pos2) -> Pos2 {
    Pos2::new(
        (pos.x * 100.0).round() / 100.0,
        (pos.y * 100.0).round() / 100.0
    )
}
