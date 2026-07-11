use crate::environment::geometry::FieldBounds;
use crate::experiment::geometry::is_inside_field_bounds;
use egui::Pos2;

/// Round coordinates to 2 decimal places (centimeters) to prevent floating-point precision issues
pub fn round_to_centimeters(pos: Pos2) -> Pos2 {
    Pos2::new(
        (pos.x * 100.0).round() / 100.0,
        (pos.y * 100.0).round() / 100.0
    )
}

/// A position is valid for station placement as long as it doesn't fall
/// inside any field's (padded) bounding rectangle.
pub fn is_position_valid(position: Pos2, field_bounds: &[FieldBounds]) -> bool {
    !is_inside_field_bounds(position, field_bounds)
}