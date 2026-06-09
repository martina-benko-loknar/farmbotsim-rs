use crate::environment::field_config::FieldConfig;
use crate::environment::field_config::VariantFieldConfig;

use serde::{Serialize, Deserialize};

#[derive(Debug, Clone, Copy, Serialize, Deserialize)]
pub struct FieldBounds {
    pub min_x: f32,
    pub max_x: f32,
    pub min_y: f32,
    pub max_y: f32,
}

impl FieldBounds {
    pub fn from_field_config(cfg: &FieldConfig) -> Self {

        let mut min_x = f32::INFINITY;
        let mut min_y = f32::INFINITY;
        let mut max_x = f32::NEG_INFINITY;
        let mut max_y = f32::NEG_INFINITY;

        for config in &cfg.configs {

            match config {

                VariantFieldConfig::Line(c) => {

                    // Row direction from angle
                    let theta = c.angle.to_base_unit().to_radians();

                    let dir_x = theta.cos();
                    let dir_y = theta.sin();

                    // Perpendicular direction for row spacing
                    let perp_x = -dir_y;
                    let perp_y = dir_x;

                    let row_length = c.length.to_base_unit();
                    let row_spacing = c.line_spacing.to_base_unit();

                    for i in 0..c.n_lines {

                        let offset = i as f32 * row_spacing;

                        // Start point of row
                        let start_x =
                            c.left_top_pos.x + perp_x * offset;

                        let start_y =
                            c.left_top_pos.y + perp_y * offset;

                        // End point of row
                        let end_x =
                            start_x + dir_x * row_length;

                        let end_y =
                            start_y + dir_y * row_length;

                        // Update bounds with both endpoints
                        min_x = min_x.min(start_x).min(end_x);
                        min_y = min_y.min(start_y).min(end_y);

                        max_x = max_x.max(start_x).max(end_x);
                        max_y = max_y.max(start_y).max(end_y);
                    }
                }

                VariantFieldConfig::Point(c) => {

                    let width =
                        c.line_spacing.to_base_unit()
                        * (c.n_lines.saturating_sub(1)) as f32;

                    let height =
                        c.point_spacing.to_base_unit()
                        * (c.n_points_per_line.saturating_sub(1)) as f32;

                    let x0 = c.left_top_pos.x;
                    let y0 = c.left_top_pos.y;

                    let x1 = x0 + width;
                    let y1 = y0 + height;

                    min_x = min_x.min(x0).min(x1);
                    min_y = min_y.min(y0).min(y1);

                    max_x = max_x.max(x0).max(x1);
                    max_y = max_y.max(y0).max(y1);
                }
            }
        }

        Self {
            min_x,
            max_x,
            min_y,
            max_y,
        }
    }
}


// impl FieldBounds {
//     pub fn from_field_config(cfg: &FieldConfig) -> Self {
//         let mut min_x = f32::INFINITY;
//         let mut min_y = f32::INFINITY;
//         let mut max_x = f32::NEG_INFINITY;
//         let mut max_y = f32::NEG_INFINITY;

//         for config in &cfg.configs {
//             match config {
//                 VariantFieldConfig::Line(c) => {
//                     min_x = min_x.min(c.left_top_pos.x);
//                     min_y = min_y.min(c.left_top_pos.y);

//                     // approximate extents
//                     let dir = c.length.to_base_unit();
//                     let spacing = c.line_spacing.to_base_unit();

//                     max_x = max_x.max(c.left_top_pos.x + dir + spacing * c.n_lines as f32);
//                     max_y = max_y.max(c.left_top_pos.y + dir + spacing * c.n_lines as f32);
//                 }

//                 VariantFieldConfig::Point(c) => {
//                     min_x = min_x.min(c.left_top_pos.x);
//                     min_y = min_y.min(c.left_top_pos.y);

//                     let width = c.line_spacing.to_base_unit() * c.n_lines as f32;
//                     let height = c.point_spacing.to_base_unit() * c.n_points_per_line as f32;

//                     max_x = max_x.max(c.left_top_pos.x + width);
//                     max_y = max_y.max(c.left_top_pos.y + height);
//                 }
//             }
//         }

//         Self { min_x, max_x, min_y, max_y }
//     }
// }

// pub fn compute_field_bounds(
//     obstacles: &[Obstacle]
// ) -> (f32, f32, f32, f32) {

//     let mut min_x = f32::INFINITY;
//     let mut max_x = f32::NEG_INFINITY;

//     let mut min_y = f32::INFINITY;
//     let mut max_y = f32::NEG_INFINITY;

//     for obs in obstacles {

//         for p in &obs.points {

//             min_x = min_x.min(p.x);
//             max_x = max_x.max(p.x);

//             min_y = min_y.min(p.y);
//             max_y = max_y.max(p.y);
//         }
//     }

//     (
//         min_x,
//         max_x,
//         min_y,
//         max_y,
//     )
// }