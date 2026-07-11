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
    /// Bounding rectangle per configured field group (one per `Line`/`Point`
    /// config), matching the same row/spacing-axis convention as
    /// `FieldConfig::get_obstacles()` (row direction runs along `angle + 90°`,
    /// row-to-row spacing runs along `angle`) so this stays geometrically
    /// consistent with the physical rows actually rendered/simulated.
    pub fn per_group_from_field_config(cfg: &FieldConfig) -> Vec<Self> {

        cfg.configs.iter().map(|config| {

            let mut min_x = f32::INFINITY;
            let mut min_y = f32::INFINITY;
            let mut max_x = f32::NEG_INFINITY;
            let mut max_y = f32::NEG_INFINITY;

            match config {

                VariantFieldConfig::Line(c) => {

                    // to_base_unit() already returns radians
                    let theta = c.angle.to_base_unit();

                    // Row direction runs along angle + 90°; spacing between
                    // rows runs along angle (matches get_obstacles()).
                    let row_dir_x = -theta.sin();
                    let row_dir_y = theta.cos();

                    let spacing_dir_x = theta.cos();
                    let spacing_dir_y = theta.sin();

                    let row_length = c.length.to_base_unit();
                    let row_spacing = c.line_spacing.to_base_unit();

                    // Matches get_obstacles(): n_lines + 1 rows, starting
                    // half a spacing before left_top_pos, so the bounding
                    // box covers the actual physical row extent rather than
                    // undershooting it by spacing/2 on the near edge.
                    for k in 0..=c.n_lines {

                        let offset = (k as f32 - 0.5) * row_spacing;

                        // Start point of row
                        let start_x =
                            c.left_top_pos.x + spacing_dir_x * offset;

                        let start_y =
                            c.left_top_pos.y + spacing_dir_y * offset;

                        // End point of row
                        let end_x =
                            start_x + row_dir_x * row_length;

                        let end_y =
                            start_y + row_dir_y * row_length;

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

            Self { min_x, max_x, min_y, max_y }

        }).collect()
    }

    /// Single bounding rectangle spanning all configured field groups
    /// combined. Useful for sizing the overall search domain, but does not
    /// distinguish which parts of that envelope are actually field vs open
    /// space between separate groups -- use `per_group_from_field_config`
    /// when checking whether a specific point falls inside a field.
    pub fn from_field_config(cfg: &FieldConfig) -> Self {

        let groups = Self::per_group_from_field_config(cfg);

        let mut min_x = f32::INFINITY;
        let mut min_y = f32::INFINITY;
        let mut max_x = f32::NEG_INFINITY;
        let mut max_y = f32::NEG_INFINITY;

        for g in groups {
            min_x = min_x.min(g.min_x);
            min_y = min_y.min(g.min_y);
            max_x = max_x.max(g.max_x);
            max_y = max_y.max(g.max_y);
        }

        Self {
            min_x,
            max_x,
            min_y,
            max_y,
        }
    }

    /// This rectangle expanded outward by `margin` on all sides.
    pub fn padded(&self, margin: f32) -> Self {
        Self {
            min_x: self.min_x - margin,
            max_x: self.max_x + margin,
            min_y: self.min_y - margin,
            max_y: self.max_y + margin,
        }
    }
}