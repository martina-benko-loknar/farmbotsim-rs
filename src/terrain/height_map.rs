use crate::environment::geometry::FieldBounds;

#[derive(Clone, Debug, PartialEq)]
pub struct HeightMap {
    pub width: usize,
    pub height: usize,

    /// size of one grid cell in meters
    pub cell_size_x_m: f64,
    pub cell_size_y_m: f64,

    /// elevation grid in meters
    pub elevations: Vec<Vec<f64>>,

    /// origin in world-coordinates (meters)
    pub origin_x: f64,
    pub origin_y: f64,
}

impl HeightMap {

    // GRID SPACE
    pub fn elevation_at(&self, gx: usize, gy: usize) -> f64 {
        self.elevations
            .get(gy)
            .and_then(|row| row.get(gx))
            .copied()
            .unwrap_or(0.0)
    }

    /// WORLD SPACE: Returns height (meters) at a world position (x, y in meters)
    // height_at_world --> convert to gx, gy --> elevation_at(gx, gy)
    pub fn height_at_world(&self, x_m: f64, y_m: f64) -> Option<f64> {
        let gx = ((x_m - self.origin_x as f64) / self.cell_size_x_m) as isize;
        let gy = ((y_m - self.origin_y as f64) / self.cell_size_y_m) as isize;

        if gx < 0 || gy < 0 {
            return None;
        }

        let gx = gx as usize;
        let gy = gy as usize;

        if gx >= self.width || gy >= self.height {
            return None;
        }

        Some(self.elevations[gy][gx])
    }

    /// Computes slope (radians) between two world positions
    pub fn slope_between(
        &self,
        x1_m: f64,
        y1_m: f64,
        x2_m: f64,
        y2_m: f64,
    ) -> Option<f64> {

        let h1 = self.height_at_world(x1_m, y1_m)?;
        let h2 = self.height_at_world(x2_m, y2_m)?;

        let dx = x2_m - x1_m;
        let dy = y2_m - y1_m;

        let horizontal_dist = (dx * dx + dy * dy).sqrt();

        if horizontal_dist < 1e-6 {
            return Some(0.0);
        }

        let dz = h2 - h1;

        Some((dz / horizontal_dist).atan())
    }

    pub fn grid_to_world(&self, gx: usize, gy: usize) -> (f64, f64) {
        let x = self.origin_x + gx as f64 * self.cell_size_x_m;
        let y = self.origin_y + gy as f64 * self.cell_size_y_m;
        (x, y)
    }

    pub fn world_to_grid(&self, x: f64, y: f64) -> (usize, usize) {
        let gx = ((x - self.origin_x) / self.cell_size_x_m).floor() as usize;
        let gy = ((y - self.origin_y) / self.cell_size_y_m).floor() as usize;

        (gx, gy)
}

    pub fn print_points(&self) {
        println!("(x, y): elevation");
        println!("grid size: {} × {} points", self.width, self.height);

        for gy in 0..self.height {
            for gx in 0..self.width {
                let h = self.elevation_at(gx, gy);

                if h == 0.0 {
                    continue;
                }

                let (x_m, y_m) = self.grid_to_world(gx, gy);

                println!("({:.2}, {:.2}): {:.3}", x_m, y_m, h);
            }
        }
    }

    pub fn bounds(&self) -> FieldBounds {

        let max_x =
            self.origin_x
            + (self.width.saturating_sub(1)) as f64
                * self.cell_size_x_m;

        let max_y =
            self.origin_y
            + (self.height.saturating_sub(1)) as f64
                * self.cell_size_y_m;

        FieldBounds {
            min_x: self.origin_x as f32,
            max_x: max_x as f32,
            min_y: self.origin_y as f32,
            max_y: max_y as f32,
        }
    }

}
