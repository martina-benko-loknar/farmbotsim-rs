#[derive(Clone, Debug, PartialEq)]
pub struct HeightMap {
    pub width: usize,
    pub height: usize,

    /// size of one grid cell in meters
    pub cell_size_x_m: f64,
    pub cell_size_y_m: f64,

    /// elevation grid in meters
    pub elevations: Vec<Vec<f32>>,

    /// origin in world-coordinates (meters)
    pub origin_x: f64,
    pub origin_y: f64,
}

// DEBUG TEST: 
impl HeightMap {

    pub fn elevation_at(&self, gx: usize, gy: usize) -> f32 {
        self.elevations
            .get(gy)
            .and_then(|row| row.get(gx))
            .copied()
            .unwrap_or(0.0)
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
}
