use crate::environment::{
    scene_config::SceneConfig,
    station_module::station_config::StationConfig,
};

use crate::optimization::geometry::round_to_centimeters;
use crate::optimization::geometry::is_position_valid;
use crate::environment::geometry::FieldBounds;
use crate::experiment::search_domain::SearchDomain;

use ndarray::{ArrayView2};
use rand::Rng;
use std::fmt;
use egui::Pos2;

// Define the parameters for station position optimization
#[derive(Clone)]
pub struct StationPositions {
    pub station_positions: Vec<Pos2>,
}

impl StationPositions {
    // Create StationPositions from optimization vector
    pub fn from_optimization_vector(
        x: &ArrayView2<f64>,
        n_stations: usize,
        domain: &SearchDomain,
    ) -> Self {
        let mut station_positions = Vec::with_capacity(n_stations);

        // -------------------------------------------------
        // Extract positions
        // -------------------------------------------------
        for i in 0..n_stations {

            let x_coord = x[[0, i * 2]] as f32;
            let y_coord = x[[0, i * 2 + 1]] as f32;

            // Clamp to the same search domain the optimizer is bounded to
            let x_clamped = x_coord.clamp(domain.min_x, domain.max_x);
            let y_clamped = y_coord.clamp(domain.min_y, domain.max_y);

            station_positions.push(
                round_to_centimeters(
                    Pos2::new(x_clamped, y_clamped)
                )
            );
        }

        Self {
            station_positions,
        }
    }

    // Generate a valid position: within the search domain and not inside any field
    fn generate_valid_position(
        domain: &SearchDomain,
        field_bounds: &[FieldBounds],
        rng: &mut impl Rng,
    ) -> Pos2 {
        let max_attempts = 100; // Prevent infinite loops

        // -------------------------------------------------
        // Try random valid positions
        // -------------------------------------------------
        for _ in 0..max_attempts {

            let x = rng.random_range(domain.min_x..domain.max_x);
            let y = rng.random_range(domain.min_y..domain.max_y);

            let candidate =
                round_to_centimeters(Pos2::new(x, y));

            if is_position_valid(candidate, field_bounds) {
                return candidate;
            }
        }

        // -------------------------------------------------
        // Fallback: warn and return the domain center
        // (may still be invalid if the domain is mostly occupied by fields)
        // -------------------------------------------------
        println!(
            "Warning: Could not find valid position after {} attempts, using domain center",
            max_attempts
        );

        round_to_centimeters(Pos2::new(
            (domain.min_x + domain.max_x) / 2.0,
            (domain.min_y + domain.max_y) / 2.0,
        ))
    }

    // Helper method to create station configs from optimized positions
    pub fn create_station_configs(&self, original_scene: &SceneConfig) -> Vec<StationConfig> {
        let mut station_configs = Vec::new();

        // Use original stations as template but with new positions
        for (i, position) in self.station_positions.iter().enumerate() {
            let original_station = if i < original_scene.station_configs.len() {
                &original_scene.station_configs[i]
            } else {
                &original_scene.station_configs[0] // Fallback to first station
            };

            // Create new station config with optimized position but keep other properties
            let mut station_config = original_station.clone();
            station_config.pose.position = *position;

            station_configs.push(station_config);
        }

        station_configs
    }

    // Generate initial population for EGO
    pub fn generate_initial_population(
        domain: &SearchDomain,
        field_bounds: &[FieldBounds],
        n_stations: usize,
        population_size: usize,
        rng: &mut impl Rng,
    ) -> Vec<StationPositions> {
        let mut population = Vec::with_capacity(population_size);

        for _ in 0..population_size {
            let mut station_positions = Vec::with_capacity(n_stations);

            for _ in 0..n_stations {
                let position = Self::generate_valid_position(domain, field_bounds, rng);
                station_positions.push(position);
            }

            population.push(StationPositions {
                station_positions,
            });
        }

        population
    }
}

// Display implementation for logging
impl fmt::Display for StationPositions {
    fn fmt(&self, f: &mut fmt::Formatter) -> fmt::Result {
        write!(f, "Stations: {} - Positions: [", self.station_positions.len())?;

        for (i, pos) in self.station_positions.iter().enumerate() {
            if i > 0 { write!(f, ", ")?; }
            write!(f, "({:.1},{:.1})", pos.x, pos.y)?;
        }

        write!(f, "]")
    }
}
