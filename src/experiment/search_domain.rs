use crate::environment::geometry::FieldBounds;

#[derive(Debug, Clone, Copy)]
pub struct SearchDomain {
    pub min_x: f32,
    pub max_x: f32,
    pub min_y: f32,
    pub max_y: f32,
}

impl SearchDomain {

    pub fn from_bounds(
        vineyard: FieldBounds,
        terrain: FieldBounds,
        vineyard_padding: f32,
        station_margin: f32,
    ) -> Self {

        // --------------------------------------------------
        // 1. Expand vineyard bounds
        // --------------------------------------------------

        let expanded_min_x = vineyard.min_x - vineyard_padding;
        let expanded_max_x = vineyard.max_x + vineyard_padding;

        let expanded_min_y = vineyard.min_y - vineyard_padding;
        let expanded_max_y = vineyard.max_y + vineyard_padding;

        // --------------------------------------------------
        // 2. Intersect with terrain bounds
        // --------------------------------------------------

        let intersected_min_x =
            expanded_min_x.max(terrain.min_x);

        let intersected_max_x =
            expanded_max_x.min(terrain.max_x);

        let intersected_min_y =
            expanded_min_y.max(terrain.min_y);

        let intersected_max_y =
            expanded_max_y.min(terrain.max_y);

        // --------------------------------------------------
        // 3. Shrink by station margin
        // --------------------------------------------------

        let min_x = intersected_min_x + station_margin;
        let max_x = intersected_max_x - station_margin;

        let min_y = intersected_min_y + station_margin;
        let max_y = intersected_max_y - station_margin;

        Self {
            min_x,
            max_x,
            min_y,
            max_y,
        }
    }
}