// Define field boundaries for optimization (adjust these based on your actual farm layout)
pub const FIELD_MIN_X: f32 = 0.0;
pub const FIELD_MAX_X: f32 = 25.0;  // width in meters
pub const FIELD_MIN_Y: f32 = 0.0;
pub const FIELD_MAX_Y: f32 = 25.0;  // height in meters

pub const STATION_MARGIN: f32 = 0.4; // Keep stations at least 0.4m from field edges
pub const OBSTACLE_MARGIN: f32 = 0.4; // Keep stations at least 0.4m from obstacles