// Define field boundaries for optimization
pub const STATION_MARGIN: f32 = 0.4; // Keep stations at least 0.4m from field edges
pub const OBSTACLE_MARGIN: f32 = 0.4; // Keep stations at least 0.4m from obstacles

// SoC-threshold optimization (Level II) bounds: stay a safety margin above
// the hard critical-SoC floor, cap at a sane upper limit.
pub const SOC_THRESHOLD_MARGIN_ABOVE_CRITICAL: f32 = 5.0;
pub const SOC_THRESHOLD_MAX_PERCENT: f32 = 90.0;
pub const SOC_THRESHOLD_INITIAL_SAMPLES: usize = 6;