use serde::Deserialize;
use std::fs;

/// Piecewise voltage-drop-per-meter model as a function of terrain slope,
/// fit from real measurements taken at a single reference speed. Both
/// regions are real measurements (not placeholders): for slope >= 0 deg the
/// model is linear (`slope_coeff * x + intercept_v_per_m`); for slope < 0
/// deg it is flat, at the same value as the linear model's constant term
/// (`intercept_v_per_m`) -- i.e. the measured curve is continuous at 0 deg.
/// Travel and work are measured (and modeled) separately -- see
/// `SlopeConsumptionConfig`.
#[derive(Clone, Copy, Debug, PartialEq, Deserialize)]
pub struct SlopeConsumptionModel {
    /// V/m at 0 deg slope, and the flat value used for all slope < 0 deg.
    pub intercept_v_per_m: f32,
    /// dV/m per degree of slope, for slope_deg >= 0.
    pub slope_coeff: f32,
    /// Speed (m/s) this curve was measured at. Informational only for now
    /// -- the model does not yet correct for the agent's actual speed
    /// differing from this reference.
    pub reference_speed_mps: f32,
    /// Measured slope domain in degrees; inputs are clamped to
    /// +/- this value before evaluating the curve.
    pub max_abs_slope_deg: f32,
}

impl SlopeConsumptionModel {
    pub fn voltage_drop_per_m(&self, slope_rad: f32) -> f32 {
        let slope_deg = slope_rad
            .to_degrees()
            .clamp(-self.max_abs_slope_deg, self.max_abs_slope_deg);

        if slope_deg >= 0.0 {
            self.slope_coeff * slope_deg + self.intercept_v_per_m
        } else {
            self.intercept_v_per_m
        }
    }
}

/// Loads the paired travel/work consumption curves from JSON.
#[derive(Clone, Debug, Deserialize)]
pub struct SlopeConsumptionConfig {
    pub travel: SlopeConsumptionModel,
    pub work: SlopeConsumptionModel,
}

impl SlopeConsumptionConfig {
    pub fn from_json_file(path: &str) -> Self {
        let content = fs::read_to_string(path)
            .unwrap_or_else(|e| panic!("Failed to read slope consumption config: {e}"));

        serde_json::from_str(&content)
            .unwrap_or_else(|e| panic!("Invalid slope consumption config JSON: {e}"))
    }
}
