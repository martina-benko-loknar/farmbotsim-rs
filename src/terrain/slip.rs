
/// Slip model (TODO: CHECK):
///
/// slip = K + K1 * slope + K2 * speed
///
/// Units:
/// - slope: radians
/// - speed (wheel speed): m/s
/// - slip: dimensionless (0.0 → 1.0)
///
/// Example: real_robot_speed = wheel_speed * (1.0 - slip)


use serde::Deserialize;
use std::fs;

#[derive(Debug, Deserialize)]
pub struct SlipConfig {
    pub fit: FitSection,
}

#[derive(Debug, Deserialize)]
pub struct FitSection {
    pub coefficients: Vec<f32>,
}

#[derive(Clone, Debug, PartialEq)]
pub struct SlipModel {
    pub k: f32,
    pub k1: f32,
    pub k2: f32,
}

impl SlipModel {
    /// Load slip parameters from JSON file
    pub fn from_json_file(path: &str) -> Self {
        let content = 
            fs::read_to_string(path)
            .unwrap_or_else(|e| panic!("Failed to read slip config: {e}"));

        let cfg: SlipConfig = 
            serde_json::from_str(&content)
            .expect("Invalid JSON");

        let coeffs = cfg.fit.coefficients;

        assert!(coeffs.len() == 3, "Slip model requires 3 coefficients");

        Self {
            k: coeffs[0],
            k1: coeffs[1],
            k2: coeffs[2],
        }
    }

    /// Compute slip factor
    pub fn compute_slip(&self, slope: f32, speed: f32) -> f32 {

        self.k + self.k1 * slope + self.k2 * speed
    }

    // Convert wheel speed into real robot speed
    pub fn compute_robot_speed(&self, wheel_speed: f32, slope: f32) -> f32 {

        let slip = self.compute_slip(slope, wheel_speed);

        wheel_speed * (1.0 - slip)
    }
}
