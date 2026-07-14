pub mod cc_cv;
pub mod seasonal_solar;
pub use cc_cv::CcCvChargingModel;
pub use seasonal_solar::SeasonalBatteryModel;

use crate::battery_module::battery_config::BatteryConfig;
use crate::logger::log_error_and_panic;
use crate::units::{duration::Duration, energy::Energy};

/// Which charging model a `BatteryConfig` should build.
#[derive(Debug, Clone, Copy, PartialEq, serde::Serialize, serde::Deserialize)]
pub enum ChargingModelKind {
    CcCv,
    SeasonalSolar,
}

/// Wraps the concrete charging models so `Battery` can hold either behind one
/// type, picked at construction time via `BatteryConfig::charging_model`.
#[derive(Clone, Debug, PartialEq)]
pub enum ChargingModel {
    CcCv(CcCvChargingModel),
    SeasonalSolar(SeasonalBatteryModel),
}

impl ChargingModel {
    pub fn from_config(config: &BatteryConfig) -> Self {
        match config.charging_model {
            ChargingModelKind::CcCv =>
                Self::CcCv(CcCvChargingModel::from_config(config)),
            ChargingModelKind::SeasonalSolar =>
                Self::SeasonalSolar(SeasonalBatteryModel::from_config(config)),
        }
    }

    /// Advances charge by `duration`. `month` (1-12) is only used by the
    /// seasonal-solar model; the CC-CV model ignores it.
    pub fn compute_charge(&mut self, energy: Energy, duration: Duration, month: u32) -> Energy {
        match self {
            Self::CcCv(model) =>
                model.compute_charge(energy, duration),
            Self::SeasonalSolar(model) =>
                model.compute_charge(energy, duration, month)
                    .unwrap_or_else(|e| log_error_and_panic(
                        &format!("seasonal-solar charging failed: {e}")
                    )),
        }
    }
}