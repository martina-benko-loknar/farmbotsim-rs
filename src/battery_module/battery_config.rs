use crate::{units::{duration::Duration, energy::Energy, power::Power, voltage::Voltage}, utilities::utils::load_json_or_panic};

/// Configuration for a battery model, including capacity, voltage, and charging parameters.
///
/// The CC-CV fields and the seasonal-solar fields are each optional so a given
/// battery config only needs to populate whichever charging model it uses.
#[derive(Debug, Clone, serde::Serialize, serde::Deserialize)]
pub struct BatteryConfig {
    pub name: String,
    pub capacity: Energy,
    pub voltage: Voltage,

    /// Constant-current charging power used below `cv_fraction` of capacity.
    #[serde(default)]
    pub cc_power: Option<Power>,
    /// Fraction of `capacity` at which charging switches from CC to CV.
    #[serde(default)]
    pub cv_fraction: Option<f32>,
    /// Time constant of the exponential charge-current taper during the CV phase.
    #[serde(default)]
    pub cv_time_constant: Option<Duration>,

    /// Seasonal-solar charge curve data (January maximum irradiance day).
    #[serde(default)]
    pub jan_max: Option<String>,
    /// Seasonal-solar charge curve data (January minimum irradiance day).
    #[serde(default)]
    pub jan_min: Option<String>,
    /// Seasonal-solar charge curve data (June maximum irradiance day).
    #[serde(default)]
    pub jun_max: Option<String>,
}
impl BatteryConfig {
    /// Loads a BatteryConfig from a config.json file inside the given folder.
    /// Panics if the file is missing or invalid.
    pub fn from_json_file(folder_name: String) -> Self {
        let path_str = format!("{}/config.json", folder_name);
        load_json_or_panic(path_str)
    }
}