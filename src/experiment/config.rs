use serde::Serialize;
use serde::Deserialize;

use crate::environment::field_config::FieldConfig;
use crate::environment::scene_config::SceneConfig;
use crate::utilities::utils::load_json_or_panic;
use crate::utilities::utils::load_scene_config;

#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct ExperimentConfig {
    pub seed: u64,

    pub battery_capacity_wh: f32,
    pub battery_voltage_v: f32,

    pub n_agents: u32,

    pub critical_soc_percent: f32,
    pub soc_threshold_percent: f32,

    pub field_resolution: usize,
    pub field_config_path: String,
}

impl ExperimentConfig {
    pub fn load_field_config(&self) -> FieldConfig {
        load_json_or_panic(self.field_config_path.clone())
    }

    pub fn load_scene_config(&self) -> SceneConfig {
        load_scene_config(self.field_config_path.clone())
    }
}

impl Default for ExperimentConfig {

    fn default() -> Self {

        Self {
            seed: 0,
            battery_capacity_wh: 73.2,
            battery_voltage_v: 10.8,
            n_agents: 1,
            critical_soc_percent: 45.0,
            soc_threshold_percent: 60.0,
            field_resolution: 20,
            field_config_path: "configs/field_configs/vineyard/xlarge.json".to_string(),
        }
    }
}