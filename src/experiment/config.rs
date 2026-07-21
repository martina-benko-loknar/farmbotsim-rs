use serde::Serialize;
use serde::Deserialize;

use crate::environment::field_config::FieldConfig;
use crate::environment::scene_config::SceneConfig;
use crate::experiment::profile::ExperimentProfile;
use crate::utilities::utils::load_json_or_panic;
use crate::utilities::utils::load_scene_config;

#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct ExperimentConfig {
    pub seed: u64,

    /// Agent config selecting the battery (capacity/voltage and
    /// charging/discharging model) this experiment runs with. Set via
    /// `ExperimentProfile::agent_config_path`.
    pub agent_config_path: String,

    pub battery_capacity_wh: f32,
    pub battery_voltage_v: f32,

    pub n_agents: u32,

    pub critical_soc_percent: f32,
    pub soc_threshold_percent: f32,

    pub field_resolution: usize,
    pub field_config_path: String,

    /// Number of completed tasks a single evaluation runs until. Set via
    /// `ExperimentProfile::n_tasks_target` so slower/lower-capacity robots
    /// aren't held to the same task count as the legacy robot.
    pub n_tasks_target: u32,
}

impl ExperimentConfig {
    pub fn load_field_config(&self) -> FieldConfig {
        load_json_or_panic(self.field_config_path.clone())
    }

    pub fn load_scene_config(&self) -> SceneConfig {
        load_scene_config(self.field_config_path.clone())
    }

    /// Short label describing the field size, derived from field_config_path
    /// so filenames can't drift out of sync with the config actually in use.
    pub fn field_size_label(&self) -> String {
        match self.field_config_path.as_str() {
            "configs/field_configs/vineyard/small.json" => "S".to_string(),
            "configs/field_configs/vineyard/medium.json" => "M".to_string(),
            "configs/field_configs/vineyard/large.json" => "L".to_string(),
            "configs/field_configs/vineyard/xlarge.json" => "XL".to_string(),
            other => std::path::Path::new(other)
                .file_stem()
                .and_then(|stem| stem.to_str())
                .unwrap_or(other)
                .to_string(),
        }
    }
}

impl ExperimentConfig {
    /// Baseline config for a given experiment profile: agent/battery config,
    /// field config, and battery capacity/voltage all come from `profile`,
    /// so the field-set and the battery model can never drift out of sync.
    pub fn for_profile(profile: ExperimentProfile) -> Self {
        Self {
            seed: 0,
            agent_config_path: profile.agent_config_path().to_string(),
            battery_capacity_wh: profile.battery_capacity_wh(),
            battery_voltage_v: profile.battery_voltage_v(),
            n_agents: 1,
            critical_soc_percent: 45.0,
            soc_threshold_percent: 60.0,
            field_resolution: 35,
            field_config_path: profile.default_field_config().to_string(),
            n_tasks_target: profile.n_tasks_target(),
        }
    }
}

impl Default for ExperimentConfig {
    fn default() -> Self {
        Self::for_profile(ExperimentProfile::Vineyard)
    }
}