use serde::Serialize;
use serde::Deserialize;

use crate::environment::field_config::FieldConfig;
use crate::environment::scene_config::SceneConfig;
use crate::experiment::profile::ExperimentProfile;
use crate::utilities::utils::load_json_or_panic;
use crate::utilities::utils::load_scene_config;

fn default_n_station_slots() -> u32 { 1 }

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

    /// Charging slots at the (single, EGO-placed) station. Defaults to 1
    /// (the historical behavior -- see StationConfig::default). Set above 1
    /// to test whether fleet-size energy overhead is a queueing-contention
    /// artifact of a single shared slot rather than a travel-distance
    /// effect -- see fleet_slots_sweep.
    #[serde(default = "default_n_station_slots")]
    pub n_station_slots: u32,

    pub critical_soc_percent: f32,
    pub soc_threshold_percent: f32,

    /// When both are set, each agent's initial SoC is drawn uniformly from
    /// `[initial_soc_min_percent, initial_soc_max_percent]` (seeded off
    /// `seed`, on its own RNG stream — see `EnvOverrides` /
    /// `initial_soc_percents` in `env.rs`). `None` (the default) keeps the
    /// old deterministic behavior: every agent starts at the agent
    /// template's fixed `battery_soc`.
    pub initial_soc_min_percent: Option<f32>,
    pub initial_soc_max_percent: Option<f32>,
    /// Decouples the initial-SoC draw from `seed`. Leave `None` to piggyback
    /// off `seed` (the default -- fine for just toggling SoC randomization
    /// on). Set explicitly to vary this axis while holding `seed` (spawn
    /// position/heading) fixed, or vice versa.
    pub initial_soc_seed: Option<u64>,

    /// When both are set, each work task's duration is jittered by an
    /// independent uniform draw from
    /// `[task_duration_jitter_min_factor, task_duration_jitter_max_factor]`
    /// (seeded off `seed` by default, on its own RNG stream — see
    /// `EnvOverrides` / `TaskDurationJitter` in `env.rs`/`task_manager.rs`).
    /// `None` (the default) keeps every task at its configured duration.
    pub task_duration_jitter_min_factor: Option<f32>,
    pub task_duration_jitter_max_factor: Option<f32>,
    /// Decouples the task-duration draw from `seed`, same "unset =
    /// piggyback, set = decouple" pattern as `initial_soc_seed`.
    pub task_duration_jitter_seed: Option<u64>,

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
            n_station_slots: default_n_station_slots(),
            critical_soc_percent: 45.0,
            soc_threshold_percent: 60.0,
            initial_soc_min_percent: None,
            initial_soc_max_percent: None,
            initial_soc_seed: None,
            task_duration_jitter_min_factor: None,
            task_duration_jitter_max_factor: None,
            task_duration_jitter_seed: None,
            field_resolution: 50,
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