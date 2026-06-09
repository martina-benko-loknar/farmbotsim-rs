use std::path::Path;
use serde::{Deserialize, Serialize};

use crate::{
    cfg::{
        DEFAULT_AGENT_CONFIG_PATH, 
        DEFAULT_SCENE_CONFIG_PATH,
        DEFAULT_FIELD_CONFIG_PATH}, environment::datetime::DateTimeConfig, task_module::task_manager_config::TaskManagerConfig, utilities::utils::load_json_or_panic
};


/// Configuration settings for the environment.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct EnvConfig {
    /// Number of agents in the environment.
    pub n_agents: u32,
    /// Path to the agent configuration file.
    pub agent_config_path: String,
    /// Configuration for date and time settings.
    #[serde(rename = "date_time")]
    pub datetime_config: DateTimeConfig,
    /// Path to the scene configuration file.
    pub scene_config_path: String,
    /// Path to the field configuration file.
    pub field_config_path: String,
    /// Configuration for the task manager.
    #[serde(rename = "task_manager")]
    pub task_manager_config: TaskManagerConfig,
}

impl Default for EnvConfig {
    /// Creates a default environment configuration with preset values.
    fn default() -> Self {
        Self {
            n_agents: 1,
            agent_config_path: DEFAULT_AGENT_CONFIG_PATH.to_string(),
            datetime_config: DateTimeConfig::from_string("01.01.2025 00:00:00".to_string()),
            scene_config_path: DEFAULT_SCENE_CONFIG_PATH.to_string(),
            field_config_path: DEFAULT_FIELD_CONFIG_PATH.to_string(),
            task_manager_config: TaskManagerConfig::default(),
        }
    }
}

impl EnvConfig {
    /// Creates a new `EnvConfig` with specified parameters.
    pub fn new(
        n_agents: u32, 
        agent_config_path: String, 
        datetime_config: DateTimeConfig, 
        scene_config_path: String, 
        field_config_path: String, 
        task_manager_config: TaskManagerConfig
    ) -> Self {
        Self {
            n_agents,
            agent_config_path,
            datetime_config,
            scene_config_path,
            field_config_path,
            task_manager_config,
        }
    }
}

impl EnvConfig {
    /// Loads an `EnvConfig` from a JSON file at the given path.
    /// Panics if the JSON file cannot be loaded or parsed.
    pub fn from_json_file<P: AsRef<Path>>(file_path: P) -> Self {
        load_json_or_panic(file_path)
    }
}
