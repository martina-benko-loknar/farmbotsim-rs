use egui::Pos2;
use crate::cfg::{ 
    DEFAULT_AGENT_CONFIG_PATH,
};
use::std::time::Instant;

use crate::environment::{
    datetime::DateTimeConfig,
    env_module::env_config::EnvConfig,
    scene_config::SceneConfig,
};
use crate::task_module::strategies::{ChargingStrategy, ChooseStationStrategy};
use crate::tool_module::experiment_tool::{SingleEvaluation, TerminationCondition};
use crate::units::{energy::Energy, duration::Duration};
pub struct LayoutEvaluation {
    pub energy: f64,
    pub total_distance: f64,
    pub charging_distance: f64,
    pub runtime_sec: f64,
}

fn default_env_config(
    scene_config_path: String,
) -> EnvConfig {

    let mut env_config = EnvConfig::default();

    env_config.scene_config_path =
        scene_config_path;

    env_config.agent_config_path =
        DEFAULT_AGENT_CONFIG_PATH.to_string();

    env_config.datetime_config =
        DateTimeConfig::from_string(
            "01.01.2025 08:00:00".to_string()
        );

    env_config.n_agents = 1;

    env_config.task_manager_config.charging_strategy =
        ChargingStrategy::CriticalOnly;

    env_config.task_manager_config.choose_station_strategy =
        ChooseStationStrategy::ClosestManhattan;

    env_config
}

pub fn evaluate_station_layout(
    stations: &[Pos2],
    original_scene: &SceneConfig,
) -> LayoutEvaluation {

    let start = Instant::now();

    let mut modified_scene = original_scene.clone();

    modified_scene.station_configs.clear();

    for position in stations {

        let mut station_config =
            original_scene.station_configs[0].clone();

        station_config.pose.position = *position;

        modified_scene
            .station_configs
            .push(station_config);
    }

    for (station_cfg, position) in modified_scene
        .station_configs
        .iter_mut()
        .zip(stations.iter())
    {
        station_cfg.pose.position = *position;
    }

    let random_id: u32 = rand::random();

    let temp_scene_path = format!(
        "configs/scene_configs/temp_eval_{}.json",
        random_id
    );

    let serialized =
        serde_json::to_string(&modified_scene).unwrap();

    std::fs::write(
        &temp_scene_path,
        &serialized,
    )
    .unwrap();

    let env_config = 
        default_env_config(temp_scene_path.clone());


    let mut runner = SingleEvaluation {
        running: false,
        scene_config_path: temp_scene_path.clone(),
        agent_config_path:
            env_config.agent_config_path.clone(),
        datetime_config:
            env_config.datetime_config.clone(),
        env_config,
        termination_condition:
            TerminationCondition::NumberCompletedTasks(1000),
        env: None,
        save_to_file: false,
        save_file_name:
            format!("layout_eval_{}", random_id),
        start_datetime: None,
        start_time: None,
        total_energy_consumed:
            Energy::watt_hours(0.0),
        total_distance_driven: 0.0,
        total_charging_distance: 0.0,
        total_charging_approach_distance: 0.0,
        total_charging_departure_distance: 0.0,
        agents_departing_from_charging: Vec::new(),
        completed_stationary_tasks: 0,
        completed_moving_tasks: 0,
        agent_actions: Vec::new(),
        previous_agent_states: Vec::new(),
        previous_agent_positions: Vec::new(),
        step_start_time: Duration::ZERO,
    };

    runner.run_simulation();

    let _ = std::fs::remove_file(temp_scene_path);
    let runtime_sec = start.elapsed().as_secs_f64();

    LayoutEvaluation {
        energy:
            runner.total_energy_consumed.value as f64,

        total_distance:
            runner.total_distance_driven as f64,

        charging_distance:
            runner.total_charging_distance as f64,
        
        runtime_sec,
    }
}