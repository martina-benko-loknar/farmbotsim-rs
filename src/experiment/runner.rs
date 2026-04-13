use crate::cfg::{
    DEFAULT_SCENE_CONFIG_PATH, DEFAULT_AGENT_CONFIG_PATH,
};
use crate::tool_module::experiment_tool::{ExperimentRunner, TerminationCondition};
use crate::environment::{
    datetime::DateTimeConfig,
    env_module::env_config::EnvConfig
};
use crate::task_module::strategies::{ChargingStrategy, ChooseStationStrategy};
use crate::units::{energy::Energy, duration::Duration};

pub fn run_experiment() {
    println!("Starting farmbot simulation experiment...");
    
    // Configure environment settings (similar to PerformanceMatrixTool)
    let mut env_config = EnvConfig::default();
    env_config.n_agents = 4;  // Number of agents
    
    // Configure task manager strategies (this is how PerformanceMatrixTool does it)
    env_config.task_manager_config.charging_strategy = ChargingStrategy::CriticalOnly;
    env_config.task_manager_config.choose_station_strategy = ChooseStationStrategy::ClosestMinQueueManhattan;
    
    // Set scene and agent config paths
    env_config.scene_config_path = DEFAULT_SCENE_CONFIG_PATH.to_string();
    env_config.agent_config_path = DEFAULT_AGENT_CONFIG_PATH.to_string();
    
    // Set datetime config
    env_config.datetime_config = DateTimeConfig::from_string("01.01.2025 08:00:00".to_string());
    
    // Create experiment runner
    let mut runner = ExperimentRunner {
        running: false,
        scene_config_path: env_config.scene_config_path.clone(),
        agent_config_path: env_config.agent_config_path.clone(),
        datetime_config: env_config.datetime_config.clone(),
        env_config,
        termination_condition: TerminationCondition::NumberCompletedTasks(1000), // Options: AllTasksCompleted, EnvDuration(Duration::days(1.0)), NumberCompletedTasks(3)
        env: None,
        save_to_file: true,
        save_file_name: "my_experiment_2025".to_string(),
        start_datetime: None,
        start_time: None,
        total_energy_consumed: Energy::watt_hours(0.0),
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
    
    println!("Configuration:");
    println!("- Agents: {}", runner.env_config.n_agents);
    println!("- Charging Strategy: {:?}", runner.env_config.task_manager_config.charging_strategy);
    println!("- Station Strategy: {:?}", runner.env_config.task_manager_config.choose_station_strategy);
    println!("- Termination: {:?}", runner.termination_condition);
    println!("- Scene: {}", runner.scene_config_path);
    
    // Run the experiment
    let start = std::time::Instant::now();
    runner.run_simulation();
    let elapsed = start.elapsed();
    
    println!("Experiment completed in {:?}", elapsed);
    println!("Results saved to: {}{}.json", 
        crate::cfg::EXPERIMENTS_PATH, 
        runner.save_file_name);
}