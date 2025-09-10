use std::io::Write;
use serde::{Deserialize, Serialize};
use chrono::{DateTime, Local};

use crate::{
    cfg::{EXPERIMENTS_PATH},
    environment::{
        datetime::DateTimeConfig,
        env_module::{env::Env, env_config::EnvConfig},
        field_config::FieldConfig,
        scene_config::SceneConfig,
    },
    logger::log_error_and_panic,
    units::duration::{format_duration, Duration},
    units::energy::Energy,
    utilities::utils::load_json_or_panic,
};

#[derive(Debug, Clone, PartialEq)]
pub enum TerminationCondition {
    AllTasksCompleted,
    NumberCompletedTasks(u32),
    EnvDuration(Duration),
}

#[derive(Debug, Serialize, Deserialize)]
struct SimulationSummary {
    start_datetime: DateTime<Local>,
    evaluation_duration: std::time::Duration,
    scene_config_path: String,
    result: EpisodeResult,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct AgentAction {
    timestamp: String,
    agent_id: u32,
    action_type: String,
    from_position: (f32, f32),
    to_position: (f32, f32),
    duration_seconds: f32,
    energy_consumed: f32,
    task_details: Option<String>,
}

#[derive(Debug, Serialize, Deserialize)]
struct EpisodeResult {
    env_config: EnvConfig,
    n_completed_tasks: u32,
    env_duration: String,
    total_energy_consumed: String,
    total_distance_driven: String,
    total_charging_distance: String,
    total_charging_approach_distance: String,
    total_charging_departure_distance: String,
    completed_stationary_tasks: u32,
    completed_moving_tasks: u32,
    agent_actions: Vec<AgentAction>,
}

pub struct ExperimentRunner {
    pub running: bool,
    pub scene_config_path: String,
    pub agent_config_path: String,
    pub datetime_config: DateTimeConfig,
    pub env_config: EnvConfig,
    pub termination_condition: TerminationCondition,
    pub env: Option<Env>,
    pub save_to_file: bool,
    pub save_file_name: String,
    pub start_datetime: Option<DateTime<Local>>,
    pub start_time: Option<std::time::Instant>,
    pub total_energy_consumed: Energy,
    pub total_distance_driven: f32,
    pub total_charging_distance: f32,
    pub total_charging_approach_distance: f32,
    pub total_charging_departure_distance: f32,
    pub agents_departing_from_charging: Vec<bool>, // Track agents in their first task after charging
    pub completed_stationary_tasks: u32,
    pub completed_moving_tasks: u32,
    pub agent_actions: Vec<AgentAction>,
    pub previous_agent_states: Vec<crate::agent_module::agent_state::AgentState>,
    pub previous_agent_positions: Vec<egui::Pos2>,
    pub step_start_time: Duration,
}

impl ExperimentRunner {
    /// Runs the simulation for a single episode and saves the result.
    pub fn run_simulation(&mut self) {
        self.running = true;
        self.start_datetime = Some(Local::now());
        self.start_time = Some(std::time::Instant::now());
        self.total_energy_consumed = Energy::watt_hours(0.0);
        self.total_distance_driven = 0.0;
        self.total_charging_distance = 0.0;
        self.total_charging_approach_distance = 0.0;
        self.total_charging_departure_distance = 0.0;
        self.env = Some(Env::from_config(self.env_config.clone()));
        
        // Initialize tracking for agents departing from charging
        let n_agents = self.env_config.n_agents as usize;
        self.agents_departing_from_charging = vec![false; n_agents];

        // Initialize task type counters
        self.completed_stationary_tasks = 0;
        self.completed_moving_tasks = 0;

        // Initialize action tracking
        self.agent_actions = Vec::new();
        let n_agents = self.env_config.n_agents as usize;
        self.previous_agent_states = vec![crate::agent_module::agent_state::AgentState::Wait; n_agents];
        self.previous_agent_positions = vec![egui::Pos2::ZERO; n_agents];
        self.step_start_time = Duration::ZERO;

        while self.running {
            if let Some(env) = &mut self.env {

                // Track completed tasks count before the step
                let completed_tasks_count_before = env.task_manager.completed_tasks.len();
   
                env.task_manager.assign_tasks(&mut env.agents, &mut env.stations);

                // Track energy levels before the step
                let energy_before: Vec<Energy> = env.agents.iter()
                    .map(|agent| agent.battery.energy)
                    .collect();

                // Track positions before the step
                let positions_before: Vec<egui::Pos2> = env.agents.iter()
                    .map(|agent| agent.pose.position)
                    .collect();

                // Track charging task status before the step
                let is_charging_task_before: Vec<bool> = env.agents.iter()
                    .map(|agent| {
                        if let Some(ref task) = agent.current_task {
                            *task.get_intent() == crate::task_module::task::Intent::Charge
                        } else {
                            false
                        }
                    })
                    .collect();

                // Track agent states before the step (to detect charging departure)
                let agent_states_before: Vec<crate::agent_module::agent_state::AgentState> = env.agents.iter()
                    .map(|agent| agent.state.clone())
                    .collect();


                env.step(); // Perform the simulation step
                
                // Track agent actions
                for (i, agent) in env.agents.iter().enumerate() {
                    if i < self.previous_agent_states.len() && i < self.previous_agent_positions.len() {
                        let current_state = &agent.state;
                        let current_position = agent.pose.position;
                        let previous_state = &self.previous_agent_states[i];
                        let previous_position = self.previous_agent_positions[i];
                        
                        // Check if agent changed state or moved significantly
                        let moved = (current_position.x - previous_position.x).abs() > 0.1 || 
                                   (current_position.y - previous_position.y).abs() > 0.1;
                        let state_changed = current_state != previous_state;
                        
                        if moved || state_changed {
                            let action_type = match current_state {
                                crate::agent_module::agent_state::AgentState::Travel => "travel",
                                crate::agent_module::agent_state::AgentState::Work => {
                                    if let Some(ref task) = agent.current_task {
                                        match task {
                                            crate::task_module::task::Task::Stationary { .. } => "work_stationary",
                                            crate::task_module::task::Task::Moving { .. } => "work_moving",
                                            _ => "work_other"
                                        }
                                    } else {
                                        "work_unknown"
                                    }
                                },
                                crate::agent_module::agent_state::AgentState::Charging => "charging",
                                crate::agent_module::agent_state::AgentState::Wait => "waiting",
                                crate::agent_module::agent_state::AgentState::Discharged => "discharged",
                            };
                            
                            let task_details = if let Some(ref task) = agent.current_task {
                                Some(format!("Intent: {:?}", task.get_intent()))
                            } else {
                                None
                            };
                            
                            let action = AgentAction {
                                timestamp: format!("{}s", env.duration.value),
                                agent_id: format!("{}", agent.id).parse().unwrap_or(0),
                                action_type: action_type.to_string(),
                                from_position: (previous_position.x, previous_position.y),
                                to_position: (current_position.x, current_position.y),
                                duration_seconds: 1.0, // Assume 1 second time step for now
                                energy_consumed: 0.0, // We could calculate this if needed
                                task_details,
                            };
                            
                            self.agent_actions.push(action);
                        }
                        
                        // Update previous state tracking
                        self.previous_agent_states[i] = current_state.clone();
                        self.previous_agent_positions[i] = current_position;
                    }
                }

                // Track energy levels after the step
                let energy_after: Vec<Energy> = env.agents.iter()
                    .map(|agent| agent.battery.energy)
                    .collect();

                // Track positions after the step
                let positions_after: Vec<egui::Pos2> = env.agents.iter()
                    .map(|agent| agent.pose.position)
                    .collect();

                // Track agent states after the step
                let agent_states_after: Vec<crate::agent_module::agent_state::AgentState> = env.agents.iter()
                    .map(|agent| agent.state.clone())
                    .collect();
                
                // Get agent tasks after step for departure tracking
                let agent_tasks_after: Vec<Option<bool>> = env.agents.iter()
                    .map(|agent| agent.current_task.as_ref().map(|task| task.is_travel()))
                    .collect();

                // Track completed tasks by task type (only actual work tasks make it to completed_tasks)
                let completed_tasks_count_after = env.task_manager.completed_tasks.len();
                
                // Count work tasks that were officially completed this step
                if completed_tasks_count_after > completed_tasks_count_before {
                    let new_completed_tasks = &env.task_manager.completed_tasks[completed_tasks_count_before..];
                    for task in new_completed_tasks {
                        match task {
                            crate::task_module::task::Task::Stationary { .. } => self.completed_stationary_tasks += 1,
                            crate::task_module::task::Task::Moving { .. } => self.completed_moving_tasks += 1,
                            crate::task_module::task::Task::Travel { .. } | 
                            crate::task_module::task::Task::WaitDuration { .. } | 
                            crate::task_module::task::Task::WaitInfinite { .. } => {}
                        }
                    }
                }

                // Calculate consumed and charged energy
                for (before, after) in energy_before.iter().zip(energy_after.iter()) {
                    if after < before {
                        // Energy was consumed
                        self.total_energy_consumed = self.total_energy_consumed + (*before - *after);
                    }
                }

                // Calculate distance driven
                for (i, (pos_before, pos_after)) in positions_before.iter().zip(positions_after.iter()).enumerate() {
                    let distance = ((pos_after.x - pos_before.x).powi(2) + (pos_after.y - pos_before.y).powi(2)).sqrt();
                    self.total_distance_driven += distance;
                    
                    // Check if this was a charging approach (going to charge)
                    if let Some(&is_charging) = is_charging_task_before.get(i) {
                        if is_charging {
                            self.total_charging_approach_distance += distance;
                        }
                    }
                    
                    // Detect agents that just finished charging and mark them for departure tracking
                    if let (Some(state_before), Some(state_after)) = (agent_states_before.get(i), agent_states_after.get(i)) {
                        if *state_before == crate::agent_module::agent_state::AgentState::Charging && 
                           *state_after != crate::agent_module::agent_state::AgentState::Charging {
                            // Agent just finished charging, start tracking departure distance
                            if i < self.agents_departing_from_charging.len() {
                                self.agents_departing_from_charging[i] = true;
                            }
                        }
                    }
                    
                    // Track departure distance for agents that are departing from charging
                    if i < self.agents_departing_from_charging.len() && self.agents_departing_from_charging[i] {
                        self.total_charging_departure_distance += distance;
                        
                        // Check if agent has reached their destination (stop tracking departure)
                        if let Some(Some(is_travel_task)) = agent_tasks_after.get(i) {
                            // Stop tracking when agent completes their travel task
                            if !is_travel_task {
                                self.agents_departing_from_charging[i] = false;
                            }
                        } else {
                            // No task or invalid index, stop tracking
                            self.agents_departing_from_charging[i] = false;
                        }
                    }
                }
                
                // Update total charging distance (sum of approach and departure)
                self.total_charging_distance = self.total_charging_approach_distance + self.total_charging_departure_distance;

                let (finished, n_completed_tasks, env_duration) = match self.termination_condition {
                    TerminationCondition::AllTasksCompleted => {
                        let scene_config: SceneConfig = load_json_or_panic(self.scene_config_path.clone());
                        let field_config: FieldConfig = load_json_or_panic(scene_config.field_config_path);
                        if let Some(n_actions) = field_config.number_of_actions() {
                            if env.task_manager.completed_tasks.len() as u32 == n_actions {
                                (true, env.task_manager.completed_tasks.len() as u32, env.duration)
                            } else {
                                (false, 0, Duration::ZERO)
                            }
                        } else {
                            (false, 0, Duration::ZERO)
                        }
                    },
                    TerminationCondition::EnvDuration(duration) => {
                        if env.duration >= duration {
                            (true, env.task_manager.completed_tasks.len() as u32, env.duration)
                        } else {
                            (false, 0, Duration::ZERO)
                        }
                    },
                    TerminationCondition::NumberCompletedTasks(n_tasks) => {
                        if env.task_manager.completed_tasks.len() as u32 == n_tasks {
                            (true, env.task_manager.completed_tasks.len() as u32, env.duration)
                        } else {
                            (false, 0, Duration::ZERO)
                        }
                    }
                };

                if finished {
                    self.save_result(n_completed_tasks, env_duration);
                    self.running = false;
                }
            } else {
                self.running = false;
            }
        }
    }

    /// Saves the result of the single episode to a file.
    fn save_result(&self, n_completed_tasks: u32, env_duration: Duration) {
        if !self.save_to_file {
            return;
        }
        let evaluation_duration = self.start_time
            .map(|start| start.elapsed())
            .unwrap_or_default();
        let start_datetime: DateTime<Local> = self.start_datetime.unwrap_or_else(Local::now);
        let episode_result = EpisodeResult {
            env_config: self.env_config.clone(),
            n_completed_tasks,
            env_duration: format_duration(&env_duration),
            total_energy_consumed: format!("{:.2} {}", 
                self.total_energy_consumed.value, 
                self.total_energy_consumed.unit.as_str()),
            total_distance_driven: format!("{:.2} m", self.total_distance_driven),
            total_charging_distance: format!("{:.2} m", self.total_charging_distance),
            total_charging_approach_distance: format!("{:.2} m", self.total_charging_approach_distance),
            total_charging_departure_distance: format!("{:.2} m", self.total_charging_departure_distance),
            completed_stationary_tasks: self.completed_stationary_tasks,
            completed_moving_tasks: self.completed_moving_tasks,
            agent_actions: self.agent_actions.clone(),
        };
        let summary = SimulationSummary {
            start_datetime,
            evaluation_duration,
            scene_config_path: self.scene_config_path.clone(),
            result: episode_result,
        };
        let json = serde_json::to_string_pretty(&summary).unwrap_or_else(|e| {
            let msg = format!("Failed to serialize summary {:?}: {}", summary, e);
            log_error_and_panic(&msg);
        });
        let path = format!("{}{}.json", EXPERIMENTS_PATH, self.save_file_name);
        let mut file = std::fs::File::create(path.clone()).unwrap_or_else(|e| {
            let msg = format!("Failed to open file {:?}: {}", path, e);
            log_error_and_panic(&msg);
        });
        file.write_all(json.as_bytes()).unwrap_or_else(|e| {
            let msg = format!("Failed to write {:?}: {}", json, e);
            log_error_and_panic(&msg);
        });
    }
}