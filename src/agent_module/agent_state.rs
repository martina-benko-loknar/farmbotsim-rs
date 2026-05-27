use std::collections::HashMap;

use crate::{
    agent_module::agent::Agent, battery_module::{discharging::{traits::DischargeModel}, is_battery::IsBattery}, cfg::{
        POWER_CONSUMPTION_TRAVEL, POWER_CONSUMPTION_WAIT
    }, environment::datetime::DateTimeManager, task_module::task::Task, units::{
        duration::Duration,
        power::Power,
    }
};

use crate::units::energy::Energy;

/// Represents states an agent can be in during simulation.
#[derive(Clone, Debug, PartialEq)]
pub enum AgentState {
    Wait,
    Travel,
    Work,
    Charging,
    Discharged,
}

impl AgentState {
    /// Called when the agent enters this state.
    pub fn on_enter(&mut self, agent: &mut Agent) {
        match self {
            AgentState::Wait => { },
            AgentState::Travel => { },
            AgentState::Work => { },
            AgentState::Charging => {
                agent.battery.charging_model.start_index = HashMap::from([
                    ("jan".to_string(), 1),
                    ("jun".to_string(), 1),
                ]);
            },
            AgentState::Discharged => { },
        }
    }
    
    /// Updates the agent state based on current conditions, battery, and tasks.
    /// Returns Some(new_state) if a state transition should occur, otherwise None.
    pub fn update(
        &mut self, 
        simulation_step: Duration, 
        agent: &mut Agent, 
        date_time_manager: &DateTimeManager
    ) -> Option<AgentState> {
        match self {
            AgentState::Wait => {
                // discharge battery
                agent.battery.discharge(POWER_CONSUMPTION_WAIT, simulation_step);
                // check battery
                if let Some(discharge) = Self::check_battery(agent) { return Some(discharge); }
                // transitions
                if let Some(task) = &agent.current_task {
                    if task.is_wait() { None }
                    else if task.is_travel() { Some(AgentState::Travel) }
                    else { None }
                }
                else { None }
            },
            AgentState::Travel => {
                // discharge battery
                    // let power = Self::calculate_power_travel(agent);
                    // agent.battery.discharge(power, simulation_step);      

                // TODO: TEMP: flat terrain
                let slope_rad = 0.0;

                // wheel speed from movement system
                let wheel_speed =
                    agent.velocity_lin.to_base_unit();

                // physics-based discharge
                let energy_loss = agent.battery
                    .discharging_model
                    .compute_energy_loss(
                        wheel_speed,
                        slope_rad,
                        simulation_step,
                    );  

                agent.battery.energy = agent.battery.energy - energy_loss;

                if agent.battery.energy < Energy::ZERO {
                    agent.battery.energy = Energy::ZERO;
                }

                agent.battery.soc =
                    (agent.battery.energy / agent.battery.capacity)
                        * 100.0;    

                // check battery
                if let Some(discharge) = Self::check_battery(agent) { return Some(discharge); }
                // transitions
                if let Some(task) = &agent.current_task {
                    if task.is_work() { Some(AgentState::Work) }
                    else if task.is_wait() && task.is_charge_intent() { Some(AgentState::Charging) }
                    else if task.is_wait() { Some(AgentState::Wait) }
                    else { None }
                }
                else { Some(AgentState::Wait) }
            },
            AgentState::Work => {
                // discharge battery
                    // let power = match &agent.current_task {
                    //     Some(task) => {
                    //         Self::calculate_power_work(agent, task)
                    //     },
                    //     None => {
                    //         Power::ZERO // when task is complete and removed but state is still Work instead of other
                    //     }
                    // };
                    // agent.battery.discharge(power, simulation_step);

                // TODO: TEMP: flat terrain
                let slope_rad = 0.0;

                let wheel_speed =
                    agent.velocity_lin.to_base_unit();

                let energy_loss = agent.battery
                    .discharging_model
                    .compute_energy_loss(
                        wheel_speed,
                        slope_rad,
                        simulation_step,
                    );

                agent.battery.energy = agent.battery.energy - energy_loss;

                if agent.battery.energy < Energy::ZERO {
                    agent.battery.energy = Energy::ZERO;
                }

                agent.battery.soc =
                    (agent.battery.energy / agent.battery.capacity)
                        * 100.0;

                // check battery
                if let Some(discharge) = Self::check_battery(agent) { return Some(discharge); }
                // transitions
                if let Some(task) = &agent.current_task {
                    if !task.is_work() { Some(AgentState::Travel) }
                    else { None }
                }
                else { Some(AgentState::Wait) }
            },
            AgentState::Charging => {
                // charge battery
                agent.battery.charge(simulation_step, date_time_manager.get_month());
                // transitions
                if let Some(task) = &agent.current_task {
                    if !task.is_wait() && !task.is_charge_intent() { Some(AgentState::Travel) }
                    else { None }
                }
                else { None }
            },
            AgentState::Discharged => {
                None
            },
        }
    }

    /// Called when exiting this state.
    pub fn on_exit(&mut self, _agent: &mut Agent) {
        match self {
            AgentState::Wait => { },
            AgentState::Travel => { },
            AgentState::Work => { },
            AgentState::Charging => { },
            AgentState::Discharged => { },
        }
    }


    // Checks if the battery is depleted, triggering Discharged state
    fn check_battery(agent: &Agent) -> Option<AgentState> {
        if agent.battery.get_soc() <= 0.0 {
            return Some(AgentState::Discharged);
        }
        None
    }
    // Power consumption when traveling, scaled by velocity ratio
    fn calculate_power_travel(agent: &Agent) -> Power {
        POWER_CONSUMPTION_TRAVEL * (agent.velocity_lin / agent.movement.max_velocity())
    }
    // Power consumption while working, depends on task type
    fn calculate_power_work(agent: &Agent, task: &Task) -> Power {
        match task {
            Task::Stationary { power, .. } => *power,
            Task::Moving { power, .. } => *power + Self::calculate_power_travel(agent),
            _ => Power::ZERO
        }
    }
}
