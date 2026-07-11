use crate::{
    agent_module::agent::Agent, battery_module::{is_battery::IsBattery}, cfg::{
        POWER_CONSUMPTION_WAIT
    }, environment::datetime::DateTimeManager, units::{
        duration::Duration,
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
    pub fn on_enter(&mut self, _agent: &mut Agent) {
        match self {
            AgentState::Wait => { },
            AgentState::Travel => { },
            AgentState::Work => { },
            AgentState::Charging => { },
            AgentState::Discharged => { },
        }
    }
    
    /// Updates the agent state based on current conditions, battery, and tasks.
    /// Returns Some(new_state) if a state transition should occur, otherwise None.
    pub fn update(
        &mut self, 
        simulation_step: Duration, 
        agent: &mut Agent, 
        _date_time_manager: &DateTimeManager
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
                //let slope_rad = 0.0; // TEMP until movement system provides slope

                let x1 = agent.previous_pose.position.x;
                let y1 = agent.previous_pose.position.y;

                let x2 = agent.pose.position.x;
                let y2 = agent.pose.position.y;

                let slope_rad = agent.battery
                    .discharging_model
                    .terrain
                    .slope_between(
                        x1 as f64, 
                        y1 as f64, 
                        x2 as f64, 
                        y2 as f64)
                    .unwrap_or(0.0);

                let energy_loss = agent.compute_energy_loss(
                    slope_rad as f32,
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
                    if task.is_work() {
                        Some(AgentState::Work) 
                    } else if task.is_wait() && task.is_charge_intent() {
                        Some(AgentState::Charging) 
                    } else if task.is_wait() {
                        Some(AgentState::Wait) 
                    } else {
                        None
                    }
                } else {
                    Some(AgentState::Wait) 
                }
            },
            AgentState::Work => {
                // discharge battery
                //let slope_rad = 0.0; // TEMP until movement system provides slope

                let x1 = agent.previous_pose.position.x;
                let y1 = agent.previous_pose.position.y;

                let x2 = agent.pose.position.x;
                let y2 = agent.pose.position.y;

                let slope_rad = agent.battery
                    .discharging_model
                    .terrain
                    .slope_between(
                        x1 as f64, 
                        y1 as f64, 
                        x2 as f64, 
                        y2 as f64)
                    .unwrap_or(0.0);
                
                let energy_loss = agent.compute_energy_loss(
                    slope_rad as f32,
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
                    if !task.is_work() {
                        Some(AgentState::Travel)
                    } else {
                        None
                    }
                } else {
                    Some(AgentState::Wait)
                }
            },
            AgentState::Charging => {
                // charge battery
                agent.battery.charge(simulation_step);
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

}
