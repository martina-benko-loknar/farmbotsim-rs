use egui::{Color32, Pos2, Vec2};

use crate::{
    agent_module::{
        agent_config::AgentConfig, agent_state::AgentState, work_schedule::WorkSchedule
    },
    battery_module::{battery::Battery, battery_config::BatteryConfig}, 
    cfg::{TOLERANCE_ANGLE, TOLERANCE_DISTANCE}, environment::datetime::DateTimeManager, movement_module::{is_movement::IsMovement, movement::{Movement, MovementInputs}, pose::Pose}, task_module::task::Task, units::{
        angle::Angle, angular_velocity::AngularVelocity, duration::Duration, linear_velocity::LinearVelocity
    }, utilities::pos2::ExtendedPos2
};

use crate::battery_module::discharging::physics_model::PhysicsDischargeModel;
use crate::battery_module::charging::seasonal_solar::SeasonalBatteryModel;
use crate::terrain::TerrainLoader;
use crate::terrain::slip::SlipModel;
use crate::battery_module::discharging::VoltageDropLUT;
use crate::units::energy::Energy;
use crate::battery_module::discharging::traits::DischargeModel;
/// Represents agent ID
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub struct AgentId(u32);
impl AgentId {
    pub fn new(id: u32) -> Self {
        AgentId(id)
    }
}
impl std::fmt::Display for AgentId {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(f, "{}", self.0)
    }
}

/// Represents a mobile agent in the simulation with movement, battery, and task execution capabilities.
#[derive(Clone, Debug, PartialEq)]
pub struct Agent {
    pub id: AgentId,
    pub pose: Pose,
    pub previous_pose: Pose,
    pub movement: Movement,
    pub velocity_lin: LinearVelocity,
    pub velocity_ang: AngularVelocity,
    pub color: Color32,
    pub spawn_position: Pos2,

    pub work_schedule: WorkSchedule,
    pub current_task: Option<Task>,
    pub completed_task_ids: Vec<u32>, // for storing so task manager can know

    pub state: AgentState,
    pub battery: Battery,
}

impl Agent {
    /// Constructs an [`Agent`] from an [`AgentConfig`], setting its initial state, pose, and battery.
    pub fn from_config(config: AgentConfig, id: u32, position: Pos2, direction: Vec2, color: Color32) -> Self {

        // Charging model
        let battery_config =
            BatteryConfig::from_json_file(
                config.battery,
            );
        let charging_model =
            SeasonalBatteryModel::from_config(&battery_config);

        // Discharging model
        let terrain =
            TerrainLoader::from_gps_csv(
                "configs/scene_configs/vineyard_scene/baggy-altitude-empirical-lut.csv",
            );
        let slip_model =
            SlipModel::from_json_file(
                "configs/scene_configs/vineyard_scene/baggy-slip-linear.json",
            );
        let voltage_drop_lut =
            VoltageDropLUT::from_csv(
                "configs/movement_configs/consumption/fitted_lut.csv",
            );
        let discharging_model =
            PhysicsDischargeModel::new(
                slip_model,
                voltage_drop_lut,
                terrain,
            );
        let pose= Pose::new(position, Angle::radians(direction.angle()));

        Self {
            id: AgentId(id),
            pose: pose.clone(),
            previous_pose:pose,
            movement: Movement::from_json_file(config.movement),
            velocity_lin: LinearVelocity::ZERO,
            velocity_ang: AngularVelocity::ZERO,
            color,
            spawn_position: position,

            work_schedule: WorkSchedule::default(),
            current_task: None,
            completed_task_ids: vec![],

            state: AgentState::Wait,
            battery: Battery::from_config(
                battery_config, 
                config.battery_soc,
            charging_model, 
            discharging_model),
        }
    }

    pub fn compute_energy_loss(
        &self,
        slope_rad: f32,
        dt: Duration,
    ) -> Energy {
        let wheel_speed = self.velocity_lin.to_base_unit();

        self.battery
            .discharging_model
            .compute_energy_loss(wheel_speed, slope_rad, dt)
    }

    /// Updates the agent's state, task, movement, and battery based on simulation time.
    pub fn update(&mut self, simulation_step: Duration, date_time_manager: &DateTimeManager) {
        if self.state == AgentState::Discharged { return }

        //Old model: state -> fixed power draw -> battery loss
        //New model: movement -> terrain slope -> slip -> real speed -> energy loss
        self.update_task_and_path(simulation_step);
        
        let inputs = self.get_inputs();
        self._move(simulation_step, inputs);

        self.update_state(simulation_step,date_time_manager);
    }

    /// Handles finite state machine logic and transitions.
    fn update_state(&mut self, simulation_step: Duration, date_time_manager: &DateTimeManager) {
        let mut current_state = std::mem::replace(&mut self.state, AgentState::Wait); // placeholder

        let maybe_new_state = current_state.update(simulation_step, self, date_time_manager);

        if let Some(mut new_state) = maybe_new_state {
            current_state.on_exit(self);
            new_state.on_enter(self);
            self.state = new_state;
        } else {
            self.state = current_state;
        }
    }
    
    /// Moves the agent by calculating new pose and velocities based on inputs.
    fn _move(&mut self, simulation_step: Duration, inputs: MovementInputs) {

        self.previous_pose = self.pose.clone();

        let current_task_velocity = 
            self.current_task
            .as_ref()
            .map(|task| task.get_velocity())
            .unwrap_or(LinearVelocity::ZERO);

        let (new_pose, new_velocity_l, new_velocity_a) =
            self.movement.calculate_new_pose_from_inputs(
            simulation_step, 
            inputs, 
            self.pose.clone(), 
            current_task_velocity
        );
  
        // Store new state
        self.pose = new_pose;
        self.velocity_lin = new_velocity_l;
        self.velocity_ang = new_velocity_a;

    }
    
    /// Computes movement inputs required to reach the next target in the current task.
    fn get_inputs(&self) -> MovementInputs {
        let next_pose = match &self.current_task {
            Some(task) => task.get_first_pose().unwrap_or(&self.pose),
            None => &self.pose,
        };
        self.movement.calculate_inputs_for_target(&self.pose, next_pose)
    }
    
    /// Updates the current task and its path based on agent's progress and pose.
    fn update_task_and_path(&mut self, simulation_step: Duration) {
        if let Some(task) = &mut self.current_task {
            match task {
                Task::Stationary { pose, duration, .. } => {
                    if self.pose.position.is_close_to(pose.position, TOLERANCE_DISTANCE) {
                        if *duration > Duration::ZERO {
                            *duration = *duration - simulation_step;
                        } else if let Some(id) = task.get_id() {
                            self.completed_task_ids.push(*id);
                            self.current_task = self.work_schedule.pop_front();
                        }
                    }
                }
                Task::WaitDuration { duration , ..} => {
                    if *duration > Duration::ZERO {
                        *duration = *duration - simulation_step;
                    } else {
                        self.current_task = self.work_schedule.pop_front();
                    }
                }
                Task::WaitInfinite { .. } => { }
                Task::Moving { path, .. } | Task::Travel { path, .. } => {
                    if let Some(front_pose) = path.front() {
                        if self.pose.is_close_to(front_pose, TOLERANCE_DISTANCE, TOLERANCE_ANGLE) {
                            path.pop_front();
                        }
                    }
                    if path.is_empty() {
                        if let Some(id) = task.get_id() {
                            self.completed_task_ids.push(*id);
                        }
                        self.current_task = self.work_schedule.pop_front();
                    }
                }
            }
        } else {
            self.current_task = self.work_schedule.pop_front();
        }
    }

}


