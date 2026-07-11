use egui::Vec2;

use crate::{
    agent_module::{agent::Agent, agent_config::AgentConfig}, battery_module::{battery::Battery, battery_config::BatteryConfig, charging::CcCvChargingModel, discharging::{VoltageDropLUT, physics_model::PhysicsDischargeModel}}, environment::{
        datetime::{DateTimeConfig, DateTimeManager},
        env_module::env_config::EnvConfig,
        field_config::FieldConfig,
        obstacle::Obstacle,
        scene_config::SceneConfig,
        spawn_area_module::spawn_area::SpawnArea,
        station_module::station::Station,
    }, experiment::config::ExperimentConfig, path_finding_module::visibility_graph::VisibilityGraph, task_module::task_manager::TaskManager, terrain::{TerrainLoader, slip::SlipModel}, units::{duration::Duration, energy::Energy, voltage::Voltage}, utilities::{
        pos2::random_pos2_in_rect,
        utils::{generate_colors, load_json_or_panic},
        vec2::random_vec2,
    }
};

/// -----------------------------
/// Experiment overrides 
/// -----------------------------
#[derive(Clone, Debug, Default)]
pub struct EnvOverrides {
    pub battery_capacity_wh: Option<f32>,
    pub battery_voltage_v: Option<f32>,
    pub critical_soc_percent: Option<f32>,
    pub threshold_soc_percent: Option<f32>,
}

/// Represents the environment of the simulation including agents, field, stations, obstacles,
/// and management of time and tasks.
#[derive(Debug, Clone)]
pub struct Env {
    /// Number of simulation steps performed.
    pub step_count: u32,
    /// Total duration elapsed.
    pub duration: Duration,
    /// Number of agents in the environment.
    pub n_agents: u32,
    /// File path to the agent configuration.
    pub agent_path: String,
    /// Collection of agents in the environment.
    pub agents: Vec<Agent>,
    /// Configuration of the field layout.
    pub field_config: FieldConfig,
    /// List of stations in the environment.
    pub stations: Vec<Station>,
    /// Spawn area for agent placement.
    pub spawn_area: SpawnArea,
    /// Obstacles present in the env.
    pub obstacles: Vec<Obstacle>,
    /// Visibility graph used for pathfinding.
    pub visibility_graph: VisibilityGraph,
    /// Configuration of the datetime system.
    pub datetime_config: DateTimeConfig,
    /// Manages date and time.
    pub date_time_manager: DateTimeManager,
    /// Manages tasks assigned to agents.
    pub task_manager: TaskManager,
}

impl Env {
    /// Public constructor (no experiment logic)
    /// Creates a new `Env` instance from a given `EnvConfig`.
    /// Panics if any JSON file can't be parsed or is not present.
    pub fn from_config(config: EnvConfig) -> Self {
        Self::build(config, None)
    }

    /// Experimental constructor with overrides
    pub fn from_config_with_overrides(
        config: EnvConfig,
        overrides: Option<&EnvOverrides>,
    ) -> Self {
        Self::build(config, overrides)
    }

    /// Shared internal builder
    fn build(config: EnvConfig, overrides: Option<&EnvOverrides>) -> Self {
        let scene_config: SceneConfig =
            load_json_or_panic(config.scene_config_path);

        let field_config: FieldConfig =
            load_json_or_panic(scene_config.field_config_path);

        let spawn_area =
            SpawnArea::from_config(scene_config.spawn_area_config.clone());

        let n_agents = config.n_agents;
        let agent_colors = generate_colors(n_agents as usize, 0.1);

        let agent_template: AgentConfig =
            load_json_or_panic(config.agent_config_path.clone());

        let mut agents = Vec::new();

        // --- heavy models  ---
        let terrain = TerrainLoader::from_gps_csv(
            "configs/scene_configs/vineyard_scene/baggy-altitude-empirical-lut.csv",
        );

        let slip_model = SlipModel::from_json_file(
            "configs/scene_configs/vineyard_scene/baggy-slip-linear.json",
        );

        let voltage_drop_lut = VoltageDropLUT::from_csv(
            "configs/movement_configs/consumption/fitted_lut.csv",
        );

        for i in 0..n_agents {
            let mut battery_config =
                BatteryConfig::from_json_file(agent_template.battery.clone());

            // -----------------------------
            // overrides (experiment layer)
            // -----------------------------
            if let Some(o) = overrides {
                if let Some(cap) = o.battery_capacity_wh {
                    battery_config.capacity = Energy::watt_hours(cap as f32);
                }
                if let Some(v) = o.battery_voltage_v {
                    battery_config.voltage = Voltage::volts(v as f32);
                }
            }

            // println!(
            //     "Agent {} battery capacity: {:.1}",
            //     i,
            //     battery_config.capacity
            // );

            // println!(
            //     "[ENV PTR {:p}] Agent {} battery = {:.1}",
            //     &agents as *const _,
            //     i,
            //     battery_config.capacity
            // );

            let charging_model =
                CcCvChargingModel::from_config(&battery_config);

            let discharging_model =
                PhysicsDischargeModel::new(
                    slip_model.clone(),
                    voltage_drop_lut.clone(),
                    terrain.clone(),
                );

            // let initial_soc = overrides
            //     .and_then(|o| o.initial_soc_percent)
            //     .unwrap_or(agent_template.battery_soc);

            let battery = Battery::from_config(
                battery_config,
                agent_template.battery_soc,
                charging_model,
                discharging_model,
            );

            agents.push(Agent::from_config(
                agent_template.clone(),
                i,
                random_pos2_in_rect(
                    egui::Rect {
                        min: spawn_area.left_top_pos,
                        max: spawn_area.left_top_pos
                            + Vec2::new(
                                spawn_area.width.to_base_unit(),
                                spawn_area.height.to_base_unit(),
                            ),
                    },
                    spawn_area.angle,
                ),
                random_vec2(),
                agent_colors[i as usize],
                battery,
            ));
        }

        let station_colors =
            generate_colors(scene_config.station_configs.len(), 0.0);

        let stations = scene_config
            .station_configs
            .iter()
            .enumerate()
            .map(|(i, cfg)| {
                Station::from_config(i as u32, station_colors[i], cfg.clone())
            })
            .collect();

        let obstacles = field_config.get_obstacles();

        let visibility_graph =
            VisibilityGraph::new(&field_config.get_graph_points(), obstacles.clone());

        let date_time_manager =
            DateTimeManager::from_config(config.datetime_config.clone());

        let default_exp_configs = ExperimentConfig::default();

        let critical_soc_percent = overrides
            .and_then(|o| o.critical_soc_percent)
            .unwrap_or(default_exp_configs.critical_soc_percent);

        let threshold_soc_percent = overrides
            .and_then(|o| o.threshold_soc_percent)
            .unwrap_or(default_exp_configs.soc_threshold_percent);

        let task_manager =
            TaskManager::from_config(
                config.task_manager_config, 
                field_config.clone(), 
                critical_soc_percent, 
                threshold_soc_percent,
            );

        Self {
            step_count: 0,
            duration: Duration::ZERO,
            n_agents,
            agent_path: config.agent_config_path,
            agents,
            field_config,
            stations,
            spawn_area,
            obstacles,
            visibility_graph,
            datetime_config: config.datetime_config,
            date_time_manager,
            task_manager,
        }
    }

    /// Reset environment (same logic as build, but reused pattern)
    pub fn reset(&mut self, overrides: Option<&EnvOverrides>) {
        self.agents.clear();

        let agent_colors = generate_colors(self.n_agents as usize, 0.1);

        let agent_template: AgentConfig =
            load_json_or_panic(self.agent_path.clone());

        let terrain = TerrainLoader::from_gps_csv(
            "configs/scene_configs/vineyard_scene/baggy-altitude-empirical-lut.csv",
        );

        let slip_model = SlipModel::from_json_file(
            "configs/scene_configs/vineyard_scene/baggy-slip-linear.json",
        );

        let voltage_drop_lut = VoltageDropLUT::from_csv(
            "configs/movement_configs/consumption/fitted_lut.csv",
        );

        for i in 0..self.n_agents {
            let mut battery_config =
                BatteryConfig::from_json_file(self.agent_path.clone());

            if let Some(o) = overrides {
                if let Some(cap) = o.battery_capacity_wh {
                    battery_config.capacity = Energy::watt_hours(cap as f32);
                }
                if let Some(v) = o.battery_voltage_v {
                    battery_config.voltage = Voltage::volts(v as f32);
                }
            }

            println!(
                "RESET Agent {} battery capacity: {:.1} Wh",
                i,
                battery_config.capacity
            );

            let charging_model =
                CcCvChargingModel::from_config(&battery_config);

            let discharging_model =
                PhysicsDischargeModel::new(
                    slip_model.clone(),
                    voltage_drop_lut.clone(),
                    terrain.clone(),
                );

            let battery = Battery::from_config(
                battery_config,
                agent_template.battery_soc,
                charging_model,
                discharging_model,
            );

            self.agents.push(Agent::from_config(
                agent_template.clone(),
                i,
                random_pos2_in_rect(
                    egui::Rect {
                        min: self.spawn_area.left_top_pos,
                        max: self.spawn_area.left_top_pos
                            + Vec2::new(
                                self.spawn_area.width.to_base_unit(),
                                self.spawn_area.height.to_base_unit(),
                            ),
                    },
                    self.spawn_area.angle,
                ),
                random_vec2(),
                agent_colors[i as usize],
                battery,
            ));
        }

        for station in &mut self.stations {
            station.reset();
        }

        self.date_time_manager.reset();
        self.task_manager.reset();
        self.step_count = 0;
    }

    pub fn reset_default(&mut self) {
        self.reset(None);
    }

    /// Step simulation
    pub fn step(&mut self) {
        let simulation_step = Duration::seconds(1.0);

        self.duration = self.duration + simulation_step;
        self.step_count += 1;

        self.date_time_manager
            .advance_time(simulation_step.to_base_unit() as i64);

        self.task_manager.update_waiting_list(simulation_step);

        for agent in &mut self.agents {
            agent.update(simulation_step, &self.date_time_manager);
        }

        // println!(
        //     "step={} | agent0 state={:?} energy={}",
        //     self.step_count,
        //     self.agents[0].state,
        //     self.agents[0].battery.energy
        // );
    }
}