use egui::Vec2;
use rand::Rng;
use rand::SeedableRng;
use rand::rngs::StdRng;

use crate::{
    agent_module::{agent::Agent, agent_config::AgentConfig}, battery_module::{battery::Battery, battery_config::BatteryConfig, charging::ChargingModel, discharging::{DischargeModelKind, DischargingModel, SlopeConsumptionConfig, physics_model::PhysicsDischargeModel}}, environment::{
        datetime::{DateTimeConfig, DateTimeManager},
        env_module::env_config::EnvConfig,
        field_config::FieldConfig,
        obstacle::Obstacle,
        scene_config::SceneConfig,
        spawn_area_module::spawn_area::SpawnArea,
        station_module::station::Station,
    }, experiment::config::ExperimentConfig, path_finding_module::visibility_graph::VisibilityGraph, task_module::task_manager::{TaskDurationJitter, TaskManager}, terrain::{TerrainLoader, slip::SlipModel}, units::{duration::Duration, energy::Energy, voltage::Voltage}, utilities::{
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
    /// Per-agent initial SoC is drawn uniformly from
    /// `[initial_soc_min_percent, initial_soc_max_percent]` when both are
    /// set; otherwise every agent starts at the agent template's fixed
    /// `battery_soc` (unchanged default behavior). See `initial_soc_percents`.
    pub initial_soc_min_percent: Option<f32>,
    pub initial_soc_max_percent: Option<f32>,
    /// Seed for the initial-SoC draw, independent of the env's base `seed`
    /// (which drives spawn position/heading). Leave unset to piggyback off
    /// the base seed (`seed ^ INITIAL_SOC_RNG_SALT`) -- fine for "just turn
    /// SoC randomization on too". Set explicitly to decouple the two axes,
    /// e.g. hold this fixed while sweeping `seed` to isolate spawn-position
    /// variance, or vice versa, hold `seed` fixed and sweep this to isolate
    /// initial-SoC variance.
    pub initial_soc_seed: Option<u64>,
    /// Work-task duration jitter: when both are set, each `Point` action's
    /// duration is scaled (and each `Line` action's velocity inversely
    /// scaled) by an independent uniform draw from
    /// `[task_duration_jitter_min_factor, task_duration_jitter_max_factor]`
    /// per task instance -- e.g. `(0.8, 1.2)` for +/-20% work-time
    /// variability. `None` (either unset) keeps every task at its
    /// configured duration (unchanged default behavior). See
    /// `TaskDurationJitter` / `apply_task_duration_jitter` in
    /// `task_module::task_manager`.
    pub task_duration_jitter_min_factor: Option<f32>,
    pub task_duration_jitter_max_factor: Option<f32>,
    /// Seed for the task-duration draw, independent of `seed` and of
    /// `initial_soc_seed` -- same "leave unset to piggyback, set to
    /// decouple" pattern as `initial_soc_seed`.
    pub task_duration_jitter_seed: Option<u64>,
}

/// Arbitrary constant XORed into the base experiment seed so the initial-SoC
/// draw's default stream is independent of the spawn-position/heading RNG
/// even when no explicit `initial_soc_seed` is given. This keeps spawn
/// positions identical for a given seed whether or not initial-SoC
/// randomization is enabled, and lets a seed sweep attribute variance to
/// this axis specifically instead of blending it with spawn randomness.
const INITIAL_SOC_RNG_SALT: u64 = 0x5343_0001;

/// Per-agent initial SoC: the fixed template value (default, deterministic),
/// or one uniform draw per agent from `[min, max]` when the experiment opts
/// in via `overrides`. The draw's seed defaults to `base_seed` (salted) but
/// can be set independently via `overrides.initial_soc_seed` -- see
/// `EnvOverrides::initial_soc_seed`.
fn initial_soc_percents(
    base_seed: u64,
    n_agents: u32,
    template_soc: f32,
    overrides: Option<&EnvOverrides>,
) -> Vec<f32> {
    let range = overrides.and_then(|o| {
        o.initial_soc_min_percent.zip(o.initial_soc_max_percent)
    });
    match range {
        Some((min, max)) => {
            let soc_seed = overrides
                .and_then(|o| o.initial_soc_seed)
                .unwrap_or(base_seed ^ INITIAL_SOC_RNG_SALT);
            let mut rng = StdRng::seed_from_u64(soc_seed);
            (0..n_agents).map(|_| rng.random_range(min..=max)).collect()
        }
        None => vec![template_soc; n_agents as usize],
    }
}

/// Arbitrary constant XORed into the base experiment seed so the
/// task-duration jitter draw's default stream is independent of both the
/// spawn-position/heading RNG and the initial-SoC RNG (see
/// `INITIAL_SOC_RNG_SALT`) even when no explicit
/// `task_duration_jitter_seed` is given.
const TASK_DURATION_JITTER_RNG_SALT: u64 = 0x4455_5201;

/// Resolves `overrides` into a `TaskDurationJitter` ready to hand to
/// `TaskManager::from_config`, or `None` if the min/max factors aren't both
/// set. The draw's seed defaults to `base_seed` (salted) but can be set
/// independently via `overrides.task_duration_jitter_seed` -- see
/// `EnvOverrides::task_duration_jitter_seed`.
fn task_duration_jitter(
    base_seed: u64,
    overrides: Option<&EnvOverrides>,
) -> Option<TaskDurationJitter> {
    let o = overrides?;
    let (min_factor, max_factor) = o
        .task_duration_jitter_min_factor
        .zip(o.task_duration_jitter_max_factor)?;
    let seed = o
        .task_duration_jitter_seed
        .unwrap_or(base_seed ^ TASK_DURATION_JITTER_RNG_SALT);
    Some(TaskDurationJitter { min_factor, max_factor, seed })
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
    /// Seed for the environment's random number generator (agent spawn position/orientation).
    pub seed: u64,
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
        let mut rng = StdRng::seed_from_u64(config.seed);
        let initial_socs = initial_soc_percents(
            config.seed,
            n_agents,
            agent_template.battery_soc,
            overrides,
        );

        // --- heavy models  ---
        let terrain = TerrainLoader::from_gps_csv(
            "configs/scene_configs/vineyard_scene/baggy-altitude-empirical-lut.csv",
        );

        let slip_model = SlipModel::from_json_file(
            "configs/scene_configs/vineyard_scene/baggy-slip-linear.json",
        );

        let slope_consumption = SlopeConsumptionConfig::from_json_file(
            "configs/movement_configs/consumption/slope_consumption.json",
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
                ChargingModel::from_config(&battery_config);

            let discharging_model = match battery_config.discharge_model {
                DischargeModelKind::Physics => DischargingModel::Physics(
                    PhysicsDischargeModel::new(
                        slip_model.clone(),
                        slope_consumption.travel,
                        slope_consumption.work,
                        terrain.clone(),
                    )
                ),
                DischargeModelKind::Simple => DischargingModel::Simple,
            };

            let battery = Battery::from_config(
                battery_config,
                initial_socs[i as usize],
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
                    &mut rng,
                ),
                random_vec2(&mut rng),
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
                task_duration_jitter(config.seed, overrides),
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
            seed: config.seed,
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

        let slope_consumption = SlopeConsumptionConfig::from_json_file(
            "configs/movement_configs/consumption/slope_consumption.json",
        );

        let mut rng = StdRng::seed_from_u64(self.seed);
        let initial_socs = initial_soc_percents(
            self.seed,
            self.n_agents,
            agent_template.battery_soc,
            overrides,
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
                ChargingModel::from_config(&battery_config);

            let discharging_model = match battery_config.discharge_model {
                DischargeModelKind::Physics => DischargingModel::Physics(
                    PhysicsDischargeModel::new(
                        slip_model.clone(),
                        slope_consumption.travel,
                        slope_consumption.work,
                        terrain.clone(),
                    )
                ),
                DischargeModelKind::Simple => DischargingModel::Simple,
            };

            let battery = Battery::from_config(
                battery_config,
                initial_socs[i as usize],
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
                    &mut rng,
                ),
                random_vec2(&mut rng),
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

#[cfg(test)]
mod tests {
    use super::*;
    use egui::Pos2;

    fn spawn_positions(seed: u64, n_agents: u32) -> Vec<Pos2> {
        let mut config = EnvConfig::default();
        config.n_agents = n_agents;
        config.seed = seed;
        Env::from_config(config)
            .agents
            .iter()
            .map(|a| a.spawn_position)
            .collect()
    }

    #[test]
    fn same_seed_reproduces_identical_spawn_positions() {
        assert_eq!(spawn_positions(1, 4), spawn_positions(1, 4));
    }

    #[test]
    fn different_seeds_produce_different_spawn_positions() {
        assert_ne!(spawn_positions(1, 4), spawn_positions(2, 4));
    }

    #[test]
    fn agents_within_a_run_do_not_collapse_onto_the_same_spawn_point() {
        let positions = spawn_positions(1, 4);
        assert!(positions.windows(2).any(|w| w[0] != w[1]));
    }

    #[test]
    fn initial_soc_defaults_to_template_value_when_overrides_absent() {
        let socs = initial_soc_percents(1, 4, 77.0, None);
        assert_eq!(socs, vec![77.0; 4]);
    }

    #[test]
    fn initial_soc_defaults_to_template_value_when_range_not_set() {
        let overrides = EnvOverrides::default();
        let socs = initial_soc_percents(1, 4, 77.0, Some(&overrides));
        assert_eq!(socs, vec![77.0; 4]);
    }

    #[test]
    fn initial_soc_draws_fall_within_the_requested_range() {
        let overrides = EnvOverrides {
            initial_soc_min_percent: Some(40.0),
            initial_soc_max_percent: Some(90.0),
            ..Default::default()
        };
        let socs = initial_soc_percents(1, 8, 77.0, Some(&overrides));
        assert!(socs.iter().all(|&s| (40.0..=90.0).contains(&s)));
        // With 8 draws from a wide range the odds of them all landing on
        // the same value are astronomically small.
        assert!(socs.windows(2).any(|w| w[0] != w[1]));
    }

    #[test]
    fn initial_soc_is_reproducible_for_the_same_seed() {
        let overrides = EnvOverrides {
            initial_soc_min_percent: Some(40.0),
            initial_soc_max_percent: Some(90.0),
            ..Default::default()
        };
        let a = initial_soc_percents(5, 4, 77.0, Some(&overrides));
        let b = initial_soc_percents(5, 4, 77.0, Some(&overrides));
        assert_eq!(a, b);
    }

    #[test]
    fn explicit_initial_soc_seed_decouples_it_from_the_base_seed() {
        // Same base seed, different explicit SoC seeds -> different draws.
        let a = EnvOverrides {
            initial_soc_min_percent: Some(40.0),
            initial_soc_max_percent: Some(90.0),
            initial_soc_seed: Some(1),
            ..Default::default()
        };
        let b = EnvOverrides {
            initial_soc_seed: Some(2),
            ..a.clone()
        };
        assert_ne!(
            initial_soc_percents(999, 4, 77.0, Some(&a)),
            initial_soc_percents(999, 4, 77.0, Some(&b)),
        );

        // Different base seeds, same explicit SoC seed -> identical draws:
        // the SoC axis can be held fixed while the base seed (spawn
        // position/heading) varies.
        assert_eq!(
            initial_soc_percents(1, 4, 77.0, Some(&a)),
            initial_soc_percents(2, 4, 77.0, Some(&a)),
        );
    }

    #[test]
    fn enabling_initial_soc_randomization_does_not_change_spawn_positions() {
        // The SoC draw must use its own RNG stream, independent of the one
        // used for spawn position/heading -- otherwise turning this axis on
        // would silently break reproducibility of existing seed-based runs.
        let mut config = EnvConfig::default();
        config.n_agents = 4;
        config.seed = 1;

        let without_soc_override = Env::from_config(config.clone())
            .agents
            .iter()
            .map(|a| a.spawn_position)
            .collect::<Vec<_>>();

        let overrides = EnvOverrides {
            initial_soc_min_percent: Some(10.0),
            initial_soc_max_percent: Some(100.0),
            ..Default::default()
        };
        let with_soc_override =
            Env::from_config_with_overrides(config, Some(&overrides))
                .agents
                .iter()
                .map(|a| a.spawn_position)
                .collect::<Vec<_>>();

        assert_eq!(without_soc_override, with_soc_override);
    }
}