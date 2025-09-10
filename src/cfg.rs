use crate::units::{angle::Angle, length::Length, linear_velocity::LinearVelocity, power::Power};

pub const TOLERANCE_DISTANCE: Length = Length::meters(0.005);
pub const TOLERANCE_ANGLE: Angle = Angle::degrees(0.05);

pub const POWER_CONSUMPTION_WAIT: Power = Power::watts(10.0); // W/s
pub const POWER_CONSUMPTION_TRAVEL: Power = Power::watts(2.0*350.0); // W/s

pub const MAX_VELOCITY_BETWEEN_POINTS: LinearVelocity = LinearVelocity::kilometers_per_hour(3.0); // between farm entities


pub const FARM_ENTITY_PLANS_PATH: &str = "configs/farm_entity_plans/";
pub const DEFAULT_POINT_FARM_ENTITY_PLAN_PATH: &str = "configs/farm_entity_plans/default_point.json";
pub const DEFAULT_LINE_FARM_ENTITY_PLAN_PATH: &str = "configs/farm_entity_plans/default_line.json";

pub const BATTERIES_PATH: &str = "configs/batteries/";

pub const MOVEMENT_CONFIGS_PATH: &str = "configs/movement_configs/";
pub const DEFAULT_ROMBA_MOVEMENT_CONFIG_PATH: &str = "configs/movement_configs/default_romba.json";

pub const AGENT_CONFIGS_PATH: &str = "configs/agent_configs/";
pub const DEFAULT_AGENT_CONFIG_PATH: &str = "configs/agent_configs/default.json";

pub const FIELD_CONFIGS_PATH: &str = "configs/field_configs/";
pub const DEFAULT_FIELD_CONFIG_PATH: &str = "configs/field_configs/default.json";

pub const SCENE_CONFIGS_PATH: &str = "configs/scene_configs/";
pub const DEFAULT_SCENE_CONFIG_PATH: &str = "configs/scene_configs/default.json";

pub const PERFORMANCE_MATRIX_PATH: &str = "performance_matrix/";
pub const EXPERIMENTS_PATH: &str = "experiments/";

pub const OPTIMIZATION_RESULTS_PATH: &str = "results/optimization/";