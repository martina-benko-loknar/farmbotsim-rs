use crate::units::{angle::Angle, length::Length, linear_velocity::LinearVelocity, power::Power};

pub const TOLERANCE_DISTANCE: Length = Length::meters(0.005);
pub const TOLERANCE_ANGLE: Angle = Angle::degrees(0.05);

pub const POWER_CONSUMPTION_WAIT: Power = Power::watts(10.0); // W/s
pub const POWER_CONSUMPTION_TRAVEL: Power = Power::watts(2.0*350.0); // W/s

/// Leo Rover's official average power draw while driving, backed out from
/// the manufacturer's published runtime spec for the same 3S2P 6800mAh
/// (73.2 Wh) battery used in configs/batteries/leo_rover: ~4h of nominal
/// driving on a full charge, i.e. 73.2 Wh / 4 h = 18.3 W.
/// Source: https://docs.fictionlab.pl/leo-rover/documentation/specification
/// No direct current/power-draw spec is published, so this runtime figure
/// is used instead to calibrate PhysicsDischargeModel's V-drop-to-Wh
/// conversion constant (see physics_model.rs) -- it's a single lumped
/// average over unspecified driving conditions, not a slope/speed-resolved
/// measurement, so treat it as a plausible anchor, not ground truth.
pub const LEO_ROVER_NOMINAL_DRIVING_POWER_W: f32 = 18.3;

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