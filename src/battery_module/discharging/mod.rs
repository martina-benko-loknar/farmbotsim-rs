pub mod traits;
pub mod lut;
pub mod slope_consumption;
pub mod physics_model;
pub use lut::VoltageDropLUT;
pub use slope_consumption::{SlopeConsumptionConfig, SlopeConsumptionModel};

use crate::battery_module::discharging::traits::DischargeModel;
use crate::cfg::POWER_CONSUMPTION_TRAVEL;
use crate::units::{duration::Duration, energy::Energy, linear_velocity::LinearVelocity, power::Power};
use physics_model::PhysicsDischargeModel;

/// Which discharging model a `BatteryConfig` should build.
#[derive(Debug, Clone, Copy, PartialEq, serde::Serialize, serde::Deserialize)]
pub enum DischargeModelKind {
    Physics,
    Simple,
}

/// The current task's power draw, as seen by the simple discharging model.
/// Kept independent of `task_module::Task` so `battery_module` doesn't need
/// to depend on it -- callers translate their `Task` into this.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum WorkPowerDraw {
    Stationary(Power),
    Moving(Power),
    Other,
}

/// Wraps the concrete discharging models so `Battery` can hold either behind
/// one type, picked at construction time via `BatteryConfig::discharge_model`.
#[derive(Clone, Debug, PartialEq)]
pub enum DischargingModel {
    Physics(PhysicsDischargeModel),
    /// Legacy speed/task-based model (restored from before commit 3ac5244):
    /// no slope/terrain awareness, just a flat travel-power constant scaled
    /// by velocity ratio, plus each task's own declared power draw while
    /// working.
    Simple,
}

impl DischargingModel {
    /// Energy lost while traveling. Physics uses `wheel_speed`/`slope_rad`;
    /// Simple uses `velocity_lin`/`max_velocity` instead.
    pub fn compute_travel_energy_loss(
        &self,
        velocity_lin: LinearVelocity,
        max_velocity: LinearVelocity,
        wheel_speed: f32,
        slope_rad: f32,
        duration: Duration,
    ) -> Energy {
        match self {
            Self::Physics(model) =>
                model.compute_travel_energy_loss(wheel_speed, slope_rad, duration),
            Self::Simple => {
                let power = POWER_CONSUMPTION_TRAVEL * (velocity_lin / max_velocity);
                power * duration
            }
        }
    }

    /// Energy lost while working. Physics measures the same way as travel
    /// (slope/wheel-speed); Simple instead uses the task's own power draw
    /// (`work_power_draw`), matching the original calculate_power_work.
    pub fn compute_work_energy_loss(
        &self,
        velocity_lin: LinearVelocity,
        max_velocity: LinearVelocity,
        work_power_draw: WorkPowerDraw,
        wheel_speed: f32,
        slope_rad: f32,
        duration: Duration,
    ) -> Energy {
        match self {
            Self::Physics(model) =>
                model.compute_work_energy_loss(wheel_speed, slope_rad, duration),
            Self::Simple => {
                let travel_power = POWER_CONSUMPTION_TRAVEL * (velocity_lin / max_velocity);
                let power = match work_power_draw {
                    WorkPowerDraw::Stationary(power) => power,
                    WorkPowerDraw::Moving(power) => power + travel_power,
                    WorkPowerDraw::Other => Power::ZERO,
                };
                power * duration
            }
        }
    }
}