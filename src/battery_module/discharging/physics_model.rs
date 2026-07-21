use crate::{
    battery_module::{
        discharging::slope_consumption::SlopeConsumptionModel,
    },
    cfg::LEO_ROVER_NOMINAL_DRIVING_POWER_W,
    terrain::{height_map::HeightMap,
        slip::SlipModel},
    units::duration::Duration,
    units::energy::Energy,
};
use super::traits::DischargeModel;

const SECONDS_PER_HOUR: f32 = 3600.0;

#[derive(Clone, Debug, PartialEq)]
pub struct PhysicsDischargeModel {
    pub slip: SlipModel,
    pub travel_consumption: SlopeConsumptionModel,
    pub work_consumption: SlopeConsumptionModel,
    pub terrain: HeightMap,
    /// V-drop-to-Wh conversion factor (replaces the old flat placeholder
    /// constant). Calibrated so that flat-ground travel (slope=0, using
    /// travel_consumption's own reference speed) predicts an average power
    /// draw matching LEO_ROVER_NOMINAL_DRIVING_POWER_W -- see
    /// `calibrate_wh_scale` and cfg.rs for the derivation. Applied to both
    /// travel and work, since it's a units conversion, not an
    /// activity-specific tuning knob.
    wh_scale: f32,
}

impl PhysicsDischargeModel {
    pub fn new(
        slip: SlipModel,
        travel_consumption: SlopeConsumptionModel,
        work_consumption: SlopeConsumptionModel,
        terrain: HeightMap,
    ) -> Self {
        let wh_scale = Self::calibrate_wh_scale(&travel_consumption);
        Self { slip, travel_consumption, work_consumption, terrain, wh_scale }
    }

    /// Solves for the V-drop-to-Wh scale such that one hour of continuous
    /// flat-ground travel at `travel_consumption`'s reference speed
    /// consumes LEO_ROVER_NOMINAL_DRIVING_POWER_W watt-hours:
    ///   avg_power_w = v_drop_flat * ref_speed_mps * scale * SECONDS_PER_HOUR
    /// solved for `scale`. Only travel is used as the calibration anchor --
    /// there's no equivalent published spec for "work" power draw -- and
    /// the same scale is then reused for work too (see `wh_scale` doc).
    fn calibrate_wh_scale(travel_consumption: &SlopeConsumptionModel) -> f32 {
        LEO_ROVER_NOMINAL_DRIVING_POWER_W
            / (travel_consumption.intercept_v_per_m
                * travel_consumption.reference_speed_mps
                * SECONDS_PER_HOUR)
    }

    fn compute_energy_loss(
        &self,
        consumption: &SlopeConsumptionModel,
        wheel_speed: f32,
        slope_rad: f32,
        duration: Duration,
    ) -> Energy {

        let robot_speed =
            self.slip.compute_robot_speed(
                wheel_speed,
                slope_rad,
            );

        let v_drop =
            consumption.voltage_drop_per_m(slope_rad);

        let dt =
            duration.to_base_unit() as f32;

        let distance =
            robot_speed * dt;

        let energy_loss =
            v_drop * distance * self.wh_scale;

        Energy::watt_hours(energy_loss)
    }
}


impl DischargeModel for PhysicsDischargeModel {
    fn compute_travel_energy_loss(
        &self,
        wheel_speed: f32,
        slope_rad: f32,
        duration: Duration,
    ) -> Energy {
        self.compute_energy_loss(&self.travel_consumption, wheel_speed, slope_rad, duration)
    }

    fn compute_work_energy_loss(
        &self,
        wheel_speed: f32,
        slope_rad: f32,
        duration: Duration,
    ) -> Energy {
        self.compute_energy_loss(&self.work_consumption, wheel_speed, slope_rad, duration)
    }
}