use crate::{
    battery_module::{
        discharging::{
            lut::VoltageDropLUT,
        },
    },
    terrain::{height_map::HeightMap,
        slip::SlipModel},
    units::duration::Duration,
    units::energy::Energy,
};
use super::traits::DischargeModel;

#[derive(Clone, Debug, PartialEq)]
pub struct PhysicsDischargeModel {
    pub slip: SlipModel,
    pub lut: VoltageDropLUT,
    pub terrain: HeightMap,
}

impl PhysicsDischargeModel {
    pub fn new(
        slip: SlipModel,
        lut: VoltageDropLUT,
        terrain: HeightMap,
    ) -> Self {
        Self { slip, lut, terrain }
    }
}


impl DischargeModel for PhysicsDischargeModel {
    fn compute_energy_loss(
        &self,
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
            self.lut.get(
                slope_rad,
                robot_speed,
            );

        let dt =
            duration.to_base_unit() as f32;

        let distance =
            robot_speed * dt;

        // TODO: Physically meaningful conversion: Wh = V * Ah
        // TEMP: empirical scaling constant
        let energy_loss =
            v_drop * distance * 100.0 ;

        // println!(
        //     "robot_speed={} slope={} v_drop={} distance={} energy_loss={}",
        //     robot_speed,
        //     slope_rad,
        //     v_drop,
        //     distance,
        //     energy_loss
        // );

        Energy::watt_hours(energy_loss)
    }
}