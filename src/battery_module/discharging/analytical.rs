use super::traits::ConsumptionModel;
use crate::battery_module::battery::Battery;
use crate::units::energy::Energy;

pub struct AnalyticalConsumptionModel {                     
    pub a: f32,
    pub b: f32,
    pub c: f32,
    pub d: f32,
}

impl ConsumptionModel for AnalyticalConsumptionModel {

    fn consume(
        &self,
        battery: &mut Battery,
        duration_s: u32,
        speed: f32,
        slope: f32,
    ) {

        // travelled distance
        let distance = speed * duration_s as f32;

        // voltage (or energy) per meter
        let vpm =
            self.a
            + self.b * speed
            + self.c * slope
            + self.d * speed * slope;

        // total consumed energy TODO (INCONSISTENT UNITS) !!!
        let energy_used = vpm * distance;

        battery.energy = battery.energy - Energy::watt_hours(energy_used);

        // clamp to zero
        if battery.energy < crate::units::energy::Energy::ZERO {
            battery.energy = crate::units::energy::Energy::ZERO;
        }

        // update SoC
        battery.soc =
            (battery.energy / battery.capacity) * 100.0;
    }
}
