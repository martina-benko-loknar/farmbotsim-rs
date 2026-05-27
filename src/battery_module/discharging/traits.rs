use crate::battery_module::battery::Battery;
use crate::units::duration::Duration;
use crate::units::energy::Energy;

pub trait ConsumptionModel {

    fn consume(
        &self,
        battery: &mut Battery,
        duration_s: u32,
        speed: f32,
        slope: f32,
    );
}

pub trait DischargeModel {
    fn compute_energy_loss(
        &self,
        wheel_speed: f32,
        slope_rad: f32,
        duration: Duration,
    ) -> Energy;
}

