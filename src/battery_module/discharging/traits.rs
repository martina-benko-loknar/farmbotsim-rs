use crate::units::duration::Duration;
use crate::units::energy::Energy;

pub trait DischargeModel {
    fn compute_energy_loss(
        &self,
        wheel_speed: f32,
        slope_rad: f32,
        duration: Duration,
    ) -> Energy;
}

