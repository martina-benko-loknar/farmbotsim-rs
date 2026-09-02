use crate::battery_module::battery_config::BatteryConfig;
use crate::units::{
    duration::Duration,
    energy::Energy,
    power::Power,
};

/// CC-CV (constant-current / constant-voltage) charging model.
///
/// Below `cv_fraction` of `capacity` the battery charges at constant power
/// (`cc_power`). Above it, the charger holds voltage constant and the charge
/// current tapers off, which is approximated here as an exponential approach
/// to `capacity` with time constant `cv_time_constant`.
///
/// The two phases are fit/configured independently, so nothing forces charging
/// *power* to be continuous at the `cv_fraction` boundary in general: the CV
/// phase's instantaneous power at the transition is
/// `(1 - cv_fraction) * capacity / cv_time_constant`, which only matches
/// `cc_power` if `cv_time_constant` is chosen as
/// `(1 - cv_fraction) * capacity / cc_power`. Configs should set
/// `cv_time_constant` this way unless deliberately modeling a discontinuous
/// (e.g. measured) taper -- see `configs/batteries/leo_rover/config.json`.
#[derive(Clone, Debug, PartialEq)]
pub struct CcCvChargingModel {
    pub capacity: Energy,
    pub cc_power: Power,
    pub cv_fraction: f32,
    pub cv_time_constant: Duration,
}

impl CcCvChargingModel {
    /// The CV-phase exponential only approaches `capacity` asymptotically and
    /// never reaches it exactly. Once the remaining energy gap drops below
    /// this fraction of `capacity`, treat the battery as fully charged.
    const FULL_CHARGE_EPSILON: f32 = 0.001;

    pub fn compute_charge(
        &self,
        current_energy: Energy,
        duration: Duration,
    ) -> Energy {

        let cv_energy = self.capacity * self.cv_fraction;

        // -----------------------------
        // Constant current phase
        // -----------------------------

        if current_energy < cv_energy {

            let added = self.cc_power * duration;

            return (current_energy + added)
                .min(self.capacity);
        }

        // -----------------------------
        // Constant voltage phase
        // -----------------------------

        let remaining = self.capacity - current_energy;
        let rate = 1.0 / self.cv_time_constant.to_base_unit();
        let seconds = duration.to_base_unit();

        let new_energy =
            self.capacity -
            remaining *
            (-rate * seconds).exp();

        if remaining.to_base_unit() < self.capacity.to_base_unit() * Self::FULL_CHARGE_EPSILON {
            return self.capacity;
        }

        new_energy.min(self.capacity)
    }

    pub fn from_config(config: &BatteryConfig) -> Self {
        Self {
            capacity: config.capacity,
            cc_power: config.cc_power
                .unwrap_or_else(|| panic!("battery '{}' has no cc_power for CC-CV charging", config.name)),
            cv_fraction: config.cv_fraction
                .unwrap_or_else(|| panic!("battery '{}' has no cv_fraction for CC-CV charging", config.name)),
            cv_time_constant: config.cv_time_constant
                .unwrap_or_else(|| panic!("battery '{}' has no cv_time_constant for CC-CV charging", config.name)),
        }
    }
}