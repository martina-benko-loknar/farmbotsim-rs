use std::collections::VecDeque;

use crate::{
    battery_module::{
        battery_config::BatteryConfig, charging::CcCvChargingModel, discharging::physics_model::PhysicsDischargeModel, is_battery::IsBattery
    },
    units::{duration::Duration, energy::Energy, power::Power, voltage::Voltage},
};
/// Represents a rechargeable battery with energy capacity, voltage, and a CC-CV charging model.
#[derive(Clone, Debug, PartialEq)]
pub struct Battery {
    pub voltage: Voltage,
    pub capacity: Energy,
    pub soc: f32,
    pub energy: Energy,

    update_count: u32,
    pub soc_history: VecDeque<f32>,

    pub charging_model: CcCvChargingModel,
    pub discharging_model: PhysicsDischargeModel
}

impl IsBattery for Battery {
    /// Reduces battery energy based on power draw and elapsed time.
    fn discharge(&mut self, power: Power, duration: Duration) {
        if self.energy <= Energy::ZERO {
            return;
        }

        let energy_removed = power * duration;
        let new_energy = self.energy - energy_removed;

        self.energy = new_energy.max(Energy::ZERO);
        self.soc = (self.energy / self.capacity) * 100.0;

        self.update();
    }
    
    /// Increases battery energy using the CC-CV charging model.
    fn charge(&mut self, duration: Duration) {
        if self.energy >= self.capacity {
            return;
        }

        let new_energy = self.charging_model.compute_charge(self.energy, duration);

        self.energy = new_energy.min(self.capacity);
        self.soc = (self.energy / self.capacity) * 100.0;
        self.update();
    }
    
    /// Returns the current battery state of charge.
    fn get_soc(&self) -> f32 {
        (self.energy / self.capacity) * 100.0
    }
    
    /// Updates the current energy based on state of charge.
    fn recalculate_energy(&mut self) {
        self.energy = self.capacity * self.soc * 0.01;
    }
}

impl Battery {
    /// Creates a battery from configuration and an initial SoC.
    pub fn from_config(
        config: BatteryConfig,
        initial_soc: f32,
        charging_model: CcCvChargingModel,
        discharging_model: PhysicsDischargeModel,
    ) -> Self {
        let soc = initial_soc.clamp(0.0, 100.0);

        Self {
            voltage: config.voltage,
            capacity: config.capacity,
            soc,
            energy: (soc / 100.0) * config.capacity,

            update_count: 0,
            soc_history: VecDeque::from(vec![soc; 100]),

            charging_model,
            discharging_model,
        }
    }

    
    /// Periodically stores the latest SoC in the history.
    fn update(&mut self) {
        self.update_count += 1;

        if self.update_count % 300 == 0 {
            self.soc_history.pop_front();
            self.soc_history.push_back(self.soc);
        }
    }

}