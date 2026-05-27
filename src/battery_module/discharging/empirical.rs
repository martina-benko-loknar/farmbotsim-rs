use std::collections::HashMap;
use super::traits::ConsumptionModel;
use serde::Deserialize;
use crate::{
    battery_module::battery::{
        Battery},
        units::{
            energy::Energy}
        };

#[derive(Deserialize)]
struct EmpiricalEntry {
    slope: i8,
    mode: String,
    consumption: f32,
}

#[derive(Deserialize)]
struct EmpiricalConfig {
    entries: Vec<EmpiricalEntry>,
}

#[derive(Hash, Eq, PartialEq, Clone, Copy, Debug)]
pub enum MovementMode {
    Working,
    Travelling,
}

pub struct EmpiricalConsumptionModel {
    pub table: HashMap<(i8, MovementMode), f32>, // V/m
}

impl EmpiricalConsumptionModel {

    pub fn from_file(path: &str) -> Self {

        let content =
            std::fs::read_to_string(path)
                .expect("Failed to read empirical model");

        let parsed: EmpiricalConfig =
            serde_json::from_str(&content)
                .expect("Invalid JSON");

        let mut table = HashMap::new();

        for e in parsed.entries {

            let mode = match e.mode.as_str() {
                "working" => MovementMode::Working,
                "travelling" => MovementMode::Travelling,
                _ => panic!("Invalid mode"),
            };

            table.insert((e.slope, mode), e.consumption);
        }

        Self { table }
    }
}

impl ConsumptionModel for EmpiricalConsumptionModel {

    fn consume(
        &self,
        battery: &mut Battery,
        duration_s: u32,
        speed: f32,
        slope: f32,
    ) {

        let distance = speed * duration_s as f32;

        let mode =
            if speed > 0.5 {
                MovementMode::Travelling
            } else {
                MovementMode::Working
            };

        // Snap slope to nearest category
        let slope_key =
            ((slope / 5.0).round() * 5.0) as i8;

        let consumption =
            self.table
                .get(&(slope_key, mode))
                .copied()
                .unwrap_or(0.0);

        let energy_used = consumption * distance;

        battery.energy = battery.energy -
            Energy::watt_hours(energy_used);

        if battery.energy < Energy::ZERO {
            battery.energy = Energy::ZERO;
        }

        battery.soc =
            (battery.energy / battery.capacity) * 100.0;
    }
}