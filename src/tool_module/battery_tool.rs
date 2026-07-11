use std::collections::HashMap;
use egui::Slider;
use egui_plot::{HLine, Legend, Line, Plot, PlotPoints};

use crate::{
    battery_module::{battery::Battery, battery_config::BatteryConfig, is_battery::IsBattery}, cfg::BATTERIES_PATH, tool_module::{has_help::HasHelp, tool::Tool}, utilities::utils::get_folders_in_folder
};

use crate::battery_module::charging::CcCvChargingModel;
use crate::battery_module::discharging::physics_model::PhysicsDischargeModel;
use crate::battery_module::discharging::VoltageDropLUT;
use crate::terrain::TerrainLoader;
use crate::terrain::slip::SlipModel;
use crate::units::duration::Duration;
use crate::units::energy::Energy;

/// A tool for inspecting and interacting with battery configuration data.
#[derive(Debug)]
pub struct BatteryTool {
    selected: Option<String>,
    folder_names: Vec<String>,
    battery_map: HashMap<String, Battery>,
    charge_curve: HashMap<String, Vec<(f32, f32)>>,
    pub help_open: bool,
}

impl Default for BatteryTool {
    fn default() -> Self {
        let folders = get_folders_in_folder(BATTERIES_PATH);
        Self {
            selected: None,
            folder_names: folders,
            battery_map: HashMap::new(),
            charge_curve: HashMap::new(),
            help_open: false,
        }
    }
}

/// Simulates charging from empty to (near) full and records energy over time,
/// mirroring the step-wise updates the simulation itself performs.
fn compute_charge_curve(model: &CcCvChargingModel) -> Vec<(f32, f32)> {
    let step = Duration::seconds(10.0);
    let mut energy = Energy::ZERO;
    let mut elapsed = 0.0_f32;
    let mut points = vec![(elapsed, energy.to_watt_hour())];

    while energy < model.capacity {
        let next_energy = model.compute_charge(energy, step);
        elapsed += step.to_base_unit();
        points.push((elapsed, next_energy.to_watt_hour()));

        if (next_energy - energy).to_base_unit().abs() < f32::EPSILON {
            break;
        }
        energy = next_energy;
    }

    points
}

impl Tool for BatteryTool {
    fn render_main(&mut self, ui: &mut egui::Ui) {
        match &self.selected {
            None => {}
            Some(selected) => {
                if let Some(battery) = self.battery_map.get(selected) {
                    let curve: PlotPoints = self.charge_curve
                        .get(selected)
                        .map(|points| {
                            points
                                .iter()
                                .map(|(x, y)| [f64::from(*x), f64::from(*y)])
                                .collect::<Vec<_>>()
                        })
                        .unwrap_or_default()
                        .into();
                    let line_charge_curve = Line::new("CC-CV charge curve", curve);

                    Plot::new("battery_plot")
                        .legend(Legend::default())
                        .auto_bounds(true)
                        .x_axis_label("Time (s)")
                        .y_axis_label("Energy (Wh)")
                        .show(ui, |plot_ui| {
                            let line = HLine::new("Current energy", battery.energy.value);
                            plot_ui.hline(line);
                            plot_ui.line(line_charge_curve);
                        });
                }
            }
        }
    }

    fn render_ui(&mut self, ui: &mut egui::Ui) {
        self.render_help_button(ui);
        ui.separator();

        ui.label("Batteries");

        for folder in &self.folder_names {
            let whole_path = format!("{}{}", BATTERIES_PATH, folder.clone());

            if ui.button(whole_path.clone()).clicked() {

                self.selected = Some(whole_path.clone());

                self.battery_map
                .entry(whole_path.clone())
                .or_insert_with(|| {

                    // Battery config
                    let battery_config =
                        BatteryConfig::from_json_file(
                            whole_path.clone(),
                        );
                    // Charging model
                    let charging_model =
                        CcCvChargingModel::from_config(&battery_config);

                    self.charge_curve.insert(
                        whole_path.clone(),
                        compute_charge_curve(&charging_model),
                    );

                    // Terrain
                    let terrain =
                        TerrainLoader::from_gps_csv(
                            "configs/scene_configs/vineyard_scene/baggy-altitude-empirical-lut.csv",
                        );
                    // Slip model
                    let slip_model =
                        SlipModel::from_json_file(
                            "configs/scene_configs/vineyard_scene/baggy-slip-linear.json",
                        );
                    // Voltage-drop LUT
                    let voltage_drop_lut =
                        VoltageDropLUT::from_csv(
                            "configs/movement_configs/consumption/fitted_lut.csv",
                        );
                    // Physics discharge model
                    let discharging_model =
                        PhysicsDischargeModel::new(
                            slip_model,
                            voltage_drop_lut,
                            terrain,
                        );
                    // Battery
                    Battery::from_config(
                        battery_config,
                        70.0,
                        charging_model,
                        discharging_model)
                });
            }
        }
        if ui.button("Deselect").clicked() {
            self.selected = None;
        }

        ui.separator();

        if let Some(selected) = &self.selected {
            if let Some(battery) = self.battery_map.get_mut(selected) {
                ui.label("Double click on plot to reset");
                ui.spacing();
                ui.heading(selected);
                ui.label(format!("Voltage: {}", battery.voltage));
                ui.label(format!("Capacity: {}", battery.capacity));
                ui.label(format!("Energy: {}", battery.energy));

                let response = ui.add(Slider::new(
                    &mut battery.soc,
                    0.0..=100.0)
                    .text("SoC [%]")
                    .step_by(1.0)
                );
                if response.changed() {
                    battery.recalculate_energy();
                }
            }
        }

        self.render_help(ui);
    }

    fn update(&mut self) {

    }
}

impl HasHelp for BatteryTool {
    fn help_modal(&self) -> egui::Modal {
        egui::Modal::new(egui::Id::new("Battery Tool Help"))
    }
    fn render_help_contents(&self, ui: &mut egui::Ui) {
        ui.heading("Battery Tool Help");
        ui.label("This is a battery tool where you can see selected battery charging characteristics and parameters.");
        ui.separator();

        ui.label("The plot shows the simulated CC-CV charge curve (energy vs. time from empty to full) for the selected battery.");
    }
}
