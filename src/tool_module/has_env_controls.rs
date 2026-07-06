use crate::tool_module::{path_tool::PathTool, simulation_tool::SimulationTool, task_tool::TaskTool};

/// Trait defining a common interface for environment control behavior.
pub trait HasEnvControls {
    /// Returns whether the environment simulation is currently running.
    fn is_running(&self) -> bool;

    /// Sets the running state of the environment simulation.
    fn set_running(&mut self, running: bool);

    /// Returns the current tick or frame count of the simulation.
    fn tick(&self) -> u32;

    /// Resets the environment simulation to its initial state.
    fn reset(&mut self);

    /// Returns the current step count from the environment.
    fn env_step_count(&self) -> u32;

    /// Renders a UI panel in egui with controls for the environment simulation.
    fn ui_render_controls(&mut self, ui: &mut egui::Ui) {
        ui.label(egui::RichText::new("Env controls:").size(16.0));

        ui.label(format!("Running: {}", self.is_running()));
        ui.label(format!("Env_step: {}", self.env_step_count()));

        if !self.is_running() {
            if self.tick() == 0 {
                if ui.button("Start").clicked() {
                    self.set_running(true);
                } 
            } else if ui.button("Resume").clicked() {
                self.set_running(true);
            }
        } else if ui.button("Pause").clicked() {
            self.set_running(false);
        }
        if ui.button("Reset").clicked() {
            self.reset();
        }
    }
}

macro_rules! impl_has_env_controls {
    ($t:ty) => {
        impl HasEnvControls for $t {
            fn is_running(&self) -> bool {
                self.running
            }
            fn set_running(&mut self, running: bool) {
                self.running = running;
            }
            fn tick(&self) -> u32 {
                self.tick
            }
            fn reset(&mut self) {
                self.tick = 0;
                self.running = false;
                self.env.reset_default();
            }
            fn env_step_count(&self) -> u32 {
                self.env.step_count
            }
        }
    };
}

impl_has_env_controls!(SimulationTool);
impl_has_env_controls!(PathTool);
impl_has_env_controls!(TaskTool);
