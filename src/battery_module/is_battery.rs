use crate::units::{duration::Duration, power::Power};

/// Provides an interface for battery operations.
pub trait IsBattery {
    /// Reduces battery energy based on power usage over time.
    fn discharge(&mut self, power: Power, duration: Duration);
    
    /// Increases battery energy using the CC-CV charging model.
    fn charge(&mut self, duration: Duration);
    
    /// Returns the current state of charge as a percentage.
    fn get_soc(&self) -> f32;
    
    /// Recalculates energy based on current state of charge.
    fn recalculate_energy(&mut self);
}