use serde::{Deserialize, Serialize};

use crate::{
    movement_module::{is_movement::IsMovement, pose::Pose, romba_movement::{RombaMovement, RombaMovementInputs}}, units::{
        angular_velocity::AngularVelocity,
        duration::Duration,
        linear_velocity::LinearVelocity,
    }, utilities::utils::load_json_or_panic
};

/// Represents movement control inputs for different movement models.
#[derive(Debug, Clone, Copy)]
pub enum MovementInputs {
    /// Inputs for the Romba movement model.
    Romba(RombaMovementInputs),
}

/// Represents a configurable movement model.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
#[serde(tag = "type", content = "params")]
pub enum Movement {
    /// Romba-based movement.
    RombaMovement(RombaMovement),
    /// Leo Rover movement: same differential-drive kinematics as
    /// `RombaMovement`, since the Leo Rover is itself a skid-steer/
    /// differential-drive robot -- just its own speed/wheel geometry.
    LeoRoverMovement(RombaMovement),
}
impl IsMovement for Movement {
    /// Delegates input calculation to the underlying movement model.
    fn calculate_inputs_for_target(&self, current_pose: &Pose, target_pose: &Pose) -> MovementInputs {
        match self {
            Movement::RombaMovement(romba) | Movement::LeoRoverMovement(romba) =>
                romba.calculate_inputs_for_target(current_pose, target_pose),
        }
    }
    /// Delegates pose update computation to the underlying movement model.
    fn calculate_new_pose_from_inputs(&self, simulation_step: Duration, inputs: MovementInputs, current_pose: Pose, max_velocity: LinearVelocity) -> (Pose, LinearVelocity, AngularVelocity) {
        match self {
            Movement::RombaMovement(romba) | Movement::LeoRoverMovement(romba) =>
                romba.calculate_new_pose_from_inputs(simulation_step, inputs, current_pose, max_velocity),
        }
    }
}
impl Movement {
    /// Loads a movement model from a JSON file, panicking on failure.
    pub fn from_json_file(file_path: String) -> Self {
        load_json_or_panic(file_path)
    }
    /// Returns the maximum allowed linear velocity for the movement model.
    pub fn max_velocity(&self) -> LinearVelocity {
        match &self {
            Movement::RombaMovement(rm) | Movement::LeoRoverMovement(rm) => rm.max_velocity
        }
    }
}
