use egui::Pos2;

use crate::{movement_module::pose::Pose, task_module::task::{Intent, Task}, units::{angle::Angle, duration::Duration, linear_velocity::LinearVelocity, power::Power}};

/// An instance of a farm entity action, linked to a specific entity.
#[derive(Debug, Clone, PartialEq)]
pub enum FarmEntityActionInstance {
    /// A point action instance with position, duration, power, and identifiers.
    Point {
        id: u32, // crop_id
        field_id: u32,
        line_id: u32,
        pos: Pos2,
        duration: Duration,
        power: Power,
        action_name: String,
    },
    /// A line action instance with a path, velocity, power, and identifiers.
    Line {
        id: u32, // row_id
        field_id: u32,
        path: Vec<Pos2>,
        velocity: LinearVelocity,
        power: Power,
        action_name: String,
    },
    /// A wait action instance with duration and identifier.
    Wait {
        id: u32, // crop_id / row_id
        duration: Duration,
        action_name: String,
    },
}
impl FarmEntityActionInstance {
    /// Creates a new point action instance.
    pub fn point(id: u32, field_id: u32, line_id: u32, pos: Pos2, duration: Duration, power: Power, action_name: String) -> Self {
        Self::Point { id, field_id, line_id, pos, duration, power, action_name }
    }
    /// Creates a new line action instance.
    pub fn line(id: u32, field_id: u32, path: Vec<Pos2>, velocity: LinearVelocity, power: Power, action_name: String) -> Self {
        Self::Line { id, field_id, path, velocity, power, action_name }
    }
    /// Creates a new wait action instance.
    pub fn wait(id: u32, duration: Duration) -> Self {
        Self::Wait { id, duration, action_name: "waiting".to_string() }
    }
    
    /// Converts the action instance to a `Task`.
    /// Returns None if it can't be converted.
    pub fn to_task(&self, task_id: u32) -> Option<Task> {
        match self {
            FarmEntityActionInstance::Point { id, field_id, line_id, pos, duration, power, action_name } => {
                Some(Task::Stationary {
                id: task_id,
                pose: Pose::new(*pos, Angle::ZERO),
                duration: *duration,
                intent: Intent::Work,
                farm_entity_id: *id,
                field_id: *field_id,
                line_id: *line_id,
                power: *power,
                info: action_name.clone(),
                })
            },
            FarmEntityActionInstance::Line { id, field_id, path, velocity, power, action_name } => {
                let path = path
                    .iter()
                    .map(|pos| Pose {
                        position: *pos,
                        orientation: Angle::degrees(90.0),
                    })
                    .collect();
                Some(Task::Moving {
                id: task_id,
                path,
                velocity: *velocity,
                intent: Intent::Work,
                field_id: *field_id,
                farm_entity_id: *id,
                power: *power,
                info: action_name.clone(),
                })
            },
            _ => None
        }
    }

}