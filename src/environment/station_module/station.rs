use egui::Color32;
use std::collections::VecDeque;

use crate::{agent_module::agent::AgentId, movement_module::pose::Pose, units::{angle::Angle, length::Length}};
use super::station_config::StationConfig;

/// Represents the type of position an agent can occupy at a station.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum StationPosType {
    ChargingSlot,
    QueueSlot,
}

/// Represents station ID
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, PartialOrd, Ord)]
pub struct StationId(u32);
impl StationId {
    pub fn new(id: u32) -> Self {
        StationId(id)
    }
}
impl std::fmt::Display for StationId {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(f, "{}", self.0)
    }
}

/// Represents a station where agents can queue or occupy charging slots.
#[derive(Clone, Debug, PartialEq)]
pub struct Station {
    /// Unique identifier of the station.
    pub id: StationId,
    /// Global pose (position and orientation) of the station.
    pub pose: Pose,
    /// Direction in which the queue extends from the station.
    pub queue_direction: Angle,
    /// Distance between each queued agent.
    pub waiting_offset: Length,
    /// Color used for rendering.
    pub color: Color32,
    
    /// Number of charging slots available at the station.
    pub n_slots: u32,
    /// Relative poses of each charging slot, in local coordinates.
    pub slots_pose: Vec<Pose>,
    /// Current occupancy of each charging slot. `None` means empty.
    pub slots: Vec<Option<AgentId>>,
    /// Queue of agent IDs waiting for a slot.
    pub queue: VecDeque<AgentId>,
}

impl Default for Station {
    /// Creates a new `Station` using the default configuration.
    fn default() -> Self {
        let config = StationConfig::default();
        Self {
            id: StationId::new(0),
            pose: config.pose,
            queue_direction: config.queue_direction,
            waiting_offset: config.waiting_offset,
            color: Color32::RED,

            n_slots: config.n_slots,
            slots_pose: config.slots_pose,
            slots: vec![None; config.n_slots as usize],
            queue: VecDeque::new(),
        }
    }
}

impl Station {
    /// Constructs a [`Station`] from a [`StationConfig`], identifier, and color.
    pub fn from_config(id: u32, color: Color32, config: StationConfig) -> Self {
        Self {
            id: StationId::new(id),
            pose: config.pose,
            queue_direction: config.queue_direction,
            waiting_offset: config.waiting_offset,
            color,
            n_slots: config.n_slots,
            slots_pose: config.slots_pose,
            slots: vec![None; config.n_slots as usize],
            queue: VecDeque::new(),
        }
    }
    /// Converts the current `Station` into a `StationConfig`.
    pub fn to_config(&self) -> StationConfig {
        StationConfig::new(self.pose.clone(), self.queue_direction, self.waiting_offset, self.n_slots, self.slots_pose.clone())
    }
}

impl Station {
    /// Creates a new `Station` with explicit parameters.
    pub fn new(id: u32, pose: Pose, queue_direction: Angle, waiting_offset: Length, color: Color32, n_slots: u32, slots_pose: Vec<Pose>) -> Self {
        Self {
            id: StationId::new(id),
            pose,
            queue_direction,
            waiting_offset,
            color,

            n_slots,
            slots_pose,
            slots: vec![None; n_slots as usize],
            queue: VecDeque::new(),
        }
    }
    /// Resets the station: clears all slots and empties the queue.
    pub fn reset(&mut self) {
        self.slots = vec![None; self.n_slots as usize];
        self.queue.clear();
    }
    /// Returns the number of occupied charging slots.
    pub fn n_occupied_slots(&self) -> u32 {
        self.slots.iter().filter(|slot| slot.is_some()).count() as u32
    }
    /// Returns the index of the first empty charging slot, or `None` if full.
    pub fn get_empty_slot(&self) -> Option<usize> {
        self.slots.iter().position(|x| x.is_none())
    }
    /// Requests a charging position for the given agent.
    ///
    /// If a slot is available, it is assigned and its pose is returned.
    /// Otherwise, the agent is added to the queue and its pose is returned.
    pub fn request_charge(&mut self, agent_id: AgentId) -> (Pose, StationPosType) {
        if let Some(index) = self.get_empty_slot() {
            self.slots[index] = Some(agent_id);
            if let Some(pose) = self.get_pose_for_slot(index) {
                return (pose, StationPosType::ChargingSlot)
            } else {
                self.slots[index] = None;
            }
        }
        self.queue.push_back(agent_id);
        (self.get_waiting_pose(self.queue.len()-1), StationPosType::QueueSlot)
    }
    /// Releases the agent from either the queue or its charging slot.
    ///
    /// Returns `true` if the agent was found and removed.
    pub fn release_agent(&mut self, agent_id: AgentId) -> bool {
        let mut successfully_removed = false;
        successfully_removed |= self.remove_agent_from_slots(agent_id);
        successfully_removed |= self.remove_agent_from_queue(agent_id);
        successfully_removed
    }
    /// Moves an agent from the queue to a free charging slot.
    ///
    /// Returns the pose of the slot if successful.
    pub fn move_agent_from_queue_to_slot(&mut self, agent_id: AgentId) -> Option<Pose> {
        if let Some(index) = self.get_empty_slot() {
            if self.remove_agent_from_queue(agent_id) {
                self.slots[index] = Some(agent_id);
                return self.get_pose_for_slot(index)
            }
        }
        None
    }
    /// Returns the world pose for a queued agent based on its index.
    pub fn get_waiting_pose(&self, queue_index: usize) -> Pose {
        let distance = (queue_index as f32 + 1.0) * self.waiting_offset;
        let orientation = self.pose.orientation + self.queue_direction;
        let position = self.pose.position + orientation.to_vec2() * distance;
        Pose::new(position, orientation+Angle::degrees(180.0))
    }
    /// Returns the world pose for a charging slot by index, if it exists.
    pub fn get_pose_for_slot(&self, index: usize) -> Option<Pose> {
        self.slots_pose.get(index).cloned().map(|p| p + self.pose.clone())
    }

    /// Removes the agent from the charging slots, if present.
    ///
    /// Returns `true` if the agent was removed.
    fn remove_agent_from_slots(&mut self, agent_id: AgentId) -> bool {
        if self.slots.iter().any(|slot| *slot == Some(agent_id)) {
            self.slots.iter_mut().for_each(|slot| {
                if *slot == Some(agent_id) {
                    *slot = None;
                }
            });
            return true
        }
        false
    }
    /// Removes the agent from the queue, if present.
    ///
    /// Returns `true` if the agent was removed.
    fn remove_agent_from_queue(&mut self, agent_id: AgentId) -> bool {
        if self.queue.contains(&agent_id) {
            self.queue.retain(|&id| id != agent_id);
            return true
        }
        false
    }
}

