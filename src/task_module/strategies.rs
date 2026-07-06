/// Strategies for selecting a charging station.
#[derive(Debug, Clone, PartialEq, serde::Serialize, serde::Deserialize, enum_iterator::Sequence)]
pub enum ChooseStationStrategy {
    /// Choose closest charging station (Manhattan)
    ClosestManhattan,
    /// Choose closest charging station (Path)
    ClosestPath,
    /// Choose closest charging station with min queue (Manhattan)
    ClosestMinQueueManhattan,
    /// Choose closest charging station with min queue (Path)
    ClosestMinQueuePath,
}
impl ChooseStationStrategy {
    pub fn variants() -> Vec<ChooseStationStrategy> {
        vec![
            ChooseStationStrategy::ClosestManhattan,
            ChooseStationStrategy::ClosestPath,
            ChooseStationStrategy::ClosestMinQueueManhattan,
            ChooseStationStrategy::ClosestMinQueuePath,
        ]
    }
}
impl std::fmt::Display for ChooseStationStrategy {
    fn fmt(&self, f: &mut std::fmt::Formatter) -> std::fmt::Result {
        let str = match self {
            Self::ClosestManhattan => "ClosestManhattan".to_string(),
            Self::ClosestPath => "ClosestPath".to_string(),
            Self::ClosestMinQueueManhattan => "ClosestMinQueueManhattan".to_string(),
            Self::ClosestMinQueuePath => "ClosestMinQueuePath".to_string(),
        };
        write!(f, "{str}")
    }
}

/// Strategies with behavior when to charge.
#[derive(Debug, Clone, PartialEq, serde::Serialize, serde::Deserialize)]
pub enum ChargingStrategy {
    /// Go charging only if battery is below critical
    CriticalOnly,
    /// Go charging if battery is below threshold and station is available
    /// Go charging if battery is below critical
    ThresholdWithLimit,
}
impl ChargingStrategy {
    pub fn variants() -> Vec<ChargingStrategy> {
        vec![ChargingStrategy::CriticalOnly, ChargingStrategy::ThresholdWithLimit]
    }
}
impl std::fmt::Display for ChargingStrategy {
    fn fmt(&self, f: &mut std::fmt::Formatter) -> std::fmt::Result {
        let str = match self {
            Self::CriticalOnly => "CriticalOnly".to_string(),
            Self::ThresholdWithLimit => "ThresholdWithLimit".to_string(),
        };
        write!(f, "{str}")
    }
}