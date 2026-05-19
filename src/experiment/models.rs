use egui::Pos2;
use serde::{Deserialize, Serialize};

use crate::optimization::results::EvaluationRecord;

#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct GridSearchPoint {
    pub position: Pos2,
    pub energy: f64,
    pub total_distance: f64,
    pub charging_distance: f64,
    pub runtime_sec: f64,
}

#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct GridSearchResults {
    pub points: Vec<GridSearchPoint>,
    pub grid_resolution: usize,
    //pub optimization_minimum: Option<(Pos2, f64)>,
    pub valid_points: usize,
    pub total_points: usize,
    pub best_point: GridSearchPoint,
}

#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct EgoTrace {
    pub evaluation_history: Vec<EvaluationRecord>,
    pub max_iterations: usize,
}

#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct EgoSummary {
    pub optimal_position: Pos2,
    pub optimal_energy: f64,
    pub optimization_time_sec: f64,
    pub total_evaluations: usize,
}

#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct EgoOptimizationResults {
    pub summary: EgoSummary,
    pub trace: EgoTrace,
}

#[derive(Clone, Debug)]
pub struct SingleStationExperimentResults {
    pub ego: EgoOptimizationResults,
    pub grid_search: GridSearchResults,
}

#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct StationLayout {
    pub name: String,
    pub stations: Vec<Pos2>,
}

#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct EvaluatedLayout {
    pub layout: StationLayout,

    pub energy_wh: f64,
    pub time_sec: f64,
    pub total_distance_m: f64,
    pub charging_distance_m: f64,
}

#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct SpecialistLayoutResults {
    pub layouts: Vec<EvaluatedLayout>,

    pub best_layout: EvaluatedLayout,

    pub total_layouts: usize,
}

#[derive(Clone, Debug)]
pub struct MultiStationCandidate {
    pub stations: Vec<Pos2>,

    pub energy: f64,
    pub total_distance: f64,
    pub charging_distance: f64,
    pub time_sec: f64,

    pub label: String,
}

#[derive(Clone, Debug,  Serialize, Deserialize)]
pub struct MultiStationExperimentResults {
    pub ego: EgoOptimizationResults,
    pub specialist: SpecialistLayoutResults,
}