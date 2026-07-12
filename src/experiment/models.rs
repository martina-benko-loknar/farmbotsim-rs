use egui::Pos2;
use serde::{Deserialize, Serialize};
use crate::{experiment::config::ExperimentConfig};

// ============================================================
// Core experiment representation
// ============================================================
#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct ExperimentMetrics {
    pub energy_wh: f64,
    pub total_distance_m: f64,
    pub charging_distance_m: f64,
    pub simulation_time_sec: f64,
    pub evaluation_time_sec: f64,
    pub charging_events: u32,
    pub charge_attempts: u32,
    pub failed_charge_attempts: u32,
    pub completed_tasks: u32,
}
#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct ExperimentMetadata {
    pub experiment_type: ExperimentType,
    pub config: ExperimentConfig,
    pub timestamp: String,
    pub seed: u64,
}
#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct ExperimentContext {
    pub metadata: ExperimentMetadata,
    pub metrics: ExperimentMetrics,
}

#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct ExperimentResult {
    pub context: ExperimentContext,
}

#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct TimingStatistics {
    pub average_evaluation_time_sec: f64,
    pub minimum_evaluation_time_sec: f64,
    pub maximum_evaluation_time_sec: f64,
    pub total_evaluation_time_sec: f64,
}

#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct FieldExport {
    pub config_path: String,
    pub obstacle_count: usize,
    pub obstacles: Vec<Vec<Pos2>>,
}

// ============================================================
// Experiment info
// ============================================================

#[derive(Clone, Debug, Serialize, Deserialize)]
pub enum ExperimentType {
    SingleRun,
    GridSearch,
    EGO,
    SingleStation,
    MultiStation,
    FieldSweep,
    FleetSweep,
    BatterySweep,
    SocSweep,
}
#[derive(Clone, Debug, Serialize, Deserialize)]

pub struct ExperimentInfo {
    pub experiment_type: ExperimentType,
    pub timestamp: String,
}
// ============================================================
// Per-evaluation record 
// ============================================================
#[derive(Serialize, Deserialize, Clone, Debug)]
pub struct EvaluationRecord {
    pub evaluation: usize,
    pub phase: String,           // "init" | "ego"
    pub phase_iteration: usize,
    pub metrics: ExperimentMetrics,
    pub best_energy: f64,
    pub is_new_best: bool,
    pub positions: Vec<(f32, f32)>,
    pub best_positions: Vec<(f32, f32)>,
}

#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct EvaluatedCandidate {
    pub positions: Vec<(f32, f32)>,
    pub metrics: ExperimentMetrics,
}

// ============================================================
// Grid search experiment (full evaluation over spatial grid)
// ============================================================

#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct GridSearchPoint {
    pub position: Pos2,
    pub metrics:ExperimentMetrics,
}
#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct GridSearchTrace {
    pub points: Vec<GridSearchPoint>,
}
#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct GridSearchSummary {
    pub best_point: GridSearchPoint,
    pub valid_points: usize,
    pub total_points: usize,
    pub grid_resolution: usize,
}

// Algorithm output
#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct GridSearchResults {
    pub summary: GridSearchSummary,
    pub trace: GridSearchTrace,
}
// File format
#[derive(Serialize)]
pub struct GridSearchExport {
    pub metadata: ExperimentMetadata,
    pub grid_search: GridSearchResults,
    pub timing: TimingStatistics,
    pub field: FieldExport,
}
// ============================================================
// EGO optimization (Bayesian / iterative optimizer)
// ============================================================

#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct EgoTrace {
    pub evaluation_history: Vec<EvaluationRecord>,
    pub max_iterations: usize,
}
#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct EgoSummary {
    pub best_metrics: ExperimentMetrics,
    pub optimal_position: Vec<Pos2>,
    pub optimization_time_sec: f64,
    pub total_evaluations: usize,
}
#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct EgoOptimizationResults {
    pub summary: EgoSummary,
    pub trace: EgoTrace,
}

#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct EgoExport {
    pub metadata: ExperimentMetadata,
    pub ego: EgoOptimizationResults,
    pub timing: TimingStatistics,
    pub field: FieldExport,
}
// ============================================================
// Single-station experiment (comparison of methods)
// ============================================================

#[derive(Clone, Debug)]
pub struct SingleStationExperimentResults {
    pub ego: EgoOptimizationResults,
    pub grid_search: GridSearchResults,
}

// ============================================================
// Multi-station layout evaluation
// ============================================================

#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct StationLayout {
    pub name: String,
    pub stations: Vec<Pos2>,
}
#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct EvaluatedLayout {
    pub layout: StationLayout,
    pub metrics: ExperimentMetrics
}
#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct SpecialistLayoutResults {
    pub layouts: Vec<EvaluatedLayout>,
    pub best_layout: EvaluatedLayout,
    pub total_layouts: usize,
}

// ============================================================
// Multi-station experiment (EGO vs specialist baseline)
// ============================================================
#[derive(Clone, Debug,  Serialize, Deserialize)]
pub struct MultiStationExperimentResults {
    pub ego: EgoOptimizationResults,
    pub specialist: SpecialistLayoutResults,
}
// ============================================================
// Experiment execution metadata (file-level tracking)
// ============================================================
#[derive(Clone, Debug)]
pub struct ExperimentRun {
    pub timestamp: String,
    pub output_dir: String,
    pub results_path: String,
}

// ============================================================
// Exports
// ============================================================
#[derive(Clone, Debug, Serialize)]
pub struct SingleStationExport {
    pub metadata: ExperimentMetadata,
    pub ego: EgoOptimizationResults,
    pub grid_search: GridSearchResults,
    pub timing: TimingStatistics,
    pub field: FieldExport,
}

#[derive(Clone, Debug, Serialize)]
pub struct MultiStationExport {
    pub metadata: ExperimentMetadata,
    pub ego: EgoOptimizationResults,
    pub specialist: SpecialistLayoutResults,
    pub timing: TimingStatistics,
    pub field: FieldExport,
}