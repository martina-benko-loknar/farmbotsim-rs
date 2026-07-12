use egui::Pos2;

use crate::{environment::field_config::FieldConfig, experiment::models::{
    EvaluatedLayout, ExperimentMetrics, SpecialistLayoutResults, StationLayout
}};
use crate::experiment::evaluation::evaluate_station_layout;
use crate::environment::geometry::FieldBounds;
use crate::environment::scene_config::SceneConfig;
use crate::experiment::config::ExperimentConfig;

/// Distance outside the field's row bounds at which edge/corner stations
/// are placed, matching the padding grid-search uses to keep candidate
/// positions clear of the rows themselves.
const FIELD_PADDING: f32 = 1.0;

/// Fixed gap between the two stations in the "tight_center" layout. Kept
/// as an absolute distance rather than scaled to field size, since it
/// represents a minimum physical station separation, not a field-relative
/// position.
const TIGHT_CENTER_GAP: f32 = 2.0;

/// Predefined human/specialist-inspired layouts, derived from the field's
/// actual bounds so they scale to whatever field_config is in use.
pub fn specialist_layouts(
    field_config: &FieldConfig
) -> Vec<StationLayout> {

    if field_config.configs.is_empty() {
        return Vec::new();
    }

    // ---------------------------------------------------------
    // Extract geometry
    // ---------------------------------------------------------

    let bounds = FieldBounds::from_field_config(field_config)
        .padded(FIELD_PADDING);

    let left_x = bounds.min_x;
    let right_x = bounds.max_x;

    let bottom_y = bounds.min_y;
    let top_y = bounds.max_y;

    let center_x = (left_x + right_x) / 2.0;
    let center_y = (bottom_y + top_y) / 2.0;

    let height = top_y - bottom_y;

    vec![
        StationLayout {
            name: "diagonal_corners".to_string(),
            stations: vec![
                Pos2::new(left_x, bottom_y),
                Pos2::new(right_x, top_y),
            ],
        },

        StationLayout {
            name: "horizontal_symmetry".to_string(),
            stations: vec![
                Pos2::new(left_x, center_y),
                Pos2::new(right_x, center_y),
            ],
        },

        StationLayout {
            name: "vertical_symmetry".to_string(),
            stations: vec![
                Pos2::new(center_x, bottom_y),
                Pos2::new(center_x, top_y),
            ],
        },

        StationLayout {
            name: "split_center".to_string(),
            stations: vec![
                Pos2::new(center_x, bottom_y + 0.25 * height),
                Pos2::new(center_x, bottom_y + 0.75 * height),
            ],
        },

        StationLayout {
            name: "tight_center".to_string(),
            stations: vec![
                Pos2::new(center_x, center_y - TIGHT_CENTER_GAP / 2.0),
                Pos2::new(center_x, center_y + TIGHT_CENTER_GAP / 2.0),
            ],
        },
    ]
}

pub fn evaluate_station_layouts(
    layouts: &[StationLayout],
    scene_config: &SceneConfig,
    exp: &ExperimentConfig
) -> SpecialistLayoutResults {

    let mut evaluated_layouts = Vec::new();

    for layout in layouts {

        println!(
            "Evaluating layout '{}' with {} stations...",
            layout.name, 
            layout.stations.len()
        );

        // -------------------------------------------------
        // Simulation call
        // -------------------------------------------------

        let result = 
            evaluate_station_layout(
                &layout.stations,
                &scene_config,
                &exp,
        );

        evaluated_layouts.push(EvaluatedLayout {
                layout: layout.clone(),
                metrics: ExperimentMetrics {
                    energy_wh: result.energy_wh,
                    total_distance_m: result.total_distance_m,
                    charging_distance_m: result.charging_distance_m,
                    simulation_time_sec: result.simulation_time_sec, 
                    evaluation_time_sec: result.evaluation_time_sec, 
                    charging_events: result.charging_events,
                    completed_tasks: result.completed_tasks,
                }
            }
        );
    }

    let best_layout = evaluated_layouts
        .iter()
        .min_by(|a, b| {
            a.metrics
                .energy_wh
                .partial_cmp(&b.metrics.energy_wh)
                .unwrap()
        })
        .unwrap()
        .clone();

    SpecialistLayoutResults {
        total_layouts: evaluated_layouts.len(),
        layouts: evaluated_layouts,
        best_layout,
    }
}