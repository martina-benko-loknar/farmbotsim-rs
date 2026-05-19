use egui::Pos2;

use crate::{environment::field_config::FieldConfig, experiment::models::{
    EvaluatedLayout, SpecialistLayoutResults, StationLayout
}};
use crate::experiment::evaluation::evaluate_station_layout;
use crate::environment::scene_config::SceneConfig;
use crate::cfg::DEFAULT_SCENE_CONFIG_PATH;
use crate::utilities::utils::load_json_or_panic;

/// Predefined human/specialist-inspired layouts
pub fn specialist_layouts(
    field_config: &FieldConfig
) -> Vec<StationLayout> {

    // TODO: 
    // struct FieldBounds {
    //     min_x: f32,
    //     max_x: f32,
    //     min_y: f32,
    //     max_y: f32,
    // }
    // let center_x = (bounds.min_x + bounds.max_x) / 2.0;

    // ---------------------------------------------------------
    // Assumptions: two Line configs, rectangular fields
    // ---------------------------------------------------------

    let fields = &field_config.configs;

    if fields.len() < 2 {
        return Vec::new();
    }

    // ---------------------------------------------------------
    // Extract geometry 
    // ---------------------------------------------------------

    let left_x = 1.5;
    let right_x = 23.0;

    let center_x = 12.5;
    let center_y = 12.5;

    let bottom_y = 1.5;
    let top_y = 23.5;

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
                Pos2::new(center_x, 7.5),
                Pos2::new(center_x, 17.5),
            ],
        },

        StationLayout {
            name: "tight_center".to_string(),
            stations: vec![
                Pos2::new(center_x, 11.5),
                Pos2::new(center_x, 13.5),
            ],
        },
    ]
}

pub fn evaluate_station_layouts(
    layouts: &[StationLayout],
) -> SpecialistLayoutResults {

    let scene_config: SceneConfig =
        load_json_or_panic(
            DEFAULT_SCENE_CONFIG_PATH.to_string()
        );

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
                &scene_config
        );

        evaluated_layouts.push(
            EvaluatedLayout {
                layout: layout.clone(),

                energy_wh: result.energy,
                time_sec: result.runtime_sec,
                total_distance_m: result.total_distance,
                charging_distance_m: result.charging_distance,
            }
        );
    }

    let best_layout = evaluated_layouts
        .iter()
        .min_by(|a, b| {
            a.energy_wh
                .partial_cmp(&b.energy_wh)
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