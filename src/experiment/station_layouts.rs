use egui::Pos2;

use crate::{environment::field_config::FieldConfig, experiment::models::{
    EvaluatedLayout, ExperimentMetrics, SpecialistLayoutResults, StationLayout
}};
use crate::experiment::evaluation::evaluate_station_layout;
use crate::environment::geometry::FieldBounds;
use crate::environment::farm_entity_module::farm_entity::FarmEntity;
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
            stations: {
                // Fields with exactly 2 row/point groups (e.g. field4, all
                // vineyard sizes) have real substructure -- one station per
                // group's own center is a meaningful "one station per
                // sub-field" placement, and robust to the groups having
                // different heights/widths (unlike splitting the *combined*
                // bbox, which skews toward whichever group is larger).
                // Single-group fields (e.g. field1/field2) have no such
                // substructure, so fall back to the old quarter-height split
                // of the overall bbox -- arbitrary either way, but at least
                // spreads the two stations apart within the one block.
                let groups = FieldBounds::per_group_from_field_config(field_config);
                if groups.len() == 2 {
                    vec![
                        Pos2::new(
                            (groups[0].min_x + groups[0].max_x) / 2.0,
                            (groups[0].min_y + groups[0].max_y) / 2.0,
                        ),
                        Pos2::new(
                            (groups[1].min_x + groups[1].max_x) / 2.0,
                            (groups[1].min_y + groups[1].max_y) / 2.0,
                        ),
                    ]
                } else {
                    vec![
                        Pos2::new(center_x, bottom_y + 0.25 * height),
                        Pos2::new(center_x, bottom_y + 0.75 * height),
                    ]
                }
            },
        },

        StationLayout {
            name: "tight_center".to_string(),
            stations: match field_gap(field_config) {
                // Real headland gap between 2 groups (field4, vineyard):
                // put both stations inside it, not at the combined bbox's
                // center -- which, when the 2 groups have unequal
                // height/width, sits off-center from the actual gap and can
                // push one station back into a row block.
                Some(FieldGap { along_y: true, center }) => vec![
                    Pos2::new(center_x, center - TIGHT_CENTER_GAP / 2.0),
                    Pos2::new(center_x, center + TIGHT_CENTER_GAP / 2.0),
                ],
                Some(FieldGap { along_y: false, center }) => vec![
                    Pos2::new(center - TIGHT_CENTER_GAP / 2.0, center_y),
                    Pos2::new(center + TIGHT_CENTER_GAP / 2.0, center_y),
                ],
                None => vec![
                    Pos2::new(center_x, center_y - TIGHT_CENTER_GAP / 2.0),
                    Pos2::new(center_x, center_y + TIGHT_CENTER_GAP / 2.0),
                ],
            },
        },

        StationLayout {
            name: "task_centroid".to_string(),
            stations: {
                let Pos2 { x, y } = task_centroid(field_config);
                vec![
                    Pos2::new(x, y - TIGHT_CENTER_GAP / 2.0),
                    Pos2::new(x, y + TIGHT_CENTER_GAP / 2.0),
                ]
            },
        },
    ]
}

/// Center of mass of all task locations (crop points, and row midpoints for
/// line-based tasks) in the field, unweighted by task duration/frequency.
/// Reflects "where the work actually is" rather than the field's bounding
/// box, in contrast to the other, geometry-only layouts above.
fn task_centroid(field_config: &FieldConfig) -> Pos2 {
    let mut sum_x = 0.0f32;
    let mut sum_y = 0.0f32;
    let mut n = 0.0f32;

    for entity in field_config.get_farm_entities().values() {
        let p = match entity {
            FarmEntity::Crop(crop) => crop.position,
            FarmEntity::Row(row) => {
                let (sx, sy) = row
                    .path
                    .iter()
                    .fold((0.0, 0.0), |(ax, ay), p| (ax + p.x, ay + p.y));
                Pos2::new(sx / row.path.len() as f32, sy / row.path.len() as f32)
            }
        };
        sum_x += p.x;
        sum_y += p.y;
        n += 1.0;
    }

    Pos2::new(sum_x / n, sum_y / n)
}

/// A real gap between two field row/point groups -- e.g. the headland space
/// between field4's two row blocks, or between a vineyard field's two
/// perpendicular blocks -- found along whichever axis separates the groups
/// most clearly.
struct FieldGap {
    /// `true`: groups are separated along y (gap is a horizontal band).
    /// `false`: separated along x (gap is a vertical band).
    along_y: bool,
    /// Coordinate at the middle of the empty space between the two nearest
    /// groups along the gap axis.
    center: f32,
}

/// `None` for fields that aren't exactly 2 groups -- either a single
/// homogeneous block (no substructure to find a gap in) or more groups than
/// this simple nearest-pair search is meant to handle.
fn field_gap(field_config: &FieldConfig) -> Option<FieldGap> {
    let groups = FieldBounds::per_group_from_field_config(field_config);
    if groups.len() != 2 {
        return None;
    }

    fn gap(mut ranges: Vec<(f32, f32)>) -> Option<(f32, f32)> {
        ranges.sort_by(|a, b| a.0.partial_cmp(&b.0).unwrap());
        let a_max = ranges[0].1;
        let b_min = ranges[1].0;
        (b_min > a_max).then_some((b_min - a_max, (a_max + b_min) / 2.0))
    }

    let y_gap = gap(groups.iter().map(|g| (g.min_y, g.max_y)).collect());
    let x_gap = gap(groups.iter().map(|g| (g.min_x, g.max_x)).collect());

    match (y_gap, x_gap) {
        (Some((yg, yc)), Some((xg, xc))) => Some(if yg >= xg {
            FieldGap { along_y: true, center: yc }
        } else {
            FieldGap { along_y: false, center: xc }
        }),
        (Some((_, yc)), None) => Some(FieldGap { along_y: true, center: yc }),
        (None, Some((_, xc))) => Some(FieldGap { along_y: false, center: xc }),
        (None, None) => None,
    }
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