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
            name: "anti_diagonal_corners".to_string(),
            // Mirror of `diagonal_corners` across the field's other
            // diagonal (top-left/bottom-right instead of bottom-left/
            // top-right). Replaces the old "split_center" heuristic, which
            // placed a station at the center of each row/point group's own
            // bounding box -- for any field with real 2-group substructure
            // (field4, every vineyard size) that center sits deep inside
            // the group's own rows, i.e. inside the same padded bounding
            // rectangle `is_position_valid` (optimization/geometry.rs)
            // treats as off-limits for a real station. Both corners here
            // are the same padded-bbox corners `diagonal_corners` already
            // uses, so they're outside every group by the same construction
            // (2026-09-02).
            stations: vec![
                Pos2::new(left_x, top_y),
                Pos2::new(right_x, bottom_y),
            ],
        },

        StationLayout {
            name: "tight_center".to_string(),
            stations: match group_gap(field_config) {
                // Real headland gap between 2 groups (field4, vineyard):
                // put both stations inside it, not at the combined bbox's
                // center -- which, when the 2 groups have unequal
                // height/width, sits off-center from the actual gap and can
                // push one station back into a row block.
                Some(GroupGap { along_y: true, lo, hi }) => {
                    let center = (lo + hi) / 2.0;
                    vec![
                        Pos2::new(center_x, center - TIGHT_CENTER_GAP / 2.0),
                        Pos2::new(center_x, center + TIGHT_CENTER_GAP / 2.0),
                    ]
                }
                Some(GroupGap { along_y: false, lo, hi }) => {
                    let center = (lo + hi) / 2.0;
                    vec![
                        Pos2::new(center - TIGHT_CENTER_GAP / 2.0, center_y),
                        Pos2::new(center + TIGHT_CENTER_GAP / 2.0, center_y),
                    ]
                }
                None => vec![
                    Pos2::new(center_x, center_y - TIGHT_CENTER_GAP / 2.0),
                    Pos2::new(center_x, center_y + TIGHT_CENTER_GAP / 2.0),
                ],
            },
        },

        StationLayout {
            name: "task_centroid".to_string(),
            stations: task_centroid_stations(field_config),
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

/// Two stations straddling the task centroid along the axis that separates
/// the field's row/point groups (the same axis `tight_center` centers its
/// own pair on via `group_gap`), clamped to stay within the actual gap
/// between those groups. The raw centroid can sit right at, or just inside,
/// one group's row block when the two groups are unevenly sized/shaped
/// (e.g. the vineyard fields' unequal row counts) -- clamping the *pair*
/// (shifting both stations together rather than each independently) keeps
/// them `TIGHT_CENTER_GAP` apart and preserves the centroid's cross-axis
/// coordinate, only correcting the along-gap one (2026-09-02).
fn task_centroid_stations(field_config: &FieldConfig) -> Vec<Pos2> {
    let centroid = task_centroid(field_config);

    match group_gap(field_config) {
        Some(GroupGap { along_y: true, lo, hi }) => {
            let (y0, y1) = clamp_pair(centroid.y, lo, hi);
            vec![Pos2::new(centroid.x, y0), Pos2::new(centroid.x, y1)]
        }
        Some(GroupGap { along_y: false, lo, hi }) => {
            let (x0, x1) = clamp_pair(centroid.x, lo, hi);
            vec![Pos2::new(x0, centroid.y), Pos2::new(x1, centroid.y)]
        }
        // No 2-group substructure to clamp against (single homogeneous
        // block, or more than 2 groups) -- fall back to the raw centroid,
        // same as before this fix.
        None => vec![
            Pos2::new(centroid.x, centroid.y - TIGHT_CENTER_GAP / 2.0),
            Pos2::new(centroid.x, centroid.y + TIGHT_CENTER_GAP / 2.0),
        ],
    }
}

/// Minimum clearance a clamped station is pushed past a row block's edge --
/// small enough not to eat into a narrow gap (the vineyard "small" field's
/// gap is only 2.5 m against `TIGHT_CENTER_GAP`'s 2 m), but enough that the
/// clamped point clears `is_inside_field_bounds`'s inclusive `<=`/`>=`
/// comparison rather than landing exactly on it.
const GAP_CLEARANCE: f32 = 0.1;

/// A pair of points `TIGHT_CENTER_GAP` apart, centered on `raw_center`
/// where that fits, otherwise shifted together (preserving the gap) to fit
/// within `(lo + GAP_CLEARANCE, hi - GAP_CLEARANCE)`.
fn clamp_pair(raw_center: f32, lo: f32, hi: f32) -> (f32, f32) {
    let half = TIGHT_CENTER_GAP / 2.0;
    let lo_bound = lo + GAP_CLEARANCE;
    let hi_bound = hi - GAP_CLEARANCE;

    let mut a = raw_center - half;
    let mut b = raw_center + half;

    if a < lo_bound {
        let shift = lo_bound - a;
        a += shift;
        b += shift;
    }
    if b > hi_bound {
        let shift = b - hi_bound;
        a -= shift;
        b -= shift;
    }

    (a, b)
}

/// A real gap between two field row/point groups -- e.g. the headland space
/// between field4's two row blocks, or between a vineyard field's two
/// perpendicular blocks -- found along whichever axis separates the groups
/// most clearly.
struct GroupGap {
    /// `true`: groups are separated along y (gap is a horizontal band).
    /// `false`: separated along x (gap is a vertical band).
    along_y: bool,
    /// Near edge of the empty space between the two nearest groups, along
    /// the gap axis.
    lo: f32,
    /// Far edge of that empty space.
    hi: f32,
}

/// `None` for fields that aren't exactly 2 groups -- either a single
/// homogeneous block (no substructure to find a gap in) or more groups than
/// this simple nearest-pair search is meant to handle.
fn group_gap(field_config: &FieldConfig) -> Option<GroupGap> {
    let groups = FieldBounds::per_group_from_field_config(field_config);
    if groups.len() != 2 {
        return None;
    }

    fn gap(mut ranges: Vec<(f32, f32)>) -> Option<(f32, f32)> {
        ranges.sort_by(|a, b| a.0.partial_cmp(&b.0).unwrap());
        let a_max = ranges[0].1;
        let b_min = ranges[1].0;
        (b_min > a_max).then_some((a_max, b_min))
    }

    let y_gap = gap(groups.iter().map(|g| (g.min_y, g.max_y)).collect());
    let x_gap = gap(groups.iter().map(|g| (g.min_x, g.max_x)).collect());

    match (y_gap, x_gap) {
        (Some((ylo, yhi)), Some((xlo, xhi))) => Some(if (yhi - ylo) >= (xhi - xlo) {
            GroupGap { along_y: true, lo: ylo, hi: yhi }
        } else {
            GroupGap { along_y: false, lo: xlo, hi: xhi }
        }),
        (Some((ylo, yhi)), None) => Some(GroupGap { along_y: true, lo: ylo, hi: yhi }),
        (None, Some((xlo, xhi))) => Some(GroupGap { along_y: false, lo: xlo, hi: xhi }),
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

#[cfg(test)]
mod tests {
    use super::*;
    use crate::experiment::geometry::is_inside_field_bounds;
    use crate::utilities::utils::load_json_or_panic;

    /// Every specialist layout's stations must fall outside every field
    /// group's own bounding box -- the same rule EGO/grid-search enforce
    /// (`is_position_valid`, checked against `per_group_from_field_config`
    /// bounds) for a candidate station position to be physically valid
    /// (not inside the rows themselves).
    #[test]
    fn specialist_layouts_avoid_row_blocks() {
        for path in [
            "configs/field_configs/vineyard/small.json",
            "configs/field_configs/vineyard/medium.json",
            "configs/field_configs/vineyard/large.json",
            "configs/field_configs/vineyard/xlarge.json",
            "configs/field_configs/legacy/field4.json",
        ] {
            let field_config: FieldConfig = load_json_or_panic(path.to_string());
            let groups = FieldBounds::per_group_from_field_config(&field_config);

            for layout in specialist_layouts(&field_config) {
                for (i, s) in layout.stations.iter().enumerate() {
                    assert!(
                        !is_inside_field_bounds(*s, &groups),
                        "{path}: layout '{}' station {i} ({:.2}, {:.2}) falls inside a row block",
                        layout.name, s.x, s.y,
                    );
                }
            }
        }
    }
}