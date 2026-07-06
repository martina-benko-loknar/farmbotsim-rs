use std::{any, error::Error, fs::{self, File}, io::Write, path::Path};
use egui::Pos2;
use serde::Serialize;
use serde_json::to_string_pretty;

use crate::logger::log_error_and_panic;
use crate::environment::scene_config::SceneConfig;
use crate::cfg::DEFAULT_SCENE_CONFIG_PATH;

/// Converts hsv to rgb
fn hsv_to_rgb(h: f32, s: f32, v: f32) -> (u8, u8, u8) {
    let h = (h % 1.0) * 6.0; // Hue in [0, 6)
    let f = h - h.floor();
    let p = v * (1.0 - s);
    let q = v * (1.0 - f * s);
    let t = v * (1.0 - (1.0 - f) * s);

    let (r, g, b) = match h as u8 {
        0 => (v, t, p),
        1 => (q, v, p),
        2 => (p, v, t),
        3 => (p, q, v),
        4 => (t, p, v),
        5 => (v, p, q),
        _ => (0.0, 0.0, 0.0), // This shouldn't happen due to the modulo
    };

    // Convert to 0-255 range
    (
        (r * 255.0) as u8,
        (g * 255.0) as u8,
        (b * 255.0) as u8,
    )
}

/// Generates n amount of colors with hue offset.
pub fn generate_colors(n: usize, hue_offset: f32) -> Vec<egui::Color32> {
    let mut colors = Vec::with_capacity(n);

    for i in 0..n {
        let hue = (i as f32 / n as f32 + hue_offset) % 1.0;
        let (r, g, b) = hsv_to_rgb(hue, 1.0, 1.0); // Full saturation and value
        colors.push(egui::Color32::from_rgb(r, g, b)); // Convert to Color32
    }

    colors
}

/// Get json files that are in folder with path.
pub fn get_json_files_in_folder(path: &str) -> Vec<String> {
    let mut json_files = Vec::new();

    let entries = fs::read_dir(path).unwrap_or_else(|e| {
        let msg = format!("Failed to read directory {}: {}", path, e);
        log_error_and_panic(&msg);
    });

    for entry in entries {
        let entry = entry.unwrap_or_else(|e| {
            let msg = format!("Failed to read entry in {}: {}", path, e);
            log_error_and_panic(&msg);
        });

        let path = entry.path();
        if path.is_file() && path.extension().and_then(|s| s.to_str()) == Some("json") {
            if let Some(file_name) = path.file_name().and_then(|s| s.to_str()) {
                json_files.push(file_name.to_string());
            }
        }
    }

    json_files
}

/// Get folders that are in folder with path.
pub fn get_folders_in_folder(path: &str) -> Vec<String> {
    let mut folders = Vec::new();

    let entries = fs::read_dir(path).unwrap_or_else(|e| {
        let msg = format!("Failed to read directory {}: {}", path, e);
        log_error_and_panic(&msg);
    });

    for entry in entries {
        let entry = entry.unwrap_or_else(|e| {
            let msg = format!("Error reading entry in {}: {}", path, e);
            log_error_and_panic(&msg);
        });

        let path = entry.path();
        if path.is_dir() {
            if let Some(folder_name) = path.file_name().and_then(|s| s.to_str()) {
                folders.push(folder_name.to_string());
            }
        }
    }

    folders
}

pub fn load_scene_config(
    field_config_path: String,
) -> SceneConfig {

    let mut scene: SceneConfig =
        load_json_or_panic(DEFAULT_SCENE_CONFIG_PATH.to_string());

    scene.field_config_path = field_config_path;

    scene
}

/// Loads json or panics on failure.
pub fn load_json_or_panic<T, P>(path: P) -> T
where
    T: serde::de::DeserializeOwned,
    P: AsRef<Path>,
{
    let path_ref = path.as_ref();

    // Read the file contents, or panic with logging if it fails
    let data = fs::read_to_string(path_ref).unwrap_or_else(|e| {
        let type_name = any::type_name::<T>();
        let msg = format!("Failed to read file {:?} for {}: {}", path_ref, type_name, e);
        log_error_and_panic(&msg);
    });

    // Parse JSON, or panic with logging if it fails
    serde_json::from_str(&data).unwrap_or_else(|e| {
        let type_name = any::type_name::<T>();
        let msg = format!("Failed to parse JSON from {:?} into {}: {}", path_ref, type_name, e);
        log_error_and_panic(&msg);
    })
}

/// Saves data to file with path.
pub fn save_as_json<T: Serialize>(data: &T, file_path: &str) -> Result<(), Box<dyn Error>> {
    let json = to_string_pretty(data)?;

    let mut file = File::create(file_path)?;
    file.write_all(json.as_bytes())?;

    println!("Successfully saved to {}", file_path);
    Ok(())
}

/// Renders dropdown of all json config files in folder path.
pub fn json_config_combo(
    ui: &mut egui::Ui,
    id_salt: &str,
    current_value: &mut String,
    folder_path: &str,
) -> bool {
    let mut changed = false;

    egui::ComboBox::from_id_salt(id_salt)
        .selected_text(format!("{:?}", current_value))
        .show_ui(ui, |ui| {
            let json_files = get_json_files_in_folder(folder_path);
            let previous_value = current_value.clone();

            for json_file in json_files {
                let new_value = format!("{}{}", folder_path, json_file);
                ui.selectable_value(current_value, new_value.clone(), json_file);
            }

            if *current_value != previous_value {
                changed = true;
            }
        });

    changed
}

/// Renders dropdown of all folders in folder path.
pub fn folder_select_combo(
    ui: &mut egui::Ui,
    id_salt: &str,
    current_value: &mut String,
    base_path: &str
) -> bool {
    let mut changed = false;

    egui::ComboBox::from_id_salt(id_salt)
        .selected_text(current_value.as_str())
        .show_ui(ui, |ui| {
            let options = get_folders_in_folder(base_path);
            let previous_value = current_value.clone();

            for option in options {
                let full_path = format!("{}{}", base_path, option);
                ui.selectable_value(current_value, full_path.clone(), full_path);
            }

            if *current_value != previous_value {
                changed = true;
            }
        });

    changed
}

/// Renders adjustable drag value and dropdown for unit.
pub fn value_with_unit_selector_ui<T: ToString + PartialEq + Copy + enum_iterator::Sequence>(
    ui: &mut egui::Ui,
    id_salt: &str,
    label: &str,
    value: &mut f32,
    unit: &mut T,
    min_value: Option<f32>,
    max_value: Option<f32>,
) {
    ui.horizontal(|ui| {
        ui.label(format!("        \"{}\":", label));
        ui.add(egui::DragValue::new(value).speed(0.1));
        if let Some(min) = min_value {
            if *value < min { *value = min; }
        }
        if let Some(max) = max_value {
            if *value > max { *value = max; }
        }

        egui::ComboBox::from_id_salt(id_salt)
            .selected_text(unit.to_string())
            .show_ui(ui, |ui| {
                for u in enum_iterator::all::<T>() {
                    ui.selectable_value(unit, u, u.to_string());
                }
            });
    });
}

/// Generates a set of `n` positions along a straight line with specified spacing and angle.
pub fn line_positions(n: usize, spacing: f32, angle: f32) -> Vec<Pos2> {
    let mut positions = Vec::with_capacity(n);

    let cos_a = angle.cos();
    let sin_a = angle.sin();

    if n == 1 {
        positions.push(Pos2::new(0.0, 0.0));
    } else if n % 2 == 1 {
        let mid = (n / 2) as isize;
        for i in 0..n {
            let x = (i as isize - mid) as f32 * spacing;
            // Rotate (x,0) by angle: (x*cos - 0*sin, x*sin + 0*cos) = (x*cos, x*sin)
            let pos = Pos2::new(x * cos_a, x * sin_a);
            positions.push(pos);
        }
    } else {
        let mid = (n / 2) as f32;
        for i in 0..n {
            let x = (i as f32 - mid + 0.5) * spacing;
            let pos = Pos2::new(x * cos_a, x * sin_a);
            positions.push(pos);
        }
    }

    positions
}

/// Performs linear interpolation between two points.
pub fn linear_interpolate(x0: f32, y0: f32, x1: f32, y1: f32, x: f32) -> f32 {
    if x1 == x0 {
        return y0;
    }
    y0 + (x - x0) * (y1 - y0) / (x1 - x0)
}
