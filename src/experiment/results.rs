use crate::cfg::{
    DEFAULT_SCENE_CONFIG_PATH,
};
use crate::environment::{
    scene_config::SceneConfig,
    field_config::FieldConfig,
};
use egui::Pos2;

/// Save grid search results to JSON file
pub fn save_grid_search_results(
    results: &[(Pos2, f64, f64, f64, f64)], 
    filename: &str, 
    optimization_minimum: Option<(Pos2, f64)>,
    field_config: &FieldConfig,
    grid_resolution: usize
) {
    use serde_json::{json, Value};
    
    let results_json: Vec<Value> = results.iter()
        .map(|(pos, energy, total_dist, charging_dist, time)| {
            json!({
                "x": pos.x,
                "y": pos.y,
                "energy_consumption": energy,
                "total_distance": total_dist,
                "charging_distance": charging_dist,
                "evaluation_time_sec": time
            })
        })
        .collect();
    
    // Convert optimization minimum to JSON
    let optimization_minimum_json = optimization_minimum.map(|(pos, value)| {
        json!({
            "x": pos.x,
            "y": pos.y,
            "energy_consumption": value
        })
    });
    
    // Store field config information and reference to original file
    const FIELD_MIN_X: f32 = 0.0;
    const FIELD_MAX_X: f32 = 25.0;
    const FIELD_MIN_Y: f32 = 0.0;
    const FIELD_MAX_Y: f32 = 25.0;
    
    // Load the original field config path from scene config
    let scene_config: SceneConfig = crate::utilities::utils::load_json_or_panic(DEFAULT_SCENE_CONFIG_PATH.to_string());
    let field_config_path = scene_config.field_config_path.clone();
    
    // Read the raw JSON content from the original field config file
    let field_config_raw_json = match std::fs::read_to_string(&field_config_path) {
        Ok(content) => Some(content),
        Err(_) => None,
    };
    
    let field_config_json = json!({
        "field_config_path": field_config_path,
        "field_config_raw": field_config_raw_json,
        "field_boundaries_used_in_grid_search": {
            "min_x": FIELD_MIN_X,
            "max_x": FIELD_MAX_X,
            "min_y": FIELD_MIN_Y,
            "max_y": FIELD_MAX_Y
        },
        "num_obstacles": field_config.get_obstacles().len(),
        "num_field_configs": field_config.configs.len()
    });

    let times: Vec<f64> = results.iter().map(|r| r.4).collect();

    let (avg_time, max_time, min_time, total_time) = if !times.is_empty() {
        let total: f64 = times.iter().sum();
        let avg = total / times.len() as f64;
        let max = times.iter().cloned().fold(f64::MIN, f64::max);
        let min = times.iter().cloned().fold(f64::MAX, f64::min);
        (avg, max, min, total)
    } else {
        (0.0, 0.0, 0.0, 0.0)
    };
        
    let output = json!({
        "grid_search_results": results_json,
        "total_points": results.len(),
        "grid_resolution": grid_resolution,
        "optimization_minimum": optimization_minimum_json,
        "field_config": field_config_json,
        "generated_at": chrono::Utc::now().to_rfc3339(),
        "timing": {
            "average_evaluation_time_sec": avg_time,
            "max_evaluation_time_sec": max_time,
            "min_evaluation_time_sec": min_time,
            "total_grid_search_time_sec": total_time
        }
    });
    
    // match std::fs::write(filename, serde_json::to_string_pretty(&output).unwrap()) {
    //     Ok(_) => println!("Grid search results saved to: {}", filename),
    //     Err(e) => eprintln!("Failed to save results: {}", e),
    // }

    if let Err(e) = std::fs::write(
    filename,
    serde_json::to_string_pretty(&output).unwrap(),
    ) {
        eprintln!("Failed to save results: {}", e);
    }

}