use crate::cfg::DEFAULT_SCENE_CONFIG_PATH;
use crate::environment::{
    scene_config::SceneConfig,
    field_config::FieldConfig,
};
use crate::experiment::models::{
    GridSearchPoint,
    GridSearchResults,
};
//use egui::Pos2;
use serde_json::{json, Value};

/// Save grid search results to JSON file
pub fn save_grid_search_results(
    results: &GridSearchResults, 
    output_dir: &str
) {

    // ---------------------------------------------------------
    // Create output path
    // ---------------------------------------------------------

    let filename = format!(
        "{}/grid_search_{}x{}_results.json",
        output_dir,
        results.grid_resolution,
        results.grid_resolution,
    );
    
    // ---------------------------------------------------------
    // Convert points to JSON
    // ---------------------------------------------------------

    let results_json: Vec<Value> = results
        .points
        .iter()
        .map(|point: &GridSearchPoint| {
            json!({
                "x": point.position.x,
                "y": point.position.y,
                "energy_consumption": point.energy,
                "total_distance": point.total_distance,
                "charging_distance": point.charging_distance,
                "evaluation_time_sec": point.runtime_sec
            })
        })
        .collect();
    
    // // ---------------------------------------------------------
    // // Optimization minimum
    // // ---------------------------------------------------------

    // let optimization_minimum_json =
    //     results.optimization_minimum.map(|(pos, value)| {
    //         json!({
    //             "x": pos.x,
    //             "y": pos.y,
    //             "energy_consumption": value
    //         })
    //     });
    
    // ---------------------------------------------------------
    // Load field config metadata
    // ---------------------------------------------------------

    const FIELD_MIN_X: f32 = 0.0;
    const FIELD_MAX_X: f32 = 25.0;
    const FIELD_MIN_Y: f32 = 0.0;
    const FIELD_MAX_Y: f32 = 25.0;

    let scene_config: SceneConfig =
        crate::utilities::utils::load_json_or_panic(
            DEFAULT_SCENE_CONFIG_PATH.to_string(),
        );

    let field_config_path =
        scene_config.field_config_path.clone();

    let field_config: FieldConfig =
        crate::utilities::utils::load_json_or_panic(
            field_config_path.clone(),
        );

    let field_config_raw_json =
        std::fs::read_to_string(&field_config_path).ok();

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

    // ---------------------------------------------------------
    // Timing statistics
    // ---------------------------------------------------------

    let times: Vec<f64> = results
        .points
        .iter()
        .map(|p| p.runtime_sec)
        .collect();

    let (avg_time, max_time, min_time, total_time) =
        if !times.is_empty() {
            let total: f64 = times.iter().sum();

            let avg = total / times.len() as f64;

            let max =
                times.iter().cloned().fold(f64::MIN, f64::max);

            let min =
                times.iter().cloned().fold(f64::MAX, f64::min);

            (avg, max, min, total)
        } else {
            (0.0, 0.0, 0.0, 0.0)
        };
      
    // ---------------------------------------------------------
    // Final output JSON
    // ---------------------------------------------------------

    let output = json!({
        "grid_search_results": results_json,

        "summary": {
            "valid_points": results.valid_points,
            "total_points": results.total_points,
            "grid_resolution": results.grid_resolution,

            "best_point": {
                "x": results.best_point.position.x,
                "y": results.best_point.position.y,
                "energy_consumption": results.best_point.energy,
                "total_distance": results.best_point.total_distance,
                "charging_distance": results.best_point.charging_distance,
                "runtime_sec": results.best_point.runtime_sec
            }
        },

        //"optimization_minimum": optimization_minimum_json,

        "field_config": field_config_json,

        "generated_at": chrono::Utc::now().to_rfc3339(),

        "timing": {
            "average_evaluation_time_sec": avg_time,
            "max_evaluation_time_sec": max_time,
            "min_evaluation_time_sec": min_time,
            "total_grid_search_time_sec": total_time
        }
    });

    // ---------------------------------------------------------
    // Create output directory
    // ---------------------------------------------------------

    if let Err(e) = std::fs::create_dir_all(output_dir) {
        eprintln!("Failed to create output directory: {}", e);
        return;
    }

    // ---------------------------------------------------------
    // Save file
    // ---------------------------------------------------------

    match std::fs::write(
        &filename,
        serde_json::to_string_pretty(&output).unwrap(),
    ) {
        Ok(_) => {
            println!("Grid search results saved to:");
            println!("{}", filename);
        }

        Err(e) => {
            eprintln!("Failed to save results: {}", e);
        }
    }

}