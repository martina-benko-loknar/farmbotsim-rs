use crate::cfg::DEFAULT_SCENE_CONFIG_PATH;
use crate::environment::{
    scene_config::SceneConfig,
    field_config::FieldConfig,
};
use crate::experiment::geometry::generate_valid_grid_points;
use crate::experiment::evaluation::evaluate_station_layout;
use crate::experiment::models::{
    GridSearchPoint,
    GridSearchResults,
};
use crate::utilities::utils::load_json_or_panic;
// use egui::Pos2;
// use rand;

fn separator() {
    println!("{}", "-".repeat(70));
}

/// Grid search experiment with optional optimization minimum point and value for visualization
pub fn grid_search_experiment(
    grid_resolution: usize, 
    // optimization_minimum: Option<(Pos2, f64)>,
    ) -> GridSearchResults 
    {
    
    // Load scene configuration to get field boundaries and obstacles
    let scene_config: SceneConfig = load_json_or_panic(DEFAULT_SCENE_CONFIG_PATH.to_string());
    
    let field_config: FieldConfig = crate::utilities::utils::load_json_or_panic(scene_config.field_config_path.clone());
    let obstacles = field_config.get_obstacles();
    
    // Define field boundaries (should match those in optimization.rs)
    const FIELD_MIN_X: f32 = 0.0;
    const FIELD_MAX_X: f32 = 25.0;
    const FIELD_MIN_Y: f32 = 0.0;
    const FIELD_MAX_Y: f32 = 25.0;
    const STATION_MARGIN: f32 = 0.4;
    const OBSTACLE_MARGIN: f32 = 0.4;
    
    // Generate grid points
    let grid_points = generate_valid_grid_points(
        FIELD_MIN_X + STATION_MARGIN,
        FIELD_MAX_X - STATION_MARGIN,
        FIELD_MIN_Y + STATION_MARGIN,
        FIELD_MAX_Y - STATION_MARGIN,
        grid_resolution,
        &obstacles,
        OBSTACLE_MARGIN,
    );

    //separator();
    println!("Progress | Position |  Energy | Total dist | Charging dist | Time\n");
    
    // Store results for analysis
    let mut points: Vec<GridSearchPoint> = Vec::new();    
    let total_points = grid_points.len();
    
    // Run experiment for each grid point
    for (i, grid_point) in grid_points.iter().enumerate() {
     
        let start = std::time::Instant::now();

        // Update scene config with new station position
        //let (energy_consumption, total_distance, charging_distance) = run_single_grid_experiment(*grid_point, &scene_config);
        let evaluation = evaluate_station_layout(
            &[*grid_point], 
            &scene_config);

        let energy_consumption = evaluation.energy;
        let total_distance = evaluation.total_distance;
        let charging_distance = evaluation.charging_distance;
        let elapsed = start.elapsed().as_secs_f64();

        points.push(GridSearchPoint {
            position: *grid_point,
            energy: energy_consumption,
            total_distance,
            charging_distance,
            runtime_sec: elapsed,
        });
        
        let progress = ((i + 1) as f64 / total_points as f64) * 100.0;

        let progress_str = if progress >= 100.0 {
            format!("{:>4.0}", progress)  // no decimals
        } else {
            format!("{:>4.1}", progress)  // one decimal
        };

        //let width = total_points.to_string().len(); // number of digits in total_points

        println!(
            "[{}%] | ({:>5.2},{:>5.2}) | {:.2} kWh | {:.2} km | {:.2} km | {:.2}s",
            progress_str, // progress
            // i + 1,
            // total_points, //({:>width$}/{})
            grid_point.x,
            grid_point.y,
            energy_consumption/1000.0,
            total_distance/1000.0,
            charging_distance/1000.0,
            elapsed
        );

    }
    
    // Find and report best position (based on energy consumption)
    let best_point = points
        .iter()
        .min_by(|a, b| {
            a.energy
                .partial_cmp(&b.energy)
                .unwrap()
        })
        .unwrap()
        .clone();
    
    separator();
    println!("Summary");
    println!("\nValid pts/Total pts : {}/{}", 
             grid_points.len(), grid_resolution * grid_resolution);
    println!("Best position       : ({:.2}, {:.2})", best_point.position.x, best_point.position.y);
    println!("Energy              : {:.2} kWh", best_point.energy/1000.0);
    println!("Distance            : {:.2} km", best_point.total_distance/1000.0);
    println!("Charging dist       : {:.2} km", best_point.charging_distance/1000.0);
    // println!("Results saved as    : {}", results_file);

    GridSearchResults {
        points,
        grid_resolution,
        //optimization_minimum,
        valid_points: grid_points.len(),
        total_points: grid_resolution * grid_resolution,
        best_point,
    }
    // println!("Output directory: {}", output_dir);
}