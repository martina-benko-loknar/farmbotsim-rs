use crate::experiment::geometry::generate_valid_grid_points;
use crate::experiment::evaluation::evaluate_station_layout;
use crate::experiment::models::{
    GridSearchPoint,
    GridSearchResults,
};
use crate::environment::geometry::FieldBounds;
use crate::experiment::search_domain::SearchDomain;
use crate::terrain::TerrainLoader;
use crate::experiment::geometry::vineyard_polygons;
use crate::experiment::geometry::generate_row_gap_obstacles;
use crate::experiment::config::ExperimentConfig;

// use egui::Pos2;
// use rand;

fn separator() {
    println!("{}", "-".repeat(70));
}

/// Grid search experiment with optional optimization minimum point and value for visualization
pub fn grid_search_experiment(
    grid_resolution: usize, 
    exp: &ExperimentConfig,
    // optimization_minimum: Option<(Pos2, f64)>,
    ) -> GridSearchResults 
    {
    
    // Load scene configuration to get field boundaries and obstacles
    let scene_config = exp.load_scene_config();
    let field_config = exp.load_field_config();

    //let obstacles = field_config.get_obstacles();
    let mut obstacles = field_config.get_obstacles();

    obstacles.extend(
        generate_row_gap_obstacles(
            &field_config,
            0.4,
        )
    );

    // FIELD BOUNDS
    let vineyard_bounds = FieldBounds::from_field_config(&field_config);

    let terrain_map =
        TerrainLoader::from_gps_csv("configs/scene_configs/vineyard_scene/baggy-altitude-empirical-lut.csv");

    let terrain_bounds = terrain_map.bounds();

    const VINEYARD_PADDING: f32 = 5.0;
    const STATION_MARGIN: f32 = 0.4; // Domain constraint
    const OBSTACLE_MARGIN: f32 = 0.4; // Feasibility constraint

    let domain = SearchDomain::from_bounds(
        vineyard_bounds,
        terrain_bounds,
        VINEYARD_PADDING,
        STATION_MARGIN,
    );

    //let cultivation_bounds = individual_field_bounds(&field_config);

    // Generate grid points
    let vineyards = vineyard_polygons(&field_config);

    let grid_points = generate_valid_grid_points(
        &domain,
        grid_resolution,
        &obstacles,
        OBSTACLE_MARGIN,
        &vineyards,
    );

    //separator();
    println!("Progress | Position |  Energy | Total dist | Charging dist | Time\n");
    
    // Store results for analysis
    let mut points: Vec<GridSearchPoint> = Vec::new();    
    let total_points = grid_points.len();
    let experiment_start = std::time::Instant::now();
    
    // fixed number of progress updates, independent of number of grid points
    let n_prints = 20;
    let stride = ((total_points as f64) / (n_prints as f64)).ceil() as usize;
    let stride = stride.max(1); // avoid division by zero / tiny grids

    // Run experiment for each grid point
    for (i, grid_point) in grid_points.iter().enumerate() {
     
        let start = std::time::Instant::now();

        // Update scene config with new station position
        //let (energy_consumption, total_distance, charging_distance) = run_single_grid_experiment(*grid_point, &scene_config);
        
        let evaluation = evaluate_station_layout(
            &[*grid_point], 
            &scene_config,
            &exp
        );

        let energy_consumption = evaluation.energy;
        let total_distance = evaluation.total_distance;
        let charging_distance = evaluation.charging_distance;
        let elapsed = start.elapsed().as_secs_f64();
        let sim_time = evaluation.simulation_time_sec;
        let charging_events = evaluation.charging_events;

        points.push(GridSearchPoint {
            position: *grid_point,
            energy: energy_consumption,
            total_distance,
            charging_distance,
            runtime_sec: elapsed,
            simulation_time_sec: sim_time,
            number_of_charging_events: charging_events, 
        });
        
        let progress = ((i + 1) as f64 / total_points as f64) * 100.0;

        let progress_str = if progress >= 100.0 {
            format!("{:>4.0}", progress)  // no decimals
        } else {
            format!("{:>4.1}", progress)  // one decimal
        };

        //let width = total_points.to_string().len(); // number of digits in total_points

        if i % stride == 0 || i + 1 == total_points {
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

    }

    let runtime_sec = experiment_start.elapsed().as_secs_f64();

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
    println!("Summary: ");
    println!("Valid pts/Total pts : {}/{}", 
             grid_points.len(), grid_resolution * grid_resolution);
    println!("Best point: "); 
    println!("  - position        : ({:.2} m, {:.2} m)", best_point.position.x, best_point.position.y);
    println!("  - energy          : {:.2} kWh", best_point.energy/1000.0);
    println!("  - distance        : {:.2} km", best_point.total_distance/1000.0);
    println!("  - charging dist   : {:.2} km", best_point.charging_distance/1000.0);
    println!("  - mission time    : {:.2} h", best_point.simulation_time_sec/3600.0);
    println!("  - charging events : {:.2}", best_point.number_of_charging_events);

    println!("Experiment time     : {:.2} s",  runtime_sec);
    // println!("Results saved as    : {}", results_file);

    GridSearchResults {
        points,
        grid_resolution,
        //optimization_minimum,
        valid_points: grid_points.len(),
        total_points: grid_resolution * grid_resolution,
        best_point,
        experiment_configs: exp.clone(),
    }
    // println!("Output directory: {}", output_dir);
}