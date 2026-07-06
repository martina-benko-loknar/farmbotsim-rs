use crate::experiment::runner::{run_grid_search_experiment};
use crate::experiment::config::ExperimentConfig;

pub fn run_battery_sweep(output_dir: &str) {

    let capacities = vec![250.0, 300.0, 350.0, 400.0, 450.0, 500.0];
    let seeds = [0];
    let resolution = 10;

    println!("\n===== EXPERIMENT: Battery capacity sweep ===================================");

    for battery_capacity_wh in capacities {

        for seed in seeds.clone() {

            let exp = ExperimentConfig {
                seed,
                battery_capacity_wh: battery_capacity_wh as f32,
                ..Default::default()
            };

            println!("Field size     : { }", exp.field_config_path);
            println!("Fleet size     : {:.2}", exp.n_agents);
            println!("Battery        : {:.2} Wh", battery_capacity_wh);
            println!("Threshold SoC  : {:.0} %", exp.soc_threshold_percent);
            println!("Critical SoC   : {:.0} %", exp.critical_soc_percent);
            println!("Seed           : {:.2}", seed);
            println!("Grid resolution: {:.2}×{:.2}", resolution, resolution);

            let filename=  format!(
                "size={}_fleet={}_batt={}_soc={}",
                "M",
                exp.n_agents,
                battery_capacity_wh,
                exp.soc_threshold_percent,
            );

            run_grid_search_experiment(
                resolution,
                &filename,
                output_dir,
                exp
            );

            println!("======================================================================\n");
        }
    }
}