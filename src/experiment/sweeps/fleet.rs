use crate::experiment::runner::{run_grid_search_experiment};
use crate::experiment::config::ExperimentConfig;

pub fn run_fleet_sweep(output_dir: &str) {

    let fleet_sizes = vec![1, 2, 3, 4];
    let seeds = [0];
    let resolution = 10;

    println!("\n===== EXPERIMENT: Fleet size sweep ===================================");

    for n_agents in fleet_sizes {

        for seed in seeds.clone() {

            let exp = ExperimentConfig {
                seed,
                n_agents: n_agents as u32,
                ..Default::default()
            };

            println!("Field size     : { }", exp.field_config_path);
            println!("Fleet size     : {:.2}", n_agents);
            println!("Battery        : {:.2} Wh", exp.battery_capacity_wh);
            println!("Threshold SoC  : {:.0} %", exp.soc_threshold_percent);
            println!("Critical SoC   : {:.0} %", exp.critical_soc_percent);
            println!("Seed           : {:.2}", seed);
            println!("Grid resolution: {:.2}×{:.2}", resolution, resolution);

            let filename=  format!(
                "size={}_fleet={}_batt={}_soc={}",
                "M",
                exp.n_agents,
                exp.battery_capacity_wh,
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