use crate::experiment::runner::run_grid_search_experiment;
use crate::ExperimentConfig;

pub fn run_soc_sweep(output_dir: &str) {

    let thresholds = vec![50.0, 60.0, 70.0, 80.0];
    let seeds = [0];
    let resolution = 10;
    
    println!("\n===== EXPERIMENT: SoC threshold sweep ===================================");

    for threshold in thresholds {

        for seed in seeds.clone() {
            let exp = ExperimentConfig {
                seed,
                n_agents: 3,
                soc_threshold_percent: threshold as f32,
                ..Default::default()
            };

            println!("Field size     : { }", exp.field_config_path);
            println!("Fleet size     : {:.2}", exp.n_agents);
            println!("Battery        : {:.0} Wh", exp.battery_capacity_wh);
            println!("Threshold SoC  : {:.0} %", threshold);
            println!("Critical SoC   : {:.0} %", exp.critical_soc_percent);
            println!("Seed           : {:.2}", seed);
            println!("Grid resolution: {:.2}×{:.2}", resolution, resolution);

            let filename=  format!(
                "size={}_fleet={}_batt={}_soc={}",
                "M",
                exp.n_agents,
                exp.battery_capacity_wh,
                threshold,
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