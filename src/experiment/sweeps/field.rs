use crate::experiment::runner::{run_grid_search_experiment};
use crate::ExperimentConfig;

pub fn run_field_size_sweep(output_dir: &str) {

    let field_sizes = vec![
        ("S", "configs/field_configs/vineyard/small.json"),
        ("M", "configs/field_configs/vineyard/medium.json"),
        ("L", "configs/field_configs/vineyard/large.json"),
        ("XL", "configs/field_configs/vineyard/xlarge.json"),
    ];

    let seeds = [0];
    let resolution = 10;

    println!("\n===== EXPERIMENT: Field size sweep ===================================");

    for (label, path) in field_sizes {

        for seed in seeds.clone() {

            let exp = ExperimentConfig {
                seed,
                field_config_path: path.to_string(),
                ..Default::default()
            };

            println!("Field size     : {label}");
            println!("Fleet size     : {:.2}", exp.n_agents);
            println!("Battery        : {:.2} Wh", exp.battery_capacity_wh);
            println!("Threshold SoC  : {:.0} %", exp.soc_threshold_percent);
            println!("Critical SoC   : {:.0} %", exp.critical_soc_percent);
            println!("Seed           : {:.2}", seed);
            println!("Grid resolution: {:.2}×{:.2}", resolution, resolution);

            let filename=  format!(
                "size={}_fleet={}_batt={}_soc={}",
                label,
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