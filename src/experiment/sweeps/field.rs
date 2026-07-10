use crate::experiment::runner::run_single_station_experiment;
use crate::experiment::sweeps::sweep_utils::create_sweep_output_directory;
use crate::experiment::sweeps::sweep_utils::print_experiment_info;
use crate::experiment::models::{ExperimentType, ExperimentInfo};
use crate::ExperimentConfig;

pub fn run_field_size_sweep(
    output_dir: &str,
) -> Result<(), Box<dyn std::error::Error>> {

    let field_sizes = vec![
        ("S", "configs/field_configs/vineyard/small.json"),
        ("M", "configs/field_configs/vineyard/medium.json"),
        ("L", "configs/field_configs/vineyard/large.json"),
        ("XL", "configs/field_configs/vineyard/xlarge.json"),
    ];

    let seeds = 0..5;
    let resolution = 15;
    let output_dir_sweep = create_sweep_output_directory(output_dir, "raw/field_sweep")?;

    println!("\n===== EXPERIMENT: Field size sweep ===================================");

    for (label, path) in field_sizes {

        for seed in seeds.clone() {

            let exp = ExperimentConfig {
                seed,
                field_config_path: path.to_string(),
                ..Default::default()
            };

            print_experiment_info(label, &exp);

            let filename=  format!(
                "size={}_fleet={}_batt={}_soc={}",
                label,
                exp.n_agents,
                exp.battery_capacity_wh,
                exp.soc_threshold_percent,
            );

            let timestamp = chrono::Utc::now()
                .format("%H%M%S")
                .to_string();

            let info = ExperimentInfo {
                experiment_type: ExperimentType::FieldSweep,
                timestamp: timestamp.clone(),
            };

            run_single_station_experiment(
                resolution,
                &filename,
                &output_dir_sweep,
                exp,
                info,
            );

            println!("======================================================================\n");
        }
    }

    Ok(())
}