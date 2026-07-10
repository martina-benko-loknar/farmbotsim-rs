use std::error::Error;
use std::fs;
use crate::ExperimentConfig;

pub fn create_sweep_output_directory(
    base_output: &str,
    sweep_output: &str,
) -> Result<String, Box<dyn Error>> {
    let output_dir = format!("{base_output}/{sweep_output}");

    fs::create_dir_all(&output_dir)?;

    Ok(output_dir)
}

pub fn print_experiment_info(
    label: &str,
    exp: &ExperimentConfig,
) {
    println!("Field size     : {label}");
    println!("Fleet size     : {}", exp.n_agents);
    println!("Battery        : {} Wh", exp.battery_capacity_wh);
    println!("Threshold SoC  : {} %", exp.soc_threshold_percent);
    println!("Critical SoC   : {} %", exp.critical_soc_percent);
    println!("Seed           : {}", exp.seed);
}