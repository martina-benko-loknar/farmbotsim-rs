use crate::ExperimentConfig;

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