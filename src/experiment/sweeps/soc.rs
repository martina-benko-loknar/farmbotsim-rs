use crate::experiment::runner::run_single_station_experiment;
use crate::experiment::sweeps::sweep_utils::create_sweep_output_directory;
use crate::experiment::sweeps::sweep_utils::print_experiment_info;
use crate::experiment::models::{ExperimentType, ExperimentInfo};
use crate::ExperimentConfig;

pub fn run_soc_sweep(
    output_dir: &str
)-> Result<(), Box<dyn std::error::Error>> {

    let thresholds = vec![50.0, 60.0, 70.0, 80.0];
    let seeds = 0..5;
    let resolution = 15;
    let label = "M";
    let output_dir_sweep = create_sweep_output_directory(output_dir, "raw/soc_sweep")?;
   
    println!("\n===== EXPERIMENT: SoC threshold sweep ===================================");

    for threshold in thresholds {

        for seed in seeds.clone() {
            let exp = ExperimentConfig {
                seed,
                n_agents: 3,
                soc_threshold_percent: threshold as f32,
                ..Default::default()
            };

            print_experiment_info(label, &exp);

            let filename=  format!(
                "size={}_fleet={}_batt={}_soc={}",
                label,
                exp.n_agents,
                exp.battery_capacity_wh,
                threshold,
            );

            let timestamp = chrono::Utc::now()
                .format("%H%M%S")
                .to_string();

            let info = ExperimentInfo {
                experiment_type: ExperimentType::SocSweep,
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