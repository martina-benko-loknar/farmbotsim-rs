use crate::experiment::runner::run_single_station_experiment;

pub fn run_combined_sweep(output_dir: &str) {

    let fleet_sizes = vec![1, 4, 8];
    let battery_sizes = vec![250.0, 423.0, 600.0];
    let soc_thresholds = vec![10.0, 20.0, 30.0];

    let seeds = 0..10;

    for n_agents in fleet_sizes {

        for battery in battery_sizes.clone() {

            for soc in soc_thresholds.clone() {

                for seed in seeds.clone() {

                    let run = run_single_station_experiment(
                        10,
                        output_dir,
                    );

                    println!(
                        "agents={} battery={} soc={} seed={} -> {}",
                        n_agents,
                        battery,
                        soc,
                        seed,
                        run.results_path
                    );
                }
            }
        }
    }
}