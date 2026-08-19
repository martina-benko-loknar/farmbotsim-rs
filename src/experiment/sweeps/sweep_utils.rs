use crate::ExperimentConfig;

/// Range for the initial-SoC randomization axis (see
/// `EnvOverrides::initial_soc_min_percent`/`max_percent` in `env.rs`).
/// Shared by every sweep that turns this axis on -- `initial_soc.rs`
/// (dedicated sensitivity sweep) and the "full_noise" condition in `soc.rs`/
/// `battery.rs` (robustness check, see
/// `various/randomness_axes_sweep_design.txt`) -- so the range can't drift
/// between sweeps and stays comparable across them.
pub const INITIAL_SOC_RANGE: (f32, f32) = (50.0, 90.0);

/// Range for the task-duration-jitter randomization axis, shared the same
/// way as `INITIAL_SOC_RANGE` -- see `task_duration_jitter.rs` and the
/// "full_noise" condition in `soc.rs`/`battery.rs`.
pub const TASK_DURATION_JITTER_RANGE: (f32, f32) = (0.8, 1.2);

pub fn print_experiment_info(
    exp: &ExperimentConfig,
) {
    println!("Field size     : {}", exp.field_size_label());
    println!("Fleet size     : {}", exp.n_agents);
    println!("Battery        : {} Wh", exp.battery_capacity_wh);
    println!("Threshold SoC  : {} %", exp.soc_threshold_percent);
    println!("Critical SoC   : {} %", exp.critical_soc_percent);
    println!("Seed           : {}", exp.seed);
}