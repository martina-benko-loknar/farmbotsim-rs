import os
import sys

from loaders.json_loader import load_single_station_results
from aggregation.aggregate import summarize
from results_dir import resolve_results_root, resolve_profile, results_root_tag

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "plotting"))
from sensitivity import generate_sensitivity_errorbar_plot

SWEEP_NAME = "fleet_sweep"
EXPORTS_BASE_DIR = "exports"


def main():
    results_root = resolve_results_root()
    profile = resolve_profile()
    RESULTS_DIR = f"{results_root}/raw/{profile}/{SWEEP_NAME}"
    FIGURES_DIR = f"{results_root}/figures/{profile}/{SWEEP_NAME}"
    EXPORTS_DIR = f"{EXPORTS_BASE_DIR}/{results_root_tag(results_root)}/{profile}"

    df = load_single_station_results(RESULTS_DIR)
    print(f"Loaded {len(df)} single-station runs from {RESULTS_DIR}")

    # ego_optimization_time_sec is EGO's own wall-clock (elapsed.as_secs_f64()
    # around the whole optimize() call in ego.rs) -- simulation evaluations
    # *and* the Bayesian-optimization machinery (GP surrogate fit +
    # acquisition-function optimization, once per iteration). The
    # per-evaluation breakdown of this (time / ego_evaluations) now lives in
    # comparison_fleet.py, paired with energy/task -- this script just plots
    # the total.
    summary = summarize(
        df,
        group_cols=["fleet_size"],
        value_cols=["ego_optimization_time_sec"],
    ).sort_values("fleet_size")

    print(summary)

    os.makedirs(EXPORTS_DIR, exist_ok=True)
    summary_path = f"{EXPORTS_DIR}/fleet_scaling_summary.csv"
    summary.to_csv(summary_path, index=False)
    print(f"Saved summary to {summary_path}")

    os.makedirs(FIGURES_DIR, exist_ok=True)
    generate_sensitivity_errorbar_plot(
        x=summary["fleet_size"],
        y_mean=summary["ego_optimization_time_sec_mean"],
        y_std=summary["ego_optimization_time_sec_std"],
        xlabel="fleet size (/)",
        ylabel="EGO optimization time (s)",
        output_dir=FIGURES_DIR,
        prefix="fleet_scaling_time",
    )


if __name__ == "__main__":
    main()
