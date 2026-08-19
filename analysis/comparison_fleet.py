import os
import sys

from loaders.json_loader import load_single_station_results
from aggregation.aggregate import summarize
from results_dir import resolve_results_root, resolve_profile, results_root_tag

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "plotting"))
from sensitivity import generate_sensitivity_mechanism_plot

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

    # Energy per completed task rather than total energy, since fleet size
    # directly changes how many tasks get completed in a run.
    df["ego_energy_wh_per_task"] = df["ego_energy_wh"] / df["ego_completed_tasks"]

    # Time per evaluation, computed before aggregation -- see
    # scaling_fleet.py's comments for what ego_optimization_time_sec
    # includes (simulation + BO machinery) and why dividing by
    # ego_evaluations isolates per-evaluation cost here (fleet_sweep holds
    # the EGO budget fixed across fleet sizes).
    df["ego_time_per_eval_sec"] = df["ego_optimization_time_sec"] / df["ego_evaluations"]

    # No grid-search comparison here: fleet_sweep is EGO-only
    # (run_ego_experiment, no grid_search block per file -- see
    # loaders/json_loader.py::load_single_station_results), by design, not
    # just for now -- see the 2026-08-18 conversation / README for why. The
    # EGO-vs-grid-search story lives in the dedicated single-station
    # calibration data instead.
    #
    # Paired as one figure -- physical cost (energy/task) and computational
    # cost (time/evaluation) both vs. fleet size -- rather than two
    # standalone plots: not a causal "top explains bottom" mechanism like
    # the SoC/battery mechanism plots, just two different notions of "cost
    # of a bigger fleet" shown together for economy of figures.
    summary = summarize(
        df,
        group_cols=["fleet_size"],
        value_cols=["ego_energy_wh_per_task", "ego_time_per_eval_sec", "ego_evaluations"],
    ).sort_values("fleet_size")

    print(summary)

    if (summary["ego_evaluations_std"] > 0).any():
        print("note: evaluation count is not constant across fleet sizes in this data "
              "(expected: fleet.rs holds the EGO budget fixed) -- the 'isolates "
              "per-evaluation cost' framing in this script's comments assumes it is, "
              "double check before trusting the bottom panel as such.")

    os.makedirs(EXPORTS_DIR, exist_ok=True)
    summary_path = f"{EXPORTS_DIR}/fleet_energy_time_summary.csv"
    summary.to_csv(summary_path, index=False)
    print(f"Saved summary to {summary_path}")

    os.makedirs(FIGURES_DIR, exist_ok=True)
    generate_sensitivity_mechanism_plot(
        x=summary["fleet_size"],
        panels=[
            {
                "ylabel": "$E_{\\mathrm{tot}}$ / task (Wh)",
                "y_mean": summary["ego_energy_wh_per_task_mean"],
                "y_std": summary["ego_energy_wh_per_task_std"],
            },
            {
                "ylabel": "time / evaluation (s)",
                "y_mean": summary["ego_time_per_eval_sec_mean"],
                "y_std": summary["ego_time_per_eval_sec_std"],
            },
        ],
        xlabel="fleet size (/)",
        output_dir=FIGURES_DIR,
        prefix="fleet_energy_time_mechanism",
    )


if __name__ == "__main__":
    main()
