import os
import sys

from loaders.json_loader import load_single_station_results
from aggregation.aggregate import summarize
from results_dir import resolve_results_root, results_root_tag

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "plotting"))
from comparison import generate_grouped_bar_comparison_plot

SWEEP_NAME = "fleet_sweep"
EXPORTS_BASE_DIR = "exports"

VALUE_COLS = [
    "ego_evaluations", "grid_valid_points",
    "ego_energy_wh", "grid_energy_wh",
    "ego_optimization_time_sec", "grid_time_sec",
]


def main():
    results_root = resolve_results_root()
    RESULTS_DIR = f"{results_root}/raw/{SWEEP_NAME}"
    FIGURES_DIR = f"{results_root}/figures/{SWEEP_NAME}/comparison"
    EXPORTS_DIR = f"{EXPORTS_BASE_DIR}/{results_root_tag(results_root)}"

    df = load_single_station_results(RESULTS_DIR)
    print(f"Loaded {len(df)} single-station runs from {RESULTS_DIR}")

    summary = summarize(
        df, group_cols=["fleet_size"], value_cols=VALUE_COLS
    ).sort_values("fleet_size")

    print(summary)

    os.makedirs(EXPORTS_DIR, exist_ok=True)
    summary_path = f"{EXPORTS_DIR}/fleet_comparison_summary.csv"
    summary.to_csv(summary_path, index=False)
    print(f"Saved summary to {summary_path}")

    os.makedirs(FIGURES_DIR, exist_ok=True)
    group_labels = summary["fleet_size"].astype(str).tolist()

    generate_grouped_bar_comparison_plot(
        group_labels,
        summary["ego_evaluations_mean"], summary["ego_evaluations_std"],
        summary["grid_valid_points_mean"], summary["grid_valid_points_std"],
        xlabel="fleet size (/)", ylabel="evaluations",
        output_dir=FIGURES_DIR, prefix="fleet_comparison_evaluations",
    )

    generate_grouped_bar_comparison_plot(
        group_labels,
        summary["ego_energy_wh_mean"], summary["ego_energy_wh_std"],
        summary["grid_energy_wh_mean"], summary["grid_energy_wh_std"],
        xlabel="fleet size (/)", ylabel="$E_{\\mathrm{tot}}$ (Wh)",
        output_dir=FIGURES_DIR, prefix="fleet_comparison_energy",
    )

    generate_grouped_bar_comparison_plot(
        group_labels,
        summary["ego_optimization_time_sec_mean"], summary["ego_optimization_time_sec_std"],
        summary["grid_time_sec_mean"], summary["grid_time_sec_std"],
        xlabel="fleet size (/)", ylabel="time (s)",
        output_dir=FIGURES_DIR, prefix="fleet_comparison_time",
    )


if __name__ == "__main__":
    main()
