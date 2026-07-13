import os
import sys

from loaders.json_loader import load_single_evaluation_results
from aggregation.aggregate import summarize
from results_dir import resolve_results_root, results_root_tag

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "plotting"))
from sensitivity import generate_sensitivity_errorbar_plot

SWEEP_NAME = "battery_sweep"
EXPORTS_BASE_DIR = "exports"


def main():
    results_root = resolve_results_root()
    RESULTS_DIR = f"{results_root}/raw/{SWEEP_NAME}"
    FIGURES_DIR = f"{results_root}/figures/{SWEEP_NAME}"
    EXPORTS_DIR = f"{EXPORTS_BASE_DIR}/{results_root_tag(results_root)}"

    df = load_single_evaluation_results(RESULTS_DIR)
    print(f"Loaded {len(df)} single-evaluation runs from {RESULTS_DIR}")

    summary = summarize(
        df,
        group_cols=["battery_wh"],
        value_cols=["energy_wh", "distance_m", "charging_distance_m"],
    ).sort_values("battery_wh")

    print(summary)

    os.makedirs(EXPORTS_DIR, exist_ok=True)
    summary_path = f"{EXPORTS_DIR}/battery_sensitivity_summary.csv"
    summary.to_csv(summary_path, index=False)
    print(f"Saved summary to {summary_path}")

    os.makedirs(FIGURES_DIR, exist_ok=True)
    generate_sensitivity_errorbar_plot(
        x=summary["battery_wh"],
        y_mean=summary["energy_wh_mean"],
        y_std=summary["energy_wh_std"],
        xlabel="battery capacity (Wh)",
        ylabel="$E_{\\mathrm{tot}}$ (Wh)",
        output_dir=FIGURES_DIR,
        prefix="battery_sensitivity_energy",
    )


if __name__ == "__main__":
    main()
