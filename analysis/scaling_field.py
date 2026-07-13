import os
import sys
import pandas as pd

from loaders.json_loader import load_single_station_results, FIELD_SIZE_ORDER
from aggregation.aggregate import summarize
from results_dir import resolve_results_root, results_root_tag

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "plotting"))
from sensitivity import generate_sensitivity_errorbar_plot

SWEEP_NAME = "field_sweep"
EXPORTS_BASE_DIR = "exports"


def main():
    results_root = resolve_results_root()
    RESULTS_DIR = f"{results_root}/raw/{SWEEP_NAME}"
    FIGURES_DIR = f"{results_root}/figures/{SWEEP_NAME}"
    EXPORTS_DIR = f"{EXPORTS_BASE_DIR}/{results_root_tag(results_root)}"

    df = load_single_station_results(RESULTS_DIR)
    print(f"Loaded {len(df)} single-station runs from {RESULTS_DIR}")
    df["field_size"] = pd.Categorical(
        df["field_size"], categories=FIELD_SIZE_ORDER, ordered=True
    )

    summary = summarize(
        df,
        group_cols=["field_size"],
        value_cols=["runtime_sec", "ego_energy_wh", "ego_distance_m"],
    ).sort_values("field_size")

    print(summary)

    os.makedirs(EXPORTS_DIR, exist_ok=True)
    summary_path = f"{EXPORTS_DIR}/field_scaling_summary.csv"
    summary.to_csv(summary_path, index=False)
    print(f"Saved summary to {summary_path}")

    os.makedirs(FIGURES_DIR, exist_ok=True)
    generate_sensitivity_errorbar_plot(
        x=summary["field_size"],
        y_mean=summary["runtime_sec_mean"],
        y_std=summary["runtime_sec_std"],
        xlabel="field size (/)",
        ylabel="runtime (s)",
        output_dir=FIGURES_DIR,
        prefix="field_scaling_runtime",
    )


if __name__ == "__main__":
    main()
