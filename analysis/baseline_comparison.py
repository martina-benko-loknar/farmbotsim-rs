import os
import sys

import matplotlib.pyplot as plt

from loaders.json_loader import load_baseline_comparison_results
from aggregation.aggregate import summarize
from results_dir import resolve_results_root, resolve_profile, results_root_tag

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "plotting"))
from viz_utils import setup_latex_fonts

SWEEP_NAME = "baseline_comparison"
EXPORTS_BASE_DIR = "exports"

# Display order: geometry-only heuristics first, task-aware heuristic next,
# EGO's own result last (the thing everything else is compared against).
LAYOUT_ORDER = [
    "diagonal_corners", "horizontal_symmetry", "vertical_symmetry",
    "split_center", "tight_center", "task_centroid", "ego_best",
]

LAYOUT_LABELS = {
    "diagonal_corners": "diagonal\ncorners",
    "horizontal_symmetry": "horizontal\nsymmetry",
    "vertical_symmetry": "vertical\nsymmetry",
    "split_center": "split\ncenter",
    "tight_center": "tight\ncenter",
    "task_centroid": "task\ncentroid",
    "ego_best": "EGO\n(best)",
}


def main():
    results_root = resolve_results_root()
    profile = resolve_profile()
    RESULTS_DIR = f"{results_root}/raw/{profile}/{SWEEP_NAME}"
    FIGURES_DIR = f"{results_root}/figures/{profile}/{SWEEP_NAME}"
    EXPORTS_DIR = f"{EXPORTS_BASE_DIR}/{results_root_tag(results_root)}/{profile}"

    df = load_baseline_comparison_results(RESULTS_DIR)
    print(f"Loaded {len(df)} baseline-comparison runs from {RESULTS_DIR}")
    if df.empty:
        print("No data -- nothing to summarize or plot.")
        return

    # Energy per completed task rather than total energy, since task count
    # can drift slightly between runs (termination is a >= check, not ==).
    df["energy_wh_per_task"] = df["energy_wh"] / df["completed_tasks"]

    summary = summarize(
        df,
        group_cols=["layout"],
        value_cols=["energy_wh_per_task", "distance_m", "charging_distance_m"],
    )
    summary["layout"] = summary["layout"].astype("category")
    summary["layout"] = summary["layout"].cat.set_categories(
        [l for l in LAYOUT_ORDER if l in summary["layout"].unique()]
    )
    summary = summary.sort_values("layout").reset_index(drop=True)

    ego_energy = summary.loc[summary["layout"] == "ego_best", "energy_wh_per_task_mean"]
    if not ego_energy.empty:
        ego_e = ego_energy.iloc[0]
        summary["pct_worse_than_ego"] = (
            (summary["energy_wh_per_task_mean"] - ego_e) / ego_e * 100.0
        )

    print(summary)

    os.makedirs(EXPORTS_DIR, exist_ok=True)
    summary_path = f"{EXPORTS_DIR}/baseline_comparison_summary.csv"
    summary.to_csv(summary_path, index=False)
    print(f"Saved summary to {summary_path}")

    os.makedirs(FIGURES_DIR, exist_ok=True)
    setup_latex_fonts(16)

    fig, ax = plt.subplots(figsize=(9, 6))
    labels = [LAYOUT_LABELS.get(l, l) for l in summary["layout"]]
    colors = ["#0072B2"] * (len(summary) - 1) + ["black"] if "ego_best" in summary["layout"].values else ["#0072B2"] * len(summary)
    ax.bar(
        labels, summary["energy_wh_per_task_mean"], yerr=summary["energy_wh_per_task_std"],
        capsize=5, color=colors,
    )
    ax.set_ylabel("$E_{\\mathrm{tot}}$ / task (Wh)")
    ax.tick_params(labelsize=13)
    ax.grid(True, axis="y", linewidth=0.5, alpha=0.5)
    plt.xticks(rotation=0)
    plt.tight_layout()
    fig_path = f"{FIGURES_DIR}/baseline_comparison_energy.pdf"
    plt.savefig(fig_path, bbox_inches="tight")
    print(f"- baseline_comparison_energy.pdf")
    plt.close()


if __name__ == "__main__":
    main()
