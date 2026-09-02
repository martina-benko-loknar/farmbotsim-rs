import os
import sys

import matplotlib.pyplot as plt

from loaders.json_loader import load_baseline_comparison_results
from aggregation.aggregate import summarize
from results_dir import resolve_results_root, resolve_profile, results_root_tag

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "plotting"))
from viz_utils import setup_latex_fonts
from convergence import SINGLE_PANEL_MECHANISM_FIGSIZE, SINGLE_PANEL_MECHANISM_MARGINS

SWEEP_NAME = "baseline_comparison_multiagent"
EXPORTS_BASE_DIR = "exports"

# Display order: geometry-only heuristics first, task-aware heuristic next,
# EGO's own result last (the thing everything else is compared against).
LAYOUT_ORDER = [
    "diagonal_corners", "horizontal_symmetry", "vertical_symmetry",
    "split_center", "tight_center", "task_centroid", "ego_best",
]

LAYOUT_LABELS = {
    # Same abbreviations introduced in the multi-station-placement text
    # (Section 4.3.1, "diagonal corners, DC; horizontal symmetry, HS; ...")
    # rather than the full names -- keeps the x-axis readable at this
    # figure's family-matched (8, 4.2) size without needing rotated labels
    # (2026-09-02).
    "diagonal_corners": "DC",
    "horizontal_symmetry": "HS",
    "vertical_symmetry": "VS",
    "split_center": "SC",
    "tight_center": "TC",
    "task_centroid": "CT",
    "ego_best": "EGO",
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
    summary_path = f"{EXPORTS_DIR}/baseline_comparison_multiagent_summary.csv"
    summary.to_csv(summary_path, index=False)
    print(f"Saved summary to {summary_path}")

    os.makedirs(FIGURES_DIR, exist_ok=True)
    # Same base font/tick size as the rest of the paper's sweep/sensitivity
    # plots (setup_latex_fonts(20) via plotting/sensitivity.py's
    # generate_..._plot helpers) -- this script draws its own axes rather
    # than reusing those helpers (categorical x-axis, star-marker overlay
    # for EGO), but should still look like the same family, not smaller.
    setup_latex_fonts(20)

    # Dots-connected-by-lines, matching the rest of the paper's sweep/
    # sensitivity plots, instead of a bar chart. The x-axis is categorical
    # (seven named layouts, not a real sweep), so the connecting line is
    # purely a visual guide across categories, not a trend -- EGO's point
    # is overlaid as a black star to keep it visually distinct, matching
    # the EGO-vs-heuristics star/circle convention used in the multi-station
    # layout figure.
    #
    # figsize/margins reconstructed so this plot's own axes box lands at the
    # literal same size as one panel of battery_sensitivity_energy_
    # mechanism.pdf / soc_sensitivity_energy_mechanism.pdf (see
    # SINGLE_PANEL_MECHANISM_FIGSIZE's docstring), crop=False (full
    # uncropped canvas, bbox_inches=None) -- instead of this script's own
    # historical (9, 6) cropped default (2026-09-02).
    fig, ax = plt.subplots(figsize=SINGLE_PANEL_MECHANISM_FIGSIZE)
    labels = [LAYOUT_LABELS.get(l, l) for l in summary["layout"]]
    x = range(len(summary))
    is_ego = (summary["layout"] == "ego_best").to_numpy()

    ax.errorbar(
        x, summary["energy_wh_per_task_mean"], yerr=summary["energy_wh_per_task_std"],
        fmt='o-', color="#0072B2", ecolor="#0072B2",
        elinewidth=1.5, capsize=5, markersize=7, linewidth=1.5,
    )
    if is_ego.any():
        ego_x = [i for i, e in enumerate(is_ego) if e]
        ax.errorbar(
            ego_x, summary.loc[is_ego, "energy_wh_per_task_mean"],
            yerr=summary.loc[is_ego, "energy_wh_per_task_std"],
            fmt='*', color="black", ecolor="black",
            elinewidth=1.5, capsize=5, markersize=16, zorder=5,
        )

    ax.set_xticks(list(x))
    # Horizontal, unrotated labels -- LAYOUT_LABELS is now the short DC/HS/
    # VS/SC/TC/CT/EGO abbreviations (see above), which fit this figure's
    # width at 20pt without the rotation the old full-name labels needed
    # (2026-09-02).
    ax.set_xticklabels(labels)
    ax.set_ylabel("$E_{\\mathrm{tot}}$ / task (Wh)")
    ax.tick_params(labelsize=20)
    ax.grid(True, axis="y", linewidth=0.5, alpha=0.5)
    # Explicit margins matching soc_convergence.pdf's own axes box (see
    # SINGLE_PANEL_MECHANISM_MARGINS) instead of plt.tight_layout(), which
    # would otherwise size the box to this plot's own content and drift
    # from the rest of the (8, 4.2) family (2026-09-02).
    fig.subplots_adjust(**SINGLE_PANEL_MECHANISM_MARGINS)
    fig_path = f"{FIGURES_DIR}/baseline_comparison_multiagent_energy.pdf"
    plt.savefig(fig_path, bbox_inches=None)
    print(f"- baseline_comparison_multiagent_energy.pdf")
    plt.close()


if __name__ == "__main__":
    main()
