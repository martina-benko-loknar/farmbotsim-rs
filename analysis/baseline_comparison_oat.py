import os
import sys

import numpy as np
import pandas as pd
import matplotlib.pyplot as plt

from loaders.json_loader import load_baseline_comparison_results
from results_dir import resolve_results_root, resolve_profile, results_root_tag

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "plotting"))
from viz_utils import setup_latex_fonts

SWEEP_NAME = "baseline_comparison"
EXPORTS_BASE_DIR = "exports"

HEURISTICS = [
    "diagonal_corners", "horizontal_symmetry", "vertical_symmetry",
    "split_center", "tight_center", "task_centroid",
]

# Anchor: XL field, 73.2 Wh, fleet 4 -- shared by all three axes below.
ANCHOR = dict(field_size="XL", fleet_size=4, battery_wh=73.2)

FIELD_AXIS = ["S", "M", "L", "XL"]
FLEET_AXIS = [1, 2, 3, 4]
BATTERY_AXIS = [65.0, 73.2, 80.0]


def gap_at(summary, field_size, fleet_size, battery_wh):
    """EGO's % advantage over the best heuristic at one (field, fleet, battery)
    combo -- positive means EGO beats every heuristic by that margin."""
    mask = (
        (summary["field_size"] == field_size)
        & (summary["fleet_size"] == fleet_size)
        & (summary["battery_wh"] == battery_wh)
    )
    sub = summary[mask]
    ego_row = sub[sub["layout"] == "ego_best"]
    heur = sub[sub["layout"].isin(HEURISTICS)]
    if ego_row.empty or heur.empty:
        return None
    ego_e = ego_row["energy_wh_per_task_mean"].iloc[0]
    ego_std = ego_row["energy_wh_per_task_std"].iloc[0]
    best_heur_row = heur.loc[heur["energy_wh_per_task_mean"].idxmin()]
    best_heur_e = best_heur_row["energy_wh_per_task_mean"]
    return {
        "field_size": field_size, "fleet_size": fleet_size, "battery_wh": battery_wh,
        "ego_energy": ego_e, "ego_std": ego_std,
        "best_heuristic": best_heur_row["layout"], "best_heuristic_energy": best_heur_e,
        "pct_ego_advantage_vs_best": (best_heur_e - ego_e) / ego_e * 100.0,
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

    df["energy_wh_per_task"] = df["energy_wh"] / df["completed_tasks"]

    summary = (
        df.groupby(["field_size", "fleet_size", "battery_wh", "layout"])
        .agg(
            energy_wh_per_task_mean=("energy_wh_per_task", "mean"),
            energy_wh_per_task_std=("energy_wh_per_task", "std"),
            n=("seed", "count"),
        )
        .reset_index()
    )
    os.makedirs(EXPORTS_DIR, exist_ok=True)
    summary.to_csv(f"{EXPORTS_DIR}/baseline_comparison_oat_summary.csv", index=False)

    anchor_gap = gap_at(summary, **ANCHOR)

    field_rows = [gap_at(summary, f, ANCHOR["fleet_size"], ANCHOR["battery_wh"]) for f in FIELD_AXIS]
    fleet_rows = [gap_at(summary, ANCHOR["field_size"], n, ANCHOR["battery_wh"]) for n in FLEET_AXIS]
    battery_rows = [gap_at(summary, ANCHOR["field_size"], ANCHOR["fleet_size"], b) for b in BATTERY_AXIS]

    all_rows = {"field": field_rows, "fleet": fleet_rows, "battery": battery_rows}
    gap_df = pd.concat(
        [pd.DataFrame([r for r in rows if r is not None]).assign(axis=name) for name, rows in all_rows.items()],
        ignore_index=True,
    )
    gap_path = f"{EXPORTS_DIR}/baseline_comparison_oat_gap.csv"
    gap_df.to_csv(gap_path, index=False)
    print(f"Saved gap summary to {gap_path}")
    print(gap_df[["axis", "field_size", "fleet_size", "battery_wh", "ego_energy", "best_heuristic",
                   "pct_ego_advantage_vs_best"]].to_string(index=False))

    # ------------------------------------------------------------------
    # Figure: three panels (field size / fleet size / battery capacity),
    # EGO's % advantage over the best heuristic on the y-axis, anchor point
    # marked distinctly since it's shared across all three panels.
    # ------------------------------------------------------------------
    os.makedirs(FIGURES_DIR, exist_ok=True)
    # Same base/tick size as the rest of the sweep-family figures (e.g.
    # baseline_comparison_multiagent.py) rather than this script's own
    # historical 14pt -- at 14pt with no tick_params override, this figure's
    # text rendered visibly smaller than every other figure in the section,
    # including the body text around it (2026-09-02).
    #
    # figsize matters here beyond aspect ratio: on the page, effective text
    # size is font_pt * (\includegraphics width / figsize width), so this
    # figure's on-page text size only matches its 20pt siblings (e.g.
    # baseline_comparison_multiagent_energy.pdf at figsize width 8, LaTeX
    # width 0.65\textwidth) if figsize width is scaled to match this
    # figure's own LaTeX width (0.85\textwidth, see main.tex) at the same
    # ratio: 12 = 0.85/0.65 * 8, rounded. A naive figsize bump that isn't
    # solved against the LaTeX width (tried first, reverted) shrinks the
    # on-page text instead of enlarging it, since more canvas gets squeezed
    # into the same page width.
    setup_latex_fonts(20)

    fig, axes = plt.subplots(1, 3, figsize=(12, 4.5), sharey=True)

    panels = [
        (axes[0], field_rows, FIELD_AXIS, "field size", ANCHOR["field_size"]),
        (axes[1], fleet_rows, FLEET_AXIS, "fleet size", ANCHOR["fleet_size"]),
        (axes[2], battery_rows, BATTERY_AXIS, "battery capacity (Wh)", ANCHOR["battery_wh"]),
    ]

    for ax, rows, xvals, xlabel, anchor_val in panels:
        y = [r["pct_ego_advantage_vs_best"] if r else np.nan for r in rows]
        x_pos = range(len(xvals))
        is_anchor = [xv == anchor_val for xv in xvals]
        ax.axhline(0, color="gray", linewidth=0.8, linestyle="--")
        # Lollipop/stem form rather than bars: a stem from the 0% baseline to
        # each point, with a marker at the value -- the anchor (the demoed
        # config, shared across all three panels) drawn as the same black
        # star used for EGO in Figure~fig:baseline-comparison-multiagent, the
        # rest as blue circles.
        stem_colors = ["black" if a else "#0072B2" for a in is_anchor]
        ax.vlines(x_pos, 0, y, color=stem_colors, linewidth=2, zorder=2)
        for xi, yi, a in zip(x_pos, y, is_anchor):
            if a:
                ax.plot(xi, yi, marker="*", markersize=16, color="black", zorder=3)
            else:
                ax.plot(xi, yi, marker="o", markersize=8, color="#0072B2", zorder=3)
        ax.set_xticks(list(x_pos))
        ax.set_xticklabels([str(xv) for xv in xvals])
        ax.set_xlim(-0.5, len(xvals) - 0.5)
        ax.set_xlabel(xlabel)
        ax.tick_params(labelsize=20)
        ax.grid(True, axis="y", linewidth=0.5, alpha=0.5)

    axes[0].set_ylabel("EGO advantage over best heuristic (\\%)")
    # Explicit margins + full uncropped canvas (bbox_inches=None) instead
    # of bbox_inches='tight' -- at this figure's bigger 20pt font (up from
    # 14pt), matplotlib's usetex bbox estimate for the rotated y-axis label
    # kept clipping its trailing "(\%)" regardless of pad_inches/
    # tight_layout() tuning (both tried, still flaky). Root cause found by
    # measurement: the label string, rotated, is close to 4.4in long at
    # 20pt -- longer than axes[0]'s own height -- so it's centered on (and
    # overflows past) the axes box vertically; a merely-larger top margin
    # doesn't fix that since the overflow is symmetric top and bottom, only
    # a taller figure (bumped to 4.5in above) actually gives it room
    # (2026-09-02).
    fig.subplots_adjust(left=0.075, right=0.995, top=0.90, bottom=0.15, wspace=0.08)

    fig_path = f"{FIGURES_DIR}/baseline_comparison_oat.pdf"
    plt.savefig(fig_path, bbox_inches=None)
    print("- baseline_comparison_oat.pdf")
    plt.close()


if __name__ == "__main__":
    main()
