import numpy as np
import matplotlib.pyplot as plt
from viz_utils import setup_latex_fonts
from convergence import EGO_COLOR, GRID_COLOR


def generate_grouped_bar_comparison_plot(
    group_labels,
    ego_mean, ego_std,
    grid_mean=None, grid_std=None,
    xlabel: str = "",
    ylabel: str = "",
    output_dir: str = "results",
    prefix: str = "comparison",
    series_a_label: str = "EGO",
    series_b_label: str = "Grid search",
    series_a_color: str = EGO_COLOR,
    series_b_color: str = GRID_COLOR,
):
    """
    Bar chart of one series, one bar per group. Also draws a paired second
    bar when grid_mean/grid_std are given (grouped bar chart) -- pass None
    (default) for single-series data, e.g. fleet_sweep/field_sweep, which no
    longer runs grid search alongside EGO. Despite the "ego"/"grid" argument
    names (this started as an EGO-vs-grid-search plot), series_a_label /
    series_b_label let callers reuse this for any two-series comparison --
    see comparison_fleet_slots.py.
    """
    setup_latex_fonts(20)

    x = np.arange(len(group_labels))
    has_grid = grid_mean is not None
    width = 0.35 if has_grid else 0.6

    fig, ax = plt.subplots(figsize=(8, 6))

    ego_x = x - width / 2 if has_grid else x
    ax.bar(ego_x, ego_mean, width, yerr=ego_std, capsize=5,
           color=series_a_color, label=series_a_label)
    if has_grid:
        ax.bar(x + width / 2, grid_mean, width, yerr=grid_std, capsize=5,
               color=series_b_color, label=series_b_label)

    ax.set_xticks(x)
    ax.set_xticklabels(group_labels)
    ax.set_xlabel(xlabel)
    ax.set_ylabel(ylabel)
    ax.tick_params(labelsize=20)
    ax.grid(True, axis="y", linewidth=0.5, alpha=0.5)
    ax.legend()

    plt.tight_layout()
    filename = f"{output_dir}/{prefix}"
    plt.savefig(f"{filename}.pdf", bbox_inches="tight")
    print(f"- {prefix}.pdf")
    plt.close()
