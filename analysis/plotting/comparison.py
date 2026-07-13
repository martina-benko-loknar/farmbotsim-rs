import numpy as np
import matplotlib.pyplot as plt
from viz_utils import setup_latex_fonts
from convergence import EGO_COLOR, GRID_COLOR


def generate_grouped_bar_comparison_plot(
    group_labels,
    ego_mean, ego_std,
    grid_mean, grid_std,
    xlabel: str,
    ylabel: str,
    output_dir: str = "results",
    prefix: str = "comparison",
):
    """Grouped bar chart comparing EGO vs grid search, one bar pair per group."""
    setup_latex_fonts(20)

    x = np.arange(len(group_labels))
    width = 0.35

    fig, ax = plt.subplots(figsize=(8, 6))

    ax.bar(x - width / 2, ego_mean, width, yerr=ego_std, capsize=5,
           color=EGO_COLOR, label="EGO")
    ax.bar(x + width / 2, grid_mean, width, yerr=grid_std, capsize=5,
           color=GRID_COLOR, label="Grid search")

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
