import matplotlib.pyplot as plt
from viz_utils import setup_latex_fonts

# ============================================================================
# Sensitivity / Sweep Plots (mean +/- std across seeds)
# ============================================================================


def generate_sensitivity_errorbar_plot(
    x,
    y_mean,
    y_std,
    xlabel: str,
    ylabel: str,
    output_dir: str = "results",
    prefix: str = "sensitivity",
):
    """Plot mean +/- std of a metric against a swept parameter."""
    setup_latex_fonts(20)

    fig, ax = plt.subplots(figsize=(8, 6))

    ax.errorbar(
        x, y_mean, yerr=y_std,
        fmt='o-', color='black', ecolor='black',
        elinewidth=1.5, capsize=5, markersize=7, linewidth=1.5,
    )

    ax.set_xlabel(xlabel)
    ax.set_ylabel(ylabel)
    ax.tick_params(labelsize=20)
    ax.grid(True, linewidth=0.5, alpha=0.5)

    plt.tight_layout()
    filename = f"{output_dir}/{prefix}"
    plt.savefig(f"{filename}.pdf", bbox_inches='tight')
    print(f"- {prefix}.pdf")
    plt.close()
