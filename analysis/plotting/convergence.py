import matplotlib.pyplot as plt
from viz_utils import setup_latex_fonts

# Okabe-Ito colorblind-safe pair: black (EGO) vs blue (grid search).
EGO_COLOR = "black"
GRID_COLOR = "#0072B2"


def generate_convergence_comparison_plot(
    ego_x, ego_mean, ego_std,
    grid_x, grid_mean, grid_std,
    output_dir: str = "results",
    prefix: str = "convergence",
):
    """Best-energy-found-so-far vs number of evaluations, EGO vs grid search."""
    setup_latex_fonts(20)

    fig, ax = plt.subplots(figsize=(8, 6))

    ax.plot(ego_x, ego_mean, "-", color=EGO_COLOR, linewidth=1.5, label="EGO")
    ax.fill_between(
        ego_x, ego_mean - ego_std, ego_mean + ego_std,
        color=EGO_COLOR, alpha=0.15, linewidth=0,
    )

    ax.plot(grid_x, grid_mean, "-", color=GRID_COLOR, linewidth=1.5, label="Grid search")
    ax.fill_between(
        grid_x, grid_mean - grid_std, grid_mean + grid_std,
        color=GRID_COLOR, alpha=0.15, linewidth=0,
    )

    ax.set_xlabel("evaluations")
    ax.set_ylabel("$E_{\\mathrm{tot}}$ (Wh), best found so far")
    ax.tick_params(labelsize=20)
    ax.grid(True, linewidth=0.5, alpha=0.5)
    ax.legend()

    plt.tight_layout()
    filename = f"{output_dir}/{prefix}"
    plt.savefig(f"{filename}.pdf", bbox_inches="tight")
    print(f"- {prefix}.pdf")
    plt.close()
