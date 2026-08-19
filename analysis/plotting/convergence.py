import matplotlib.pyplot as plt
from viz_utils import setup_latex_fonts

# Okabe-Ito colorblind-safe pair: black (EGO) vs blue (grid search).
EGO_COLOR = "black"
GRID_COLOR = "#0072B2"

# 4-class sequential blue ramp (ColorBrewer "Blues", skipping the near-white
# lightest class for visibility) for field_sweep's convergence overlay --
# field size is an ordinal variable (S < M < L < XL), so a single-hue ramp
# communicates that ordering; a qualitative/categorical palette would imply
# the sizes are unordered, which they aren't.
FIELD_SIZE_COLORS = {
    "S": "#bdd7e7",
    "M": "#6baed6",
    "L": "#3182bd",
    "XL": "#08519c",
}

# Same idea for fleet_sweep's convergence overlay, but a different hue
# (ColorBrewer "Greens", same skip-the-near-white-class treatment) so a
# fleet-size overlay and a field-size overlay stay visually distinct from
# each other if they ever appear near one another in the paper.
FLEET_SIZE_COLORS = {
    1: "#bae4b3",
    2: "#74c476",
    3: "#31a354",
    4: "#006d2c",
}


def generate_convergence_comparison_plot(
    ego_x, ego_mean, ego_std=None,
    grid_x=None, grid_mean=None, grid_std=None,
    output_dir: str = "results",
    prefix: str = "convergence",
    ylabel: str = "$E_{\\mathrm{tot}}$ (Wh), best found so far",
):
    """
    Best-energy-found-so-far vs number of evaluations for EGO. Also draws
    the grid-search curve when grid_x/grid_mean/grid_std are given -- pass
    None (default) for EGO-only data, e.g. fleet_sweep/field_sweep, which
    no longer runs grid search alongside EGO.

    ego_std/grid_std are only meaningful when ego_mean/grid_mean are
    aggregated across multiple seeds (see comparison_fleet.py /
    convergence_fleet.py) -- pass None (default) for a single run, e.g.
    single_station_viz.py, which has no seed-to-seed spread to shade. The
    shaded band is skipped entirely rather than drawn at a fake std=0
    (which happens to render as nothing, but isn't a reason to pass it) --
    the goal is that any gray on this plot legitimately comes from
    ax.grid(...), not from a std band pretending there's variance to show.
    """
    setup_latex_fonts(20)

    fig, ax = plt.subplots(figsize=(8, 6))

    ax.plot(ego_x, ego_mean, "-", color=EGO_COLOR, linewidth=1.5, label="EGO")
    if ego_std is not None:
        ax.fill_between(
            ego_x, ego_mean - ego_std, ego_mean + ego_std,
            color=EGO_COLOR, alpha=0.15, linewidth=0,
        )

    if grid_mean is not None:
        ax.plot(grid_x, grid_mean, "-", color=GRID_COLOR, linewidth=1.5, label="Grid search")
        if grid_std is not None:
            ax.fill_between(
                grid_x, grid_mean - grid_std, grid_mean + grid_std,
                color=GRID_COLOR, alpha=0.15, linewidth=0,
            )

    ax.set_xlabel("evaluations")
    ax.set_ylabel(ylabel)
    ax.tick_params(labelsize=20)
    ax.grid(True, linewidth=0.5, alpha=0.5)
    ax.legend()

    plt.tight_layout()
    filename = f"{output_dir}/{prefix}"
    plt.savefig(f"{filename}.pdf", bbox_inches="tight")
    print(f"- {prefix}.pdf")
    plt.close()


def generate_multi_convergence_plot(
    curves,
    output_dir: str = "results",
    prefix: str = "convergence_overlay",
    xlabel: str = "evaluations",
    ylabel: str = "\\% above best found",
):
    """
    Overlay several best-so-far convergence curves on one axes -- e.g. one
    per field size -- each with its own shaded cross-seed std band. Unlike
    `generate_convergence_comparison_plot`, this isn't an EGO-vs-grid-search
    plot: every curve here is drawn the same way (no "-vs-" framing), meant
    for comparing EGO's own behavior across a swept parameter instead.

    `curves` is a list of dicts: {"label": str, "x": array, "mean": array,
    "std": array (optional), "color": str (optional, falls back to
    matplotlib's default cycle)}.
    """
    setup_latex_fonts(20)

    fig, ax = plt.subplots(figsize=(8, 6))

    for curve in curves:
        color = curve.get("color")
        ax.plot(curve["x"], curve["mean"], "-", color=color, linewidth=1.5, label=curve["label"])
        if curve.get("std") is not None:
            ax.fill_between(
                curve["x"], curve["mean"] - curve["std"], curve["mean"] + curve["std"],
                color=color, alpha=0.15, linewidth=0,
            )

    ax.set_xlabel(xlabel)
    ax.set_ylabel(ylabel)
    ax.tick_params(labelsize=20)
    ax.grid(True, linewidth=0.5, alpha=0.5)
    ax.legend()

    plt.tight_layout()
    filename = f"{output_dir}/{prefix}"
    plt.savefig(f"{filename}.pdf", bbox_inches="tight")
    print(f"- {prefix}.pdf")
    plt.close()
