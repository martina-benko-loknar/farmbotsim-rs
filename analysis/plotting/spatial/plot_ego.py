import os
from typing import Dict, List, Optional

import matplotlib.pyplot as plt
from matplotlib import cm
from matplotlib.colors import Normalize

from viz_models import Obstacle
from viz_utils import setup_latex_fonts, add_obstacles_to_3d_plot

# ============================================================================
# EGO Optimization Plots
# ============================================================================

MAX_ENERGY_WH = 50000


def _energy(entry: Dict) -> float:
    return entry["metrics"]["energy_wh"]


def generate_optimization_landscape_plot(
    evaluations: List[Dict],
    obstacles: List[Obstacle],
    output_dir: str = "results",
    prefix: Optional[str] = None,
):
    """
    3D scatter of evaluated station positions, split into initial
    sampling ("init") vs Bayesian optimization ("ego") phases.
    """
    setup_latex_fonts(30)

    if not evaluations:
        print("No evaluation data provided")
        return

    os.makedirs(output_dir, exist_ok=True)

    init = [e for e in evaluations if e["phase"] == "init" and _energy(e) <= MAX_ENERGY_WH]
    bo = [e for e in evaluations if e["phase"] == "ego" and _energy(e) <= MAX_ENERGY_WH]

    if not init and not bo:
        print("All evaluations filtered out")
        return

    n_stations = len((init + bo)[0]["positions"])

    station_colors = ["blue", "green", "orange", "purple", "cyan", "magenta"]

    fig = plt.figure(figsize=(16, 8))
    ax_init = fig.add_subplot(121, projection="3d")
    ax_bo = fig.add_subplot(122, projection="3d")

    def plot_init(ax, phase_data):
        if not phase_data:
            ax.set_title("Initial Sampling\n(no data)")
            return

        station_x = [[] for _ in range(n_stations)]
        station_y = [[] for _ in range(n_stations)]
        station_z = [[] for _ in range(n_stations)]

        for entry in phase_data:
            energy = _energy(entry)
            for i, (x, y) in enumerate(entry["positions"]):
                station_x[i].append(x)
                station_y[i].append(y)
                station_z[i].append(energy)

        for i in range(n_stations):
            ax.scatter(
                station_x[i], station_y[i], station_z[i],
                c=station_colors[i % len(station_colors)],
                s=10, alpha=0.6, label=f"Station {i + 1}",
            )

            for x, y, z in zip(station_x[i], station_y[i], station_z[i]):
                ax.plot(
                    [x, x], [y, y], [0, z],
                    linestyle=":", linewidth=0.5, alpha=0.3,
                    color=station_colors[i % len(station_colors)],
                )

        best_entry = min(phase_data, key=_energy)
        best_positions = best_entry["positions"]
        best_energy = _energy(best_entry)

        opt_x = [p[0] for p in best_positions]
        opt_y = [p[1] for p in best_positions]
        opt_z = [best_energy] * len(best_positions)

        ax.scatter(opt_x, opt_y, opt_z, c="white", marker="x", s=300, linewidths=3, zorder=100)
        ax.scatter(opt_x, opt_y, opt_z, c="black", marker="x", s=200, linewidths=2, label="Best", zorder=101)

        if obstacles:
            add_obstacles_to_3d_plot(ax, obstacles, best_energy)

        ax.set_title("Initial Sampling")
        ax.set_xlabel("$x$ (m)")
        ax.set_ylabel("$y$ (m)")
        ax.set_zlabel("Energy (Wh)")
        ax.legend()

    def plot_bo(ax, phase_data):
        if not phase_data:
            ax.set_title("Bayesian Optimization\n(no data)")
            return

        energies = [_energy(e) for e in phase_data]
        norm = Normalize(vmin=min(energies), vmax=max(energies))
        cmap = plt.get_cmap("coolwarm")

        for entry in phase_data:
            energy = _energy(entry)
            color = cmap(norm(energy))
            positions = entry["positions"]

            xs = [p[0] for p in positions]
            ys = [p[1] for p in positions]
            zs = [energy] * len(positions)

            ax.scatter(xs, ys, zs, color=color, s=20, alpha=0.9)

            if len(positions) >= 2:
                for i in range(len(positions) - 1):
                    x1, y1 = positions[i]
                    x2, y2 = positions[i + 1]

                    ax.plot(
                        [x1, x2], [y1, y2], [energy, energy],
                        linestyle=":", linewidth=1.5, alpha=1, color=color,
                    )

        best = min(phase_data, key=_energy)
        best_positions = best["positions"]
        best_energy = _energy(best)

        bx = [p[0] for p in best_positions]
        by = [p[1] for p in best_positions]
        bz = [best_energy] * len(best_positions)

        ax.scatter(bx, by, bz, c="white", marker="x", s=300, linewidths=3, zorder=100)
        ax.scatter(bx, by, bz, c="black", marker="x", s=200, linewidths=2, label="Best", zorder=101)

        ax.set_title("Bayesian Optimization")
        ax.set_xlabel("$x$ (m)")
        ax.set_ylabel("$y$ (m)")
        ax.set_zlabel("Energy (Wh)")

        sm = cm.ScalarMappable(norm=norm, cmap=cmap)
        sm.set_array([])

        plt.colorbar(sm, ax=ax, shrink=0.7, pad=0.1, label="Energy (Wh)")
        ax.legend()

    plot_init(ax_init, init)
    plot_bo(ax_bo, bo)

    plt.tight_layout()

    stem = prefix or "ego"
    filename = f"{output_dir}/{stem}_optimization_landscape"
    fname = f"{stem}_optimization_landscape"
    plt.savefig(f"{filename}.pdf", bbox_inches="tight")
    print(f"- {fname}.pdf")
    plt.close()


def generate_convergence_plot(
    evaluations: List[Dict],
    output_dir: str = "results",
    prefix: Optional[str] = None,
):
    """Energy-vs-evaluation-index convergence plot, split by init/EGO phase."""
    setup_latex_fonts(30)

    if not evaluations:
        print("No convergence data available")
        return

    os.makedirs(output_dir, exist_ok=True)

    init = [e for e in evaluations if e["phase"] == "init"]
    bo = [e for e in evaluations if e["phase"] == "ego"]

    def unpack(eval_list):
        x = [e["evaluation"] for e in eval_list]
        y = [min(_energy(e), MAX_ENERGY_WH) for e in eval_list]
        best = [e["is_new_best"] for e in eval_list]
        return x, y, best

    x_init, y_init, b_init = unpack(init)
    x_bo, y_bo, b_bo = unpack(bo)

    fig, axes = plt.subplots(1, 2, figsize=(14, 8), sharey=False)

    axes[0].plot(x_init, y_init, "-", linewidth=1.5, alpha=0.8)
    axes[0].scatter(
        [x_init[i] for i, b in enumerate(b_init) if b],
        [y_init[i] for i, b in enumerate(b_init) if b],
        color="red", s=40, label="New best",
    )
    axes[0].set_title("Initial sampling")
    axes[0].set_xlabel("Evaluation")
    axes[0].set_ylabel("Energy (Wh)")
    axes[0].grid(True)

    axes[1].plot(x_bo, y_bo, "-", linewidth=1.5, alpha=0.8)
    axes[1].scatter(
        [x_bo[i] for i, b in enumerate(b_bo) if b],
        [y_bo[i] for i, b in enumerate(b_bo) if b],
        color="red", s=40, label="New best",
    )
    axes[1].set_title("Bayesian Optimization")
    axes[1].set_xlabel("Evaluation")
    axes[1].grid(True)

    if y_init:
        axes[0].axhline(min(y_init), linestyle="--", alpha=0.4)
    if y_bo:
        axes[1].axhline(min(y_bo), linestyle="--", alpha=0.4)

    plt.tight_layout()

    stem = prefix or "ego"
    filename = f"{output_dir}/{stem}_convergence"
    fname = f"{stem}_convergence"
    plt.savefig(f"{filename}.pdf", bbox_inches="tight")
    print(f"- {fname}.pdf")

    all_energies = [_energy(e) for e in evaluations]
    print("Convergence summary:")
    print(f"  INIT points: {len(init)}")
    print(f"  BO points: {len(bo)}")
    print(f"  Initial energy: {all_energies[0]:.2f} Wh")
    print(f"  Final energy: {all_energies[-1]:.2f} Wh")

    plt.close()
