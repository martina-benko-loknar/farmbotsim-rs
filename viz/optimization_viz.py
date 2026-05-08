import os
import matplotlib.pyplot as plt
from datetime import datetime
from viz_utils import setup_latex_fonts, add_obstacles_to_3d_plot
setup_latex_fonts(30)
import numpy as np

def clip_energy(values, percentile=99):
    values = np.array(values, dtype=float)
    threshold = np.percentile(values, percentile)
    return np.clip(values, None, threshold)

# ============================================================
# 3D Optimization Visualization 
# ============================================================
def visualize_optimization_results(
    evaluations,
    timestamp,
    obstacles=None,
    output_dir="results"
):
    """
    3D visualization of evaluated station positions.

    - INIT phase:
        fixed station colors

    - BO phase:
        colormap based on energy
        (blue = low energy, red = high energy)

    - Vertical dotted projection lines improve x-y readability
    """

    import os
    import numpy as np
    import matplotlib.pyplot as plt
    from matplotlib import cm
    from matplotlib.colors import Normalize

    if not evaluations:
        print("No evaluation data provided")
        return

    os.makedirs(output_dir, exist_ok=True)

    # -------------------------------------------------
    # Split INIT / BO
    # -------------------------------------------------
    init = [e for e in evaluations if e["phase"] == "init"]
    bo   = [e for e in evaluations if e["phase"] == "ego"]

    # -------------------------------------------------
    # Filter extreme outliers
    # -------------------------------------------------
    MAX_ENERGY = 50000

    init = [e for e in init if e["energy"] <= MAX_ENERGY]
    bo   = [e for e in bo if e["energy"] <= MAX_ENERGY]

    if not init and not bo:
        print("All evaluations filtered out")
        return

    # -------------------------------------------------
    # Determine number of stations
    # -------------------------------------------------
    sample = (init + bo)[0]
    n_stations = len(sample["positions"])

    # Fixed colors for INIT
    station_colors = [
        "blue",
        "green",
        "orange",
        "purple",
        "cyan",
        "magenta"
    ]

    # -------------------------------------------------
    # Create figure
    # -------------------------------------------------
    fig = plt.figure(figsize=(16, 8))

    ax_init = fig.add_subplot(121, projection='3d')
    ax_bo   = fig.add_subplot(122, projection='3d')

    # =================================================
    # INIT PLOT
    # =================================================
    def plot_init(ax, phase_data):

        if not phase_data:
            ax.set_title("Initial Sampling\n(no data)")
            return

        station_x = [[] for _ in range(n_stations)]
        station_y = [[] for _ in range(n_stations)]
        station_z = [[] for _ in range(n_stations)]

        # Collect data
        for entry in phase_data:

            energy = entry["energy"]

            for i, (x, y) in enumerate(entry["positions"]):
                station_x[i].append(x)
                station_y[i].append(y)
                station_z[i].append(energy)

        # Scatter
        for i in range(n_stations):

            ax.scatter(
                station_x[i],
                station_y[i],
                station_z[i],
                c=station_colors[i % len(station_colors)],
                s=10,
                alpha=0.6,
                label=f"Station {i+1}"
            )

            # Vertical dotted projection lines
            for x, y, z in zip(
                station_x[i],
                station_y[i],
                station_z[i]
            ):
                ax.plot(
                    [x, x],
                    [y, y],
                    [0, z],
                    linestyle=':',
                    linewidth=0.5,
                    alpha=0.3,
                    color=station_colors[i % len(station_colors)]
                )

        # Best point
        best_entry = min(phase_data, key=lambda e: e["energy"])

        best_positions = best_entry["positions"]
        best_energy = best_entry["energy"]

        opt_x = [p[0] for p in best_positions]
        opt_y = [p[1] for p in best_positions]
        opt_z = [best_energy] * len(best_positions)

        ax.scatter(
            opt_x,
            opt_y,
            opt_z,
            c='white',
            marker='x',
            s=300,
            linewidths=3,
            zorder=100
        )

        ax.scatter(
            opt_x,
            opt_y,
            opt_z,
            c='black',
            marker='x',
            s=200,
            linewidths=2,
            label='Best',
            zorder=101
        )

        if obstacles:
            add_obstacles_to_3d_plot(ax, obstacles, best_energy)

        ax.set_title("Initial Sampling")
        ax.set_xlabel('$x$ (m)')
        ax.set_ylabel('$y$ (m)')
        ax.set_zlabel('Energy (Wh)')

        ax.legend()

    # =================================================
    # BO PLOT
    # =================================================
    def plot_bo(ax, phase_data):
        if not phase_data:
            ax.set_title("Bayesian Optimization\n(no data)")
            return

        import numpy as np
        from matplotlib import cm
        from matplotlib.colors import Normalize

        energies = [e["energy"] for e in phase_data]

        norm = Normalize(vmin=min(energies), vmax=max(energies))
        cmap = cm.get_cmap("coolwarm")

        for entry in phase_data:
            energy = entry["energy"]
            color = cmap(norm(energy))

            positions = entry["positions"]

            # ---------------------------------------
            # Plot stations (same energy height)
            # ---------------------------------------
            xs = [p[0] for p in positions]
            ys = [p[1] for p in positions]
            zs = [energy] * len(positions)

            ax.scatter(xs, ys, zs, color=color, s=20, alpha=0.9)

            # ---------------------------------------
            # Connect stations
            # ---------------------------------------
            if len(positions) >= 2:
                for i in range(len(positions) - 1):
                    x1, y1 = positions[i]
                    x2, y2 = positions[i + 1]

                    ax.plot(
                        [x1, x2],
                        [y1, y2],
                        [energy, energy],  
                        linestyle=":",
                        linewidth=1.5,
                        alpha=1,
                        color=color
                    )

            # # ---------------------------------------
            # # Optional: vertical projection per station
            # # ---------------------------------------
            # for x, y in positions:
            #     ax.plot(
            #         [x, x],
            #         [y, y],
            #         [0, energy],
            #         linestyle=":",
            #         linewidth=0.4,
            #         alpha=0.25,
            #         color=color
            #     )

        # ---------------------------------------
        # Best point
        # ---------------------------------------
        best = min(phase_data, key=lambda e: e["energy"])
        best_positions = best["positions"]
        best_energy = best["energy"]

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

    # -------------------------------------------------
    # Plot both phases
    # -------------------------------------------------
    plot_init(ax_init, init)
    plot_bo(ax_bo, bo)

    # -------------------------------------------------
    # Figure title
    # -------------------------------------------------
    fig.suptitle(
        f"EGO Optimization Landscape ({timestamp})"
    )

    plt.tight_layout()

    # -------------------------------------------------
    # Save
    # -------------------------------------------------
    filename = os.path.join(
        output_dir,
        f"optimization_{timestamp}.pdf"
    )

    plt.savefig(
        filename,
        bbox_inches='tight'
    )

    print(f"Optimization plot saved to: {filename}")

    plt.close()

def generate_convergence_plot(evaluations, timestamp, output_dir="results"):
    if not evaluations:
        print("No convergence data available")
        return

    os.makedirs(output_dir, exist_ok=True)

    # -----------------------------
    # Split INIT / BO
    # -----------------------------
    init = [e for e in evaluations if e["phase"] == "init"]
    bo   = [e for e in evaluations if e["phase"] == "ego"]

    def unpack(eval_list):
        x = [e["evaluation"] for e in eval_list]
        y = [e["energy"] for e in eval_list]
        best = [e["is_new_best"] for e in eval_list]
        return x, y, best

    x_init, y_init, b_init = unpack(init)
    x_bo, y_bo, b_bo = unpack(bo)

    MAX_ENERGY = 50000
    y_init = [min(e, MAX_ENERGY) for e in y_init]
    y_bo = [min(e, MAX_ENERGY) for e in y_bo]

    # Create figure with 2 panels
    fig, axes = plt.subplots(1, 2, figsize=(14, 8), sharey=False)

    # -----------------------------
    # Left: INIT
    # -----------------------------
    axes[0].plot(x_init, y_init, '-', linewidth=1.5, alpha=0.8)
    axes[0].scatter(
        [x_init[i] for i, b in enumerate(b_init) if b],
        [y_init[i] for i, b in enumerate(b_init) if b],
        color='red',
        s=40,
        label="New best"
    )

    axes[0].set_title("Initial sampling")
    axes[0].set_xlabel("Evaluation")
    axes[0].set_ylabel("Energy (Wh)")
    axes[0].grid(True)

    # -----------------------------
    # RIGHT: BO
    # -----------------------------
    axes[1].plot(x_bo, y_bo, '-', linewidth=1.5, alpha=0.8)
    axes[1].scatter(
        [x_bo[i] for i, b in enumerate(b_bo) if b],
        [y_bo[i] for i, b in enumerate(b_bo) if b],
        color='red',
        s=40,
        label="New best"
    )

    axes[1].set_title("Bayesian Optimization")
    axes[1].set_xlabel("Evaluation")
    axes[1].grid(True)

    # -----------------------------
    # Show best value lines
    # -----------------------------
    if init:
        best_init = min(y_init)
        axes[0].axhline(best_init, linestyle="--", alpha=0.4)

    if bo:
        best_bo = min(y_bo)
        axes[1].axhline(best_bo, linestyle="--", alpha=0.4)

    # -----------------------------
    # Title, Save, Summary
    # -----------------------------
    timestamp = data.get("timestamp", datetime.utcnow().strftime("%Y%m%d_%H%M%S"))
    fig.suptitle(f"EGO Optimization Convergence ({timestamp})")

    plt.tight_layout()

    filename = os.path.join(output_dir, f"convergence_{timestamp}.pdf")
    plt.savefig(filename, bbox_inches="tight")

    print(f"Convergence plot saved to: {filename}")
    all_energies = [e["energy"] for e in evaluations]

    print("Convergence summary:")
    print(f"  INIT points: {len(init)}")
    print(f"  BO points: {len(bo)}")
    print(f"  Initial energy: {all_energies[0]:.2f} Wh")
    print(f"  Final energy: {all_energies[-1]:.2f} Wh")

    plt.close()

if __name__ == "__main__":
    print("optimization_viz.py started")

    import sys
    import json

    if len(sys.argv) < 3:
        print("Usage: optimization_viz.py <json_path> <output_dir>")
        sys.exit(1)

    json_path = sys.argv[1]
    output_dir = sys.argv[2]

    print("JSON path:", json_path)
    print("Output dir:", output_dir)

    # Load JSON
    with open(json_path, "r") as f:
        data = json.load(f)

    # Extract data
    timestamp = data["timestamp"]
    evaluations = data["evaluations"]
    obstacles = []
    # obstacles_data = data["obstacles"]

    # Convert obstacles
    # obstacles = [
    #     Obstacle(points=[Pos2(p[0], p[1]) for p in obs])
    #     for obs in obstacles_data
    # ]

    # Run plots
    visualize_optimization_results(
        evaluations,
        timestamp,
        obstacles,
        output_dir
    )

    generate_convergence_plot(
        evaluations,
        timestamp,
        output_dir
    )