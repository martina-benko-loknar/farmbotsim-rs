import os
import matplotlib.pyplot as plt
from typing import List, Tuple
from datetime import datetime
from viz_utils import setup_latex_fonts, add_obstacles_to_3d_plot
setup_latex_fonts(30)
from dataclasses import dataclass

@dataclass
class Pos2:
    """2D position"""
    x: float
    y: float

@dataclass
class Obstacle:
    """Obstacle defined by polygon points"""
    points: List[Pos2]

# ============================================================
# 3D Optimization Visualization 
# ============================================================

def visualize_optimization_results(
    evaluated_positions: List[Tuple[List[Tuple[float, float]], float]],
    obstacles: List[Obstacle],
    output_dir: str = "results"
):
    """
    """

    if not evaluated_positions:
        print("No evaluated positions provided")
        return

    os.makedirs(output_dir, exist_ok=True)

    n_stations = len(evaluated_positions[0][0])

    # --- Filter outliers ---
    print(type(evaluated_positions[0][1]))
    print(evaluated_positions[0][1])
    min_energy = min(e for _, e in evaluated_positions)
    threshold = min_energy * 1.5

    filtered = [
        (pos, e) for pos, e in evaluated_positions
        if e <= threshold
    ]

    # --- Prepare per-station arrays ---
    station_x = [[] for _ in range(n_stations)]
    station_y = [[] for _ in range(n_stations)]
    station_z = [[] for _ in range(n_stations)]

    for positions, energy in filtered:
        for i, (x, y) in enumerate(positions):
            station_x[i].append(x)
            station_y[i].append(y)
            station_z[i].append(energy)

    # --- Plot ---
    fig = plt.figure(figsize=(12, 8))
    ax = fig.add_subplot(111, projection='3d')

    colors = ["blue", "green", "orange", "purple", "cyan", "magenta"]

    # Stations
    for i in range(n_stations):
        ax.scatter(
            station_x[i],
            station_y[i],
            station_z[i],
            c=colors[i % len(colors)],
            s=8,
            alpha=0.6,
            label=f"Station {i+1}"
        )

    # --- Optimal point ---
    optimal_positions, optimal_energy = min(
        evaluated_positions, key=lambda x: x[1]
    )

    opt_x = [p[0] for p in optimal_positions]
    opt_y = [p[1] for p in optimal_positions]
    opt_z = [optimal_energy] * len(optimal_positions)

    # White cross (background)
    ax.scatter(opt_x, opt_y, opt_z,
               c='white', marker='x', s=300, linewidths=3,
               label='_nolegend_', zorder=100)

    # Black cross (foreground)
    ax.scatter(opt_x, opt_y, opt_z,
               c='black', marker='x', s=200, linewidths=2,
               label='Optimal Point', zorder=101)

    # --- Obstacles ---
    add_obstacles_to_3d_plot(ax, obstacles, optimal_energy)

    # --- Labels ---
    ax.set_xlabel('$x$ (m)')
    ax.set_ylabel('$y$ (m)')
    ax.set_zlabel('Energy (Wh)')

    ax.legend()

    plt.tight_layout()

    # --- Save ---
    os.makedirs(output_dir, exist_ok=True)
    timestamp = datetime.utcnow().strftime("%Y%m%d_%H%M%S")
    filename = os.path.join(output_dir, f"optimization_{timestamp}.pdf")

    plt.savefig(filename, bbox_inches='tight')
    print(f"Optimization plot saved to: {filename}")

    plt.close()

def generate_convergence_plot(evaluations, timestamp, output_dir="results"):
    if not evaluations:
        print("No convergence data available")
        return

    os.makedirs(output_dir, exist_ok=True)

    iterations = [rec["phase_iteration"] for rec in evaluations]
    energies = [rec["best_energy"] for rec in evaluations]

    plt.figure(figsize=(10, 6))

    plt.plot(iterations, energies, linewidth=2)
    plt.scatter(iterations, energies, s=20)

    plt.xlabel("Iteration Number")
    plt.ylabel("Best Energy (Wh)")
    plt.title("EGO Optimization Convergence")

    plt.grid(True)

    os.makedirs(output_dir, exist_ok=True)      
    filename = os.path.join(output_dir, f"convergence_{timestamp}.pdf")

    plt.tight_layout()
    plt.savefig(filename, bbox_inches='tight')

    print(f"Convergence plot saved to: {filename}")

    # --- Summary ---
    print("Convergence summary:")
    print(f"  Initial best: {energies[0]:.2f} Wh")
    print(f"  Final best: {energies[-1]:.2f} Wh")

    if energies[0] != 0:
        improvement = ((energies[0] - energies[-1]) / energies[0]) * 100
        print(f"  Improvement: {improvement:.1f}%")

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

    timestamp = data["timestamp"]

    # Extract data
    # evaluated_positions = data["evaluated_positions"]
    evaluations = data["evaluations"]
    # obstacles_data = data["obstacles"]

    # Convert obstacles
    # obstacles = [
    #     Obstacle(points=[Pos2(p[0], p[1]) for p in obs])
    #     for obs in obstacles_data
    # ]

    # Run plots
    # visualize_optimization_results(
    #     evaluated_positions,
    #     obstacles,
    #     output_dir
    # )

    generate_convergence_plot(
        evaluations,
        timestamp,
        output_dir
    )