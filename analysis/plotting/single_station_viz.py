import os
import sys

from viz_pipeline import *
from viz_io import load_json_data, parse_field_config, parse_obstacles
from convergence import generate_convergence_comparison_plot

# analysis/ root, for the aggregation package (aggregation/convergence.py) --
# this script lives in analysis/plotting/, one level below.
sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), ".."))
from aggregation.convergence import (
    ego_best_energy_per_task_so_far,
    grid_best_energy_per_task_so_far,
)


def parse_grid_search_results(data):
    """Parse grid search results from JSON data"""

    grid_search = data.get('grid_search', {})
    summary = grid_search.get('summary', {})
    results_data = grid_search.get('trace', {}).get('points', [])
    grid_resolution = summary.get('grid_resolution', 50)
    results = []

    for point in results_data:
        position = point['position']
        metrics = point['metrics']

        pos = Pos2(position['x'], position['y'])
        energy = metrics['energy_wh']
        total_distance = metrics['total_distance_m']
        charging_distance = metrics['charging_distance_m']

        results.append((pos, energy, total_distance, charging_distance))

    return results, grid_resolution


def find_optimization_minimum(data, results):
    """
    Find the point with minimum energy consumption
    """

    # First, check if the grid search summary already has the best point
    best_point = data.get('grid_search', {}).get('summary', {}).get('best_point')

    if best_point is not None:
        pos = Pos2(best_point['position']['x'], best_point['position']['y'])
        energy = best_point['metrics']['energy_wh']

        return (pos, energy)

    # Fallback: find minimum from results
    if not results:
        return None

    min_idx = min(range(len(results)), key=lambda i: results[i][1])
    pos, energy, _, _ = results[min_idx]

    return (pos, energy)


def find_ego_best(data):
    """
    EGO's own best-found (position, energy), read from the `ego` block --
    present whenever this file came from `run_single_station_experiment`
    (EGO + grid search) or `run_ego_experiment` (EGO alone). Returns None if
    there's no `ego` block or it never found a valid point.
    """

    summary = data.get('ego', {}).get('summary', {})
    best_metrics = summary.get('best_metrics')
    optimal_position = summary.get('optimal_position')

    if best_metrics is None or not optimal_position:
        return None

    pos = Pos2(optimal_position[0]['x'], optimal_position[0]['y'])
    energy = best_metrics['energy_wh']

    return (pos, energy)


def generate_convergence_comparison(data, output_dir, prefix):
    """
    EGO-vs-grid-search convergence plot for a single_station_exp run: best
    energy/task found so far vs evaluation count. Single run, not aggregated
    across seeds -- see comparison_fleet.py/convergence_fleet.py for the
    multi-seed version of this same plot (which also shades a std band).

    Used to additionally emit evaluation-count/wall-time bar charts (EGO vs
    grid search); dropped since for a single station those numbers barely
    move run to run and the bar chart added little -- see
    ego_optimization_time_sec/grid_time_sec in
    loaders/json_loader.py::load_single_station_results for that comparison
    at the aggregate (multi-file) level instead.
    """

    ego = data.get('ego')
    grid = data.get('grid_search')

    if ego is None:
        print("No 'ego' block in this file -- skipping convergence comparison plot.")
        return

    evaluation_history = ego.get('trace', {}).get('evaluation_history', [])
    if not evaluation_history:
        print("EGO trace has no evaluations -- skipping convergence comparison plot.")
        return

    grid_points = grid.get('trace', {}).get('points', []) if grid else []
    has_grid = bool(grid_points)

    # Best energy/task found so far vs evaluation index. Energy per
    # completed task rather than raw energy, matching the project's
    # convention (analysis/README.md) -- task-count termination is a >=
    # check that can drift slightly between evaluated station positions
    # even at fixed field/fleet size, so raw energy isn't always a fair
    # comparison between two candidates. No std band: this is a single run,
    # not aggregated across seeds (see convergence_fleet.py /
    # convergence_field.py for the multi-seed version, which does shade).
    ego_x, ego_y = ego_best_energy_per_task_so_far(evaluation_history)
    grid_x, grid_y = (
        grid_best_energy_per_task_so_far(grid_points) if has_grid else (None, None)
    )

    generate_convergence_comparison_plot(
        ego_x, ego_y,
        grid_x=grid_x, grid_mean=grid_y,
        output_dir=output_dir,
        prefix=f"{prefix}_convergence_comparison" if prefix else "convergence_comparison",
        ylabel="$E_{\\mathrm{tot}}$ / task (Wh), best found so far",
    )

    print(f"EGO: {ego['summary']['total_evaluations']} evaluations, "
          f"{ego['summary']['optimization_time_sec']:.2f} s")
    if has_grid:
        print(f"Grid search: {grid['summary']['valid_points']} valid points, "
              f"{sum(p['metrics']['evaluation_time_sec'] for p in grid_points):.2f} s")


def visualize_single_station(json_path, output_dir="results", prefix=None):
    """
    Visualization using Rust-generated obstacle geometry.
    """

    # Load JSON data
    data = load_json_data(json_path)

    # Parse components
    field_bounds = parse_field_config(data)

    print("-" * 70)

    print(
        f"Field bounds: "
        f"x=[{field_bounds[0]:.1f}, {field_bounds[1]:.1f}], "
        f"y=[{field_bounds[2]:.1f}, {field_bounds[3]:.1f}]"
    )

    # Use obstacle geometry exported directly from Rust
    obstacles = parse_obstacles(data)

    results, grid_resolution = parse_grid_search_results(data)

    xs = [p.x for p, *_ in results]
    ys = [p.y for p, *_ in results]

    print("-" * 70)
    print(
        f"Grid search sampled area: "
        f"x=[{min(xs):.2f}, {max(xs):.2f}], "
        f"y=[{min(ys):.2f}, {max(ys):.2f}]"
    )

    # Find optimization minimum
    optimization_minimum = find_optimization_minimum(
        data,
        results
    )

    if optimization_minimum:

        pos, energy = optimization_minimum
        print(f"Optimal position: ({pos.x:.3f}, {pos.y:.3f})")
        print(f"Minimum energy: {energy:.2f} Wh")

        # Statistics
        energies = [e for _, e, _, _ in results]
        distances = [d for _, _, d, _ in results]
        charging_dists = [cd for _, _, _, cd in results]

        print(f"Energy range:            [{min(energies):.2f}, {max(energies):.2f}] Wh")
        print(f"Distance range:          [{min(distances):.2f}, {max(distances):.2f}] m")
        print(f"Charging distance range: [{min(charging_dists):.2f}, {max(charging_dists):.2f}] m")

    # EGO's own best-found position, overlaid on the grid-search plots below
    # for a visual EGO-vs-grid comparison.
    ego_best = find_ego_best(data)
    if ego_best:
        pos, energy = ego_best
        print(f"EGO best position: ({pos.x:.3f}, {pos.y:.3f})")
        print(f"EGO best energy: {energy:.2f} Wh")

    # Create GridSearchResults object
    grid_results = GridSearchResults(
        results=results,
        grid_resolution=grid_resolution,
        obstacles=obstacles,
        field_bounds=field_bounds
    )

    # Generate all plots
    print("-" * 70)
    print("Generating plots...")

    os.makedirs(output_dir, exist_ok=True)
    generate_all_grid_plots(
        grid_results,
        optimization_minimum,
        output_dir=output_dir,
        prefix=prefix,
        ego_best=ego_best,
    )

    generate_convergence_comparison(data, output_dir, prefix)


# ============================================================================
# Main execution
# ============================================================================

if __name__ == "__main__":

    import sys

    if len(sys.argv) < 2:

        print("Usage:")
        print(
            "  python single_station_viz.py "
            "<input_json> [output_dir]"
        )

        sys.exit(1)

    json_file = sys.argv[1]

    # Default: save results next to input file
    if len(sys.argv) > 2:
        output_directory = sys.argv[2]
    else:
        output_directory = os.path.dirname(json_file)

    # Optional run stem used to prefix generated plot filenames
    run_prefix = sys.argv[3] if len(sys.argv) > 3 else None

    print(f"Input file: {json_file}")

    visualize_single_station(json_file, output_directory, run_prefix)

    print("-" * 70)
    print(f"✓ Plots generated successfully and saved to '{output_directory}'!")
    print("=" * 70)