import os

from viz_pipeline import *
from viz_io import load_json_data, parse_field_config, parse_obstacles


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
    )


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