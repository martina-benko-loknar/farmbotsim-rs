import json
import os

from viz_pipeline import *


def load_json_data(json_path):
    """Load grid search results from JSON file"""
    with open(json_path, 'r') as f:
        data = json.load(f)
    return data


def parse_field_config(data):
    """
    Compute field bounds directly from exported obstacle geometry.
    """

    field = data.get("field", {})
    obstacles = field.get("obstacles", [])

    if not obstacles:
        print("WARNING: no obstacle geometry available.")
        return (0.0, 12.0, 0.0, 12.0)

    xs = []
    ys = []

    for obs in obstacles:
        for p in obs:
            xs.append(p["x"])
            ys.append(p["y"])

    min_x = min(xs)
    max_x = max(xs)

    min_y = min(ys)
    max_y = max(ys)

    # small padding
    padding = 1.0

    return (
        min_x - padding,
        max_x + padding,
        min_y - padding,
        max_y + padding
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


# ============================================================================
# IMPORTANT:
# DO NOT reconstruct geometry from angles / rows in Python.
#
# Rust already computes the real obstacle geometry internally.
# Python should only visualize already-generated obstacle polygons.
# ============================================================================

def parse_obstacles(data):
    """
    Parse already-generated obstacle polygons exported from Rust.

    Expected JSON format:

    "field": {
        "obstacles": [
            [
                {"x": ..., "y": ...},
                {"x": ..., "y": ...},
                ...
            ],
            ...
        ]
    }
    """

    obstacles = []

    field = data.get('field', {})

    obstacles_raw = field.get('obstacles', [])

    if not obstacles_raw:

        print("WARNING:")
        print("No exported obstacle polygons found in JSON.")
        print("Python will show empty field geometry.")
        print(
            "Export final obstacle polygons from Rust "
            "(FieldConfig::get_obstacles())."
        )

        return obstacles

    total_obstacles = 0

    for obs in obstacles_raw:

        points = []

        for p in obs:
            points.append(
                Pos2(p['x'], p['y'])
            )

        obstacles.append(
            Obstacle(points)
        )

        total_obstacles += 1

    print(
        f"Loaded {total_obstacles} obstacle polygons from Rust export."
    )

    return obstacles


def visualize_single_station(json_path, output_dir="results"):
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
        output_dir=output_dir
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

    print(f"Input file: {json_file}")

    visualize_single_station(json_file, output_directory)

    print("-" * 70)
    print(f"✓ Plots generated successfully and saved to '{output_directory}'!")
    print("=" * 70)