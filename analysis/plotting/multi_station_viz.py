import os

from viz_io import load_json_data, parse_field_config, parse_obstacles
from viz_models import Pos2, MultiStationResults
from viz_pipeline import generate_all_multi_station_plots


def parse_multi_station_results(data, obstacles, field_bounds):
    """
    Parse a MultiStationExport JSON (metadata, ego, specialist, timing, field)
    into a MultiStationResults for plotting: EGO's optimum vs. each
    specialist layout it was compared against.
    """

    ego_summary = data.get("ego", {}).get("summary", {})

    optimal_stations = [
        Pos2(p["x"], p["y"])
        for p in ego_summary.get("optimal_position", [])
    ]

    best_metrics = ego_summary.get("best_metrics", {})
    optimal_energy = best_metrics.get("energy_wh", 0.0)
    optimal_distance = best_metrics.get("total_distance_m", 0.0)

    layouts = data.get("specialist", {}).get("layouts", [])

    suboptimal_energy = []
    suboptimal_distance = []

    for evaluated in layouts:
        stations = [
            Pos2(s["x"], s["y"])
            for s in evaluated["layout"]["stations"]
        ]
        metrics = evaluated["metrics"]

        suboptimal_energy.append((stations, metrics["energy_wh"]))
        suboptimal_distance.append((stations, metrics["total_distance_m"]))

    return MultiStationResults(
        optimal_stations=optimal_stations,
        optimal_energy=optimal_energy,
        optimal_distance=optimal_distance,
        suboptimal_configs_energy=suboptimal_energy,
        suboptimal_configs_distance=suboptimal_distance,
        obstacles=obstacles,
        field_bounds=field_bounds,
    )


def visualize_multi_station(json_path, output_dir="results", prefix=None):
    """
    Visualization using Rust-generated obstacle geometry.
    """

    data = load_json_data(json_path)

    field_bounds = parse_field_config(data)

    print("-" * 70)
    print(
        f"Field bounds: "
        f"x=[{field_bounds[0]:.1f}, {field_bounds[1]:.1f}], "
        f"y=[{field_bounds[2]:.1f}, {field_bounds[3]:.1f}]"
    )

    obstacles = parse_obstacles(data)

    results = parse_multi_station_results(data, obstacles, field_bounds)

    print("-" * 70)
    print(f"Optimal energy             : {results.optimal_energy:.2f} Wh")
    print(f"Optimal distance           : {results.optimal_distance:.2f} m")
    print(f"Optimal stations           : {[(s.x, s.y) for s in results.optimal_stations]}")
    print(f"Specialist layouts compared: {len(results.suboptimal_configs_energy)}")

    os.makedirs(output_dir, exist_ok=True)

    print("-" * 70)
    print("Generating plots...")

    generate_all_multi_station_plots(
        results,
        output_dir=output_dir,
        prefix=prefix,
    )


if __name__ == "__main__":

    import sys

    if len(sys.argv) < 2:

        print("Usage:")
        print(
            "  python multi_station_viz.py "
            "<input_json> [output_dir] [run_prefix]"
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

    visualize_multi_station(json_file, output_directory, run_prefix)

    print("-" * 70)
    print(f"✓ Plots generated successfully and saved to '{output_directory}'!")
    print("=" * 70)
