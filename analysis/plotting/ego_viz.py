import os

from viz_io import load_json_data, parse_obstacles
from viz_pipeline import generate_all_ego_plots


def visualize_ego(json_path, output_dir="results", prefix=None):
    """
    Visualize a standalone EGO optimization run (EgoExport JSON shape:
    metadata, ego: {summary, trace}, timing, field).
    """

    data = load_json_data(json_path)

    obstacles = parse_obstacles(data)

    ego = data.get("ego", {})
    evaluations = ego.get("trace", {}).get("evaluation_history", [])
    summary = ego.get("summary", {})

    print("-" * 70)
    print(f"Total evaluations : {len(evaluations)}")

    best_metrics = summary.get("best_metrics")
    optimal_position = summary.get("optimal_position", [])

    if best_metrics is not None:
        print(f"Best energy        : {best_metrics['energy_wh']:.2f} Wh")
        print(f"Optimal position(s): {[(p['x'], p['y']) for p in optimal_position]}")

    os.makedirs(output_dir, exist_ok=True)

    print("-" * 70)
    print("Generating plots...")

    generate_all_ego_plots(
        evaluations,
        obstacles,
        output_dir=output_dir,
        prefix=prefix,
    )


if __name__ == "__main__":

    import sys

    if len(sys.argv) < 2:

        print("Usage:")
        print(
            "  python ego_viz.py "
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

    visualize_ego(json_file, output_directory, run_prefix)

    print("-" * 70)
    print(f"✓ Plots generated successfully and saved to '{output_directory}'!")
    print("=" * 70)
