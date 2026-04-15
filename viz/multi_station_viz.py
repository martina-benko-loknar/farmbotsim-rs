"""
Script to visualize multi-agent station configurations.
Currently uses hardcoded table data (to be replaced later by file input).
"""

import json
import sys
from viz_core import (
    Pos2, Obstacle, MultiStationResults,
    generate_all_multi_station_plots
)

# ---------------------------------------------------------------------
# DATA (temporary - to be replaced later with file loading)
# ---------------------------------------------------------------------

def get_configs_data():
    """Return hardcoded configuration data (temporary)."""
    return [
        ((17.11, 1.29), (12.47, 14.34), 15049.52, 56.90, 46105.30, 735.90),
        ((1.5, 1.5), (23.0, 23.5), 15497.50, 59.28, 47773.37, 1071.48),
        ((1.5, 12.5), (23.0, 12.5), 15726.52, 61.09, 48678.67, 1992.16),
        ((6.25, 1.5), (6.25, 23.5), 15494.18, 58.80, 47766.84, 1079.20),
        ((12.5, 7.5), (12.5, 17.5), 15110.14, 58.04, 46306.12, 892.41),
        ((17.5, 1.5), (20.0, 1.5), 15231.02, 58.49, 46797.55, 800.37),
        ((12.5, 11.5), (12.5, 13.5), 15341.80, 58.53, 47181.80, 945.21),
    ]


def get_field_config_json():
    """Return field configuration JSON (temporary)."""
    return """
    {
      "configs": [
        {
          "Line": {
            "left_top_pos": { "x": 2.5, "y": 2.5 },
            "angle": "0.000 deg",
            "n_lines": 15,
            "length": "20.000 m",
            "line_spacing": "0.500 m",
            "farm_entity_plan_path": "configs/farm_entity_plans/default_line.json"
          }
        },
        {
          "Line": {
            "left_top_pos": { "x": 15.0, "y": 2.5 },
            "angle": "0.000 deg",
            "n_lines": 15,
            "length": "20.000 m",
            "line_spacing": "0.500 m",
            "farm_entity_plan_path": "configs/farm_entity_plans/default_line.json"
          }
        }
      ]
    }
    """


# ---------------------------------------------------------------------
# PARSING / TRANSFORMATION
# ---------------------------------------------------------------------

def parse_field_config(config_json: str):
    """Parse field configuration and generate obstacles."""
    config = json.loads(config_json)
    obstacles = []

    for line_config in config["configs"]:
        if "Line" not in line_config:
            continue

        line_data = line_config["Line"]

        left_top = line_data["left_top_pos"]
        n_lines = line_data["n_lines"]
        line_spacing = float(line_data["line_spacing"].replace(" m", ""))
        length = float(line_data["length"].replace(" m", ""))
        angle = float(line_data["angle"].replace(" deg", ""))

        obstacle_width = 0.08
        height_offset = 0.2

        pos1_x = left_top['x'] - line_spacing / 2.0
        pos1_y = left_top['y'] - height_offset

        half_width = obstacle_width / 2.0

        for _ in range(n_lines + 1):
            pos2_x = pos1_x
            pos2_y = pos1_y + length + 2 * height_offset

            p1 = Pos2(pos1_x - half_width, pos1_y)
            p2 = Pos2(pos1_x + half_width, pos1_y)
            p3 = Pos2(pos2_x + half_width, pos2_y)
            p4 = Pos2(pos2_x - half_width, pos2_y)

            obstacles.append(Obstacle([p1, p2, p3, p4]))

            pos1_x += line_spacing

    return obstacles


def build_multi_station_results(configs_data, obstacles):
    """Convert raw data into MultiStationResults."""
    configs = []

    for s1, s2, energy, time, distance, dist_charging in configs_data:
        stations = [Pos2(*s1), Pos2(*s2)]
        configs.append({
            'stations': stations,
            'energy': energy,
            'time': time,
            'distance': distance,
            'distance_charging': dist_charging
        })

    optimal_idx = min(range(len(configs)), key=lambda i: configs[i]['energy'])
    optimal = configs[optimal_idx]

    print(f"\nOptimal configuration: #{optimal_idx + 1} with energy: {optimal['energy']:.2f} Wh.")

    sub_energy = [(c['stations'], c['energy']) for i, c in enumerate(configs) if i != optimal_idx]
    sub_energy.sort(key=lambda x: x[1])

    sub_distance = [(c['stations'], c['distance']) for i, c in enumerate(configs) if i != optimal_idx]
    sub_distance.sort(key=lambda x: x[1])

    return MultiStationResults(
        optimal_stations=optimal['stations'],
        optimal_energy=optimal['energy'],
        optimal_distance=optimal['distance'],
        suboptimal_configs_energy=sub_energy,
        suboptimal_configs_distance=sub_distance,
        obstacles=obstacles,
        field_bounds=(0.0, 25.0, 0.0, 25.0)
    )


# ---------------------------------------------------------------------
# MAIN
# ---------------------------------------------------------------------

def main():
    if len(sys.argv) > 1:
        input_dir = sys.argv[1]
        output_dir = sys.argv[2] if len(sys.argv) > 2 else input_dir

        # TODO: replace with real file loading later
        # print(f"Input directory: {input_dir}")
        # print(f"Output directory: {output_dir}")
        
        print("Currently using hardcoded data from table.txt.\n")
        configs_data = get_configs_data()
        field_config = get_field_config_json()

    else:
        print("Using hardcoded data from table.txt.\n")

        configs_data = get_configs_data()
        field_config = get_field_config_json()
        output_dir = "sample_results"

    obstacles = parse_field_config(field_config)
    print(f"Generated {len(obstacles)} obstacles.")

    results = build_multi_station_results(configs_data, obstacles)

    print("\nGenerating plots: ")
    generate_all_multi_station_plots(results, output_dir=output_dir)

    print(f"Plots saved to '{output_dir}'")


if __name__ == "__main__":
    main()