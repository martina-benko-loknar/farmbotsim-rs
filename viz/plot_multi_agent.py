"""
Script to visualize multi-agent station configurations from table data
"""

import json
from visualization import (
    Pos2, Obstacle, MultiStationResults,
    generate_all_multi_station_plots
)
import sys

# Your table data
configs_data = [
    ((17.11, 1.29), (12.47, 14.34), 15049.52, 56.90, 46105.30, 735.90),
    ((1.5, 1.5), (23.0, 23.5), 15497.50, 59.28, 47773.37, 1071.48),
    ((1.5, 12.5), (23.0, 12.5), 15726.52, 61.09, 48678.67, 1992.16),
    ((6.25, 1.5), (6.25, 23.5), 15494.18, 58.80, 47766.84, 1079.20),
    ((12.5, 7.5), (12.5, 17.5), 15110.14, 58.04, 46306.12, 892.41),
    ((17.5, 1.5), (20.0, 1.5), 15231.02, 58.49, 46797.55, 800.37),
    ((12.5, 11.5), (12.5, 13.5), 15341.80, 58.53, 47181.80, 945.21),
]

# Field configuration
field_config_json = """
{
  "configs": [
    {
      "Line": {
        "left_top_pos": {
          "x": 2.5,
          "y": 2.5
        },
        "angle": "0.000 deg",
        "n_lines": 15,
        "length": "20.000 m",
        "line_spacing": "0.500 m",
        "farm_entity_plan_path": "configs/farm_entity_plans/default_line.json"
      }
    },
    {
      "Line": {
        "left_top_pos": {
          "x": 15.0,
          "y": 2.5
        },
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

def parse_field_config(config_json: str):
    """
    Parse field configuration and generate obstacles representing farm rows.
    Uses the exact same logic as the Rust implementation (from load_json.py).
    
    Args:
        config_json: JSON string with field configuration
    
    Returns:
        List of Obstacle objects
    """
    config = json.loads(config_json)
    obstacles = []
    
    for line_config in config["configs"]:
        if "Line" in line_config:
            line_data = line_config["Line"]
            
            # Parse parameters
            left_top = line_data["left_top_pos"]
            n_lines = line_data["n_lines"]
            line_spacing = float(line_data["line_spacing"].replace(" m", ""))
            length = float(line_data["length"].replace(" m", ""))
            angle = float(line_data["angle"].replace(" deg", ""))
            
            # Constants from Rust code
            obstacle_width = 0.08  # 8cm
            height_offset = 0.2    # 20cm
            
            # Starting position (Rust logic)
            # pos1 = left_top_pos + Vec2::new((-line_spacing/2.0), -height_offset)
            pos1_x = left_top['x'] - line_spacing / 2.0
            pos1_y = left_top['y'] - height_offset
            
            obstacle_width_half = obstacle_width / 2.0
            
            # Create n_lines + 1 obstacles (matching Rust)
            for i in range(n_lines + 1):
                # pos2 = pos1 + (0, length + 2*height_offset) [since angle=0, 90deg rotation]
                pos2_x = pos1_x
                pos2_y = pos1_y + length + 2 * height_offset
                
                # Four corners of obstacle
                p1 = Pos2(pos1_x - obstacle_width_half, pos1_y)
                p2 = Pos2(pos1_x + obstacle_width_half, pos1_y)
                p3 = Pos2(pos2_x + obstacle_width_half, pos2_y)
                p4 = Pos2(pos2_x - obstacle_width_half, pos2_y)
                
                obstacles.append(Obstacle([p1, p2, p3, p4]))
                
                # Move to next obstacle position
                pos1_x += line_spacing
    
    return obstacles

# Parse obstacles from field configuration (using Rust-style logic)
obstacles = parse_field_config(field_config_json)
print(f"Generated {len(obstacles)} obstacles from field configuration (0.08m wide, Rust-style)")

# Field boundaries from config
field_bounds = (0.0, 25.0, 0.0, 25.0)

# Convert table data to proper format
configs = []
for s1, s2, energy, time, distance, dist_charging in configs_data:
    stations = [Pos2(s1[0], s1[1]), Pos2(s2[0], s2[1])]
    configs.append({
        'stations': stations,
        'energy': energy,
        'time': time,
        'distance': distance,
        'distance_charging': dist_charging
    })

# Find optimal configuration (minimum energy)
optimal_idx = min(range(len(configs)), key=lambda i: configs[i]['energy'])
optimal_config = configs[optimal_idx]

print(f"\nOptimal configuration: Config #{optimal_idx + 1}")
print(f"  Energy: {optimal_config['energy']:.2f} Wh")
print(f"  Distance: {optimal_config['distance']:.2f} m")
print(f"  Stations: {optimal_config['stations']}")

# Create suboptimal configs lists (sorted by energy/distance)
suboptimal_energy = [(c['stations'], c['energy']) 
                     for i, c in enumerate(configs) if i != optimal_idx]
suboptimal_energy.sort(key=lambda x: x[1])

suboptimal_distance = [(c['stations'], c['distance']) 
                       for i, c in enumerate(configs) if i != optimal_idx]
suboptimal_distance.sort(key=lambda x: x[1])

# Create MultiStationResults object
multi_results = MultiStationResults(
    optimal_stations=optimal_config['stations'],
    optimal_energy=optimal_config['energy'],
    optimal_distance=optimal_config['distance'],
    suboptimal_configs_energy=suboptimal_energy,
    suboptimal_configs_distance=suboptimal_distance,
    obstacles=obstacles,
    field_bounds=field_bounds
)

# Generate all plots (both energy and distance based)
print("\nGenerating multi-station plots...")
output_directory = sys.argv[2] if len(sys.argv) > 2 else "results"
print(f"Output directory: {output_directory}")
generate_all_multi_station_plots(multi_results, output_dir="results")

print("\nPlots saved to 'results' directory:")
print("  - multi_station_2_energy.png/pdf")
print("  - multi_station_2_distance.png/pdf")