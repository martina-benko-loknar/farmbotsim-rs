import json
import sys
from viz_models import Pos2, Obstacle, MultiStationResults
from viz_pipeline import generate_all_multi_station_plots

def load_json_data(json_path):
    """Load grid search results from JSON file"""
    with open(json_path, 'r') as f:
        data = json.load(f)
    return data

def add_obstacles_from_lines(data):
    """
    Create obstacle polygons from field line configurations.
    """
    obstacles = []
    
    field_config = data.get('field', {})
    config_raw_str = field_config.get('raw_field_config', '')
    
    if not config_raw_str:
        return obstacles

    config_raw = json.loads(config_raw_str)
    configs = config_raw.get('configs', [])

    VISUAL_OBSTACLE_WIDTH = 0.08 
    VISUAL_HEIGHT_PADDING = 0.2   

    total_obstacles = 0
    n_configs = 0
    
    for config in configs:
        if 'Line' not in config:
                continue
        
        n_configs += 1                
        line_config = config['Line']
        
        # Extract line parameters (matching Rust)
        left_top = line_config['left_top_pos']
        n_lines = line_config['n_lines']
        line_spacing = float(line_config['line_spacing'].split()[0])
        length = float(line_config['length'].split()[0])
        
        # Starting position 
        pos1_x = left_top['x'] - line_spacing / 2.0
        pos1_y = left_top['y'] - VISUAL_HEIGHT_PADDING
        
        obstacle_width_half = VISUAL_OBSTACLE_WIDTH / 2.0
        
        # Create n_lines + 1 obstacles
        for _ in range(n_lines + 1):
            pos2_x = pos1_x
            pos2_y = pos1_y + length + 2 * VISUAL_HEIGHT_PADDING
            
            # Four corners of obstacle
            p1 = Pos2(pos1_x - obstacle_width_half, pos1_y)
            p2 = Pos2(pos1_x + obstacle_width_half, pos1_y)
            p3 = Pos2(pos2_x + obstacle_width_half, pos2_y)
            p4 = Pos2(pos2_x - obstacle_width_half, pos2_y)
            
            obstacles.append(Obstacle([p1, p2, p3, p4]))
            
            # Move to next obstacle position
            pos1_x += line_spacing
        
        n_obstacles = n_lines + 1
        total_obstacles += n_obstacles
        
        print(
            f"Config {n_configs} | "
            f"obstacles: {n_obstacles} | "
            #f"width: {obstacle_width:.2f} m | "
            #f"extension: {height_offset:.2f} m"
        )

    return obstacles

def parse_multi_station_results(data, obstacles):

    evals = data["specialist_layout_evaluations"]["evaluations"]

    layouts = evals["layouts"]

    ego = data["ego_summary"]

    # -------------------------------------------------
    # EGO optimum
    # -------------------------------------------------

    optimal_stations = [
        Pos2(s["x"], s["y"])
        for s in ego["optimal_positions"]
    ]

    optimal_energy = ego["optimal_energy"]

    # Temporary placeholder
    optimal_distance = 0.0

    # -------------------------------------------------
    # Specialist layouts
    # -------------------------------------------------

    suboptimal_energy = []
    suboptimal_distance = []

    for layout in layouts:

        stations = [
            Pos2(s["x"], s["y"])
            for s in layout["layout"]["stations"]
        ]

        energy = layout["energy_wh"]
        distance = layout["total_distance_m"]

        suboptimal_energy.append((stations, energy))
        suboptimal_distance.append((stations, distance))

    return MultiStationResults(
        optimal_stations=optimal_stations,
        optimal_energy=optimal_energy,
        optimal_distance=optimal_distance,
        suboptimal_configs_energy=suboptimal_energy,
        suboptimal_configs_distance=suboptimal_distance,
        obstacles=obstacles,
        field_bounds=(0.0, 25.0, 0.0, 25.0)
    )

# ---------------------------------------------------------------------
# MAIN
# ---------------------------------------------------------------------

def main():
    print("=" * 70)
    print("Multi-station visualization")
    print("=" * 70)

    if len(sys.argv) > 1:
        json_file = sys.argv[1]
        output_directory = sys.argv[2] if len(sys.argv) > 2 else json_file
    else:
        json_file = None
        output_directory = "sample_results"

    print(f"Input json path: {json_file}.")
    print(f"Output directory: {output_directory}")
    print("-" * 70)

    # configs_data = get_configs_data()
    # field_config = get_field_config_json()

    # obstacles = parse_field_config(field_config)
    
    data = load_json_data(json_file)

    obstacles = add_obstacles_from_lines(data)
    print(f"Obstacles: {len(obstacles)}")

    results = parse_multi_station_results(data, obstacles)
    optimal_energy = results.optimal_energy
    optimal_station = results.optimal_stations

    print(f"Optimal energy: {optimal_energy:.2f} Wh")
    print(f"Optimal stations: {optimal_station}") 

    print("-" * 70)
    print("Generating plots... ")
    generate_all_multi_station_plots(results, output_dir=output_directory)

    print("-" * 70)
    print("✓ Plots generated successfully!")
    print(f"Saved to '{output_directory}'")
    print("=" * 70)


if __name__ == "__main__":
    main()