import json
from viz_pipeline import *

def load_json_data(json_path):
    """Load grid search results from JSON file"""
    with open(json_path, 'r') as f:
        data = json.load(f)
    return data


def parse_field_config(data):
    """Parse field configuration from JSON data"""
    field_config = data.get('field_config', {})
    boundaries = field_config.get('field_boundaries_used_in_grid_search', {})
    
    field_bounds = (
        boundaries.get('min_x', 0.0),
        boundaries.get('max_x', 12.0),
        boundaries.get('min_y', 0.0),
        boundaries.get('max_y', 12.0)
    )
    
    return field_bounds

def parse_grid_search_results(data):
    """Parse grid search results from JSON data"""
    grid_search = data.get('grid_search', {})
    results_data = grid_search.get('points', [])

    grid_resolution = grid_search.get('grid_resolution', 50)
    
    results = []
    for point in results_data:
        pos = Pos2(point['x'], point['y'])
        energy = point['energy_consumption']
        total_distance = point['total_distance']
        charging_distance = point['charging_distance']
        
        results.append((pos, energy, total_distance, charging_distance))
    
    return results, grid_resolution


def find_optimization_minimum(data, results):

    """Find the point with minimum energy consumption"""
    # First, check if optimization_minimum is provided in JSON
    if 'optimization_minimum' in data:
        opt_min = data.get('optimization_minimum')
        if opt_min is None:
            return None  

        pos = Pos2(opt_min['x'], opt_min['y'])
        energy = opt_min['energy_consumption']
        # print(f"Using optimization minimum from JSON: ({pos.x:.3f}, {pos.y:.3f})")
        return (pos, energy)
    
    # Fallback: find minimum from results
    if not results:
        return None
    
    min_idx = min(range(len(results)), key=lambda i: results[i][1])
    pos, energy, _, _ = results[min_idx]
    #print(f"Calculated optimization minimum from results: ({pos.x:.3f}, {pos.y:.3f})")
    
    return (pos, energy)

# ============================================================================
# Enhanced version with obstacle support
# ============================================================================

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


def visualize_single_station(json_path, output_dir="results"):
    """
    Enhanced version that creates obstacles from line configurations
    """
  
    # Load JSON data
    data = load_json_data(json_path)
    
    # Parse components
    field_bounds = parse_field_config(data)
    print("-" * 70)
    print(f"Field bounds: x=[{field_bounds[0]:.1f}, {field_bounds[1]:.1f}], "
          f"y=[{field_bounds[2]:.1f}, {field_bounds[3]:.1f}]")
    
    # Try to create obstacles from line configuration
    obstacles = add_obstacles_from_lines(data)
    
    results, grid_resolution = parse_grid_search_results(data)
    
    # Find optimization minimum
    optimization_minimum = find_optimization_minimum(data, results)
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
    print(f"Generating plots...")

    os.makedirs(output_dir, exist_ok=True)
    generate_all_grid_plots(grid_results, optimization_minimum, output_dir=output_dir)


# ============================================================================
# Main execution
# ============================================================================

if __name__ == "__main__":
    import sys
    import os

    if len(sys.argv) < 2:
        print("Usage:")
        print("  python single_station_viz.py <input_json> [output_dir]")
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