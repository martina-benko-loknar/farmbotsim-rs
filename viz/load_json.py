"""
Load and visualize grid search results from JSON file
"""

import json
import numpy as np
from visualization import *


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


def parse_obstacles(data):
    """
    Parse obstacles from field configuration.
    Note: The JSON excerpt doesn't show obstacle details, but we know there are 13 obstacles.
    This function will need to be adapted based on the full JSON structure.
    """
    # Placeholder: You'll need to adapt this based on your actual JSON structure
    # For now, return empty list since obstacles aren't in the excerpt
    
    field_config = data.get('field_config', {})
    
    # Try to parse from field_config_raw if available
    config_raw = field_config.get('field_config_raw', '')
    
    # TODO: Parse obstacles from the actual JSON structure
    # The excerpt shows num_obstacles: 13, but doesn't show the obstacle data
    
    obstacles = []
    
    # If you have obstacle data in the JSON, add parsing logic here
    # Example format might be:
    # if 'obstacles' in data:
    #     for obs_data in data['obstacles']:
    #         points = [Pos2(p['x'], p['y']) for p in obs_data['points']]
    #         obstacles.append(Obstacle(points))
    
    return obstacles


def parse_grid_search_results(data):
    """Parse grid search results from JSON data"""
    results_data = data.get('grid_search_results', [])
    grid_resolution = data.get('grid_resolution', 50)
    
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
        opt_min = data['optimization_minimum']
        pos = Pos2(opt_min['x'], opt_min['y'])
        energy = opt_min['energy_consumption']
        print(f"Using optimization minimum from JSON: ({pos.x:.3f}, {pos.y:.3f})")
        return (pos, energy)
    
    # Fallback: find minimum from results
    if not results:
        return None
    
    min_idx = min(range(len(results)), key=lambda i: results[i][1])
    pos, energy, _, _ = results[min_idx]
    print(f"Calculated optimization minimum from results: ({pos.x:.3f}, {pos.y:.3f})")
    
    return (pos, energy)


def visualize_json_data(json_path, output_dir="results"):
    """
    Main function to load JSON data and generate all visualizations
    
    Args:
        json_path: Path to the JSON file
        output_dir: Directory to save output plots
    """
    print("=" * 70)
    print(f"Loading data from: {json_path}")
    print("=" * 70)
    
    # Load JSON data
    data = load_json_data(json_path)
    
    # Parse components
    field_bounds = parse_field_config(data)
    print(f"\nField bounds: x=[{field_bounds[0]:.1f}, {field_bounds[1]:.1f}], "
          f"y=[{field_bounds[2]:.1f}, {field_bounds[3]:.1f}]")
    
    obstacles = parse_obstacles(data)
    print(f"Number of obstacles: {len(obstacles)}")
    
    results, grid_resolution = parse_grid_search_results(data)
    print(f"Grid resolution: {grid_resolution}x{grid_resolution}")
    print(f"Number of grid points: {len(results)}")
    
    # Find optimization minimum
    optimization_minimum = find_optimization_minimum(data, results)
    if optimization_minimum:
        pos, energy = optimization_minimum
        print(f"\nOptimal position: ({pos.x:.3f}, {pos.y:.3f})")
        print(f"Minimum energy: {energy:.2f} Wh")
        
        # Find min/max values for context
        energies = [e for _, e, _, _ in results]
        distances = [d for _, _, d, _ in results]
        charging_dists = [cd for _, _, _, cd in results]
        
        print(f"\nEnergy range: [{min(energies):.2f}, {max(energies):.2f}] Wh")
        print(f"Distance range: [{min(distances):.2f}, {max(distances):.2f}] m")
        print(f"Charging distance range: [{min(charging_dists):.2f}, {max(charging_dists):.2f}] m")
    
    # Create GridSearchResults object
    grid_results = GridSearchResults(
        results=results,
        grid_resolution=grid_resolution,
        obstacles=obstacles,
        field_bounds=field_bounds
    )
    
    # Generate all plots
    print(f"\nGenerating plots in '{output_dir}/' directory...")
    print("-" * 70)
    
    os.makedirs(output_dir, exist_ok=True)
    generate_all_grid_plots(grid_results, optimization_minimum, output_dir=output_dir)
    
    print("-" * 70)
    print("✓ All plots generated successfully!")
    print("=" * 70)


def create_sample_json_data(output_path="sample_grid_search.json"):
    """
    Create a sample JSON file based on the provided excerpt format.
    This is useful for testing.
    """
    # Generate sample grid search results
    grid_resolution = 30
    results = []
    
    for i in range(grid_resolution):
        for j in range(grid_resolution):
            x = 0.4 + i * 11.6 / (grid_resolution - 1)
            y = 0.4 + j * 11.6 / (grid_resolution - 1)
            
            # Simulate energy consumption (parabolic function)
            energy = 6800 + ((x - 6)**2 + (y - 6)**2) * 10
            
            # Simulate distances
            total_distance = 19200 + ((x - 6)**2 + (y - 6)**2) * 5
            charging_distance = 550 + np.sqrt((x - 6)**2 + (y - 6)**2) * 10
            
            results.append({
                "x": x,
                "y": y,
                "energy_consumption": energy + np.random.normal(0, 10),
                "total_distance": total_distance + np.random.normal(0, 20),
                "charging_distance": charging_distance + np.random.normal(0, 5)
            })
    
    # Create full JSON structure
    data = {
        "field_config": {
            "field_boundaries_used_in_grid_search": {
                "min_x": 0.0,
                "max_x": 12.0,
                "min_y": 0.0,
                "max_y": 12.0
            },
            "field_config_path": "configs/field_configs/default.json",
            "num_field_configs": 1,
            "num_obstacles": 0
        },
        "generated_at": "2025-09-24T12:51:46.303097561+00:00",
        "grid_resolution": grid_resolution,
        "grid_search_results": results
    }
    
    # Save to file
    with open(output_path, 'w') as f:
        json.dump(data, f, indent=2)
    
    print(f"Sample JSON file created: {output_path}")
    return output_path


# ============================================================================
# Enhanced version with obstacle support
# ============================================================================

def add_obstacles_from_lines(data):
    """
    Create obstacle polygons from field line configurations.
    Replicates the exact Rust logic from field_config.get_obstacles()
    """
    obstacles = []
    
    try:
        # Parse the field_config_raw to get line configurations
        field_config = data.get('field_config', {})
        config_raw_str = field_config.get('field_config_raw', '')
        
        if config_raw_str:
            config_raw = json.loads(config_raw_str)
            configs = config_raw.get('configs', [])
            
            for config in configs:
                if 'Line' in config:
                    line_config = config['Line']
                    
                    # Extract line parameters (matching Rust)
                    left_top = line_config['left_top_pos']
                    n_lines = line_config['n_lines']
                    line_spacing = float(line_config['line_spacing'].split()[0])
                    length = float(line_config['length'].split()[0])
                    angle = 0.0  # angle is 0.000 deg in your config
                    
                    # Constants from Rust code
                    obstacle_width = 0.08  # 8cm
                    height_offset = 0.2    # 20cm
                    
                    # Starting position (Rust logic)
                    # pos1 = left_top_pos + Vec2::new((-line_spacing/2.0), -height_offset)
                    pos1_x = left_top['x'] - line_spacing / 2.0
                    pos1_y = left_top['y'] - height_offset
                    
                    obstacle_width_half = obstacle_width / 2.0
                    
                    # Create n_lines + 1 obstacles
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
                    
                    print(f"Created {n_lines + 1} obstacles (Rust-style, {obstacle_width}m wide, extended by {height_offset}m)")
    
    except Exception as e:
        print(f"Note: Could not parse obstacles from config: {e}")
    
    return obstacles


def visualize_json_data_with_lines(json_path, output_dir="results"):
    """
    Enhanced version that creates obstacles from line configurations
    """
    print("=" * 70)
    print(f"Loading data from: {json_path}")
    print("=" * 70)
    
    # Load JSON data
    data = load_json_data(json_path)
    
    # Parse components
    field_bounds = parse_field_config(data)
    print(f"\nField bounds: x=[{field_bounds[0]:.1f}, {field_bounds[1]:.1f}], "
          f"y=[{field_bounds[2]:.1f}, {field_bounds[3]:.1f}]")
    
    # Try to create obstacles from line configuration
    obstacles = add_obstacles_from_lines(data)
    print(f"Number of obstacles: {len(obstacles)}")
    
    results, grid_resolution = parse_grid_search_results(data)
    print(f"Grid resolution: {grid_resolution}x{grid_resolution}")
    print(f"Number of grid points: {len(results)}")
    
    # Find optimization minimum
    optimization_minimum = find_optimization_minimum(data, results)
    if optimization_minimum:
        pos, energy = optimization_minimum
        print(f"\nOptimal position: ({pos.x:.3f}, {pos.y:.3f})")
        print(f"Minimum energy: {energy:.2f} Wh")
        
        # Statistics
        energies = [e for _, e, _, _ in results]
        distances = [d for _, _, d, _ in results]
        charging_dists = [cd for _, _, _, cd in results]
        
        print(f"\nEnergy range: [{min(energies):.2f}, {max(energies):.2f}] Wh")
        print(f"Distance range: [{min(distances):.2f}, {max(distances):.2f}] m")
        print(f"Charging distance range: [{min(charging_dists):.2f}, {max(charging_dists):.2f}] m")
    
    # Create GridSearchResults object
    grid_results = GridSearchResults(
        results=results,
        grid_resolution=grid_resolution,
        obstacles=obstacles,
        field_bounds=field_bounds
    )
    
    # Generate all plots
    print(f"\nGenerating plots in '{output_dir}/' directory...")
    print("-" * 70)
    
    os.makedirs(output_dir, exist_ok=True)
    generate_all_grid_plots(grid_results, optimization_minimum, output_dir=output_dir)
    
    print("-" * 70)
    print("✓ All plots generated successfully!")
    print("=" * 70)


# ============================================================================
# Main execution
# ============================================================================

if __name__ == "__main__":
    import sys
    
    if len(sys.argv) > 1:
        # User provided a JSON file path
        json_file = sys.argv[1]
        output_directory = sys.argv[2] if len(sys.argv) > 2 else "results"
        
        print(f"Using JSON file: {json_file}")
        print(f"Output directory: {output_directory}")
        
        # Use the enhanced version that handles line obstacles
        visualize_json_data_with_lines(json_file, output_directory)
    
    else:
        # No file provided - create and use sample data
        print("No JSON file provided. Creating sample data...\n")
        sample_file = create_sample_json_data()
        print()
        visualize_json_data(sample_file, "sample_results")
        print("\nTo visualize your own JSON file, run:")
        print("  python load_json.py your_file.json [output_directory]")