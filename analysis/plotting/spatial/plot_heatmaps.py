
import numpy as np
import matplotlib.pyplot as plt
from typing import List, Tuple, Optional
from viz_utils import setup_latex_fonts, add_obstacles_to_2d_plot, add_optimization_minimum_to_2d_plot
from viz_models import Pos2, Obstacle

# ============================================================================
# 2D Heatmap Plots
# ============================================================================

def generate_energy_heatmap_plot(
    results: List[Tuple[Pos2, float, float, float]], 
    obstacles: List[Obstacle], 
    grid_resolution: int, 
    optimization_minimum: Optional[Tuple[Pos2, float]] = None,
    output_dir: str = "results"
):
    """Generate 2D heatmap plot for energy consumption with discrete grid (like Rust)"""
    setup_latex_fonts(30)
    
    energy_results = [(pos, energy) for pos, energy, _, _ in results]
    
    # Extract unique x and y coordinates
    x_vals = sorted(set(pos.x for pos, _ in energy_results))
    y_vals = sorted(set(pos.y for pos, _ in energy_results))
    
    # Create grid matching actual data points
    Z_grid = np.full((len(y_vals), len(x_vals)), np.nan)
    
    for (pos, val) in energy_results:
        i = y_vals.index(pos.y)
        j = x_vals.index(pos.x)
        Z_grid[i, j] = val
    
    fig, ax = plt.subplots(figsize=(10.75, 10))
    
    # Create discrete heatmap (blue to red)
    im = ax.pcolormesh(x_vals, y_vals, Z_grid, cmap='RdYlBu_r', shading='auto')
    cbar = plt.colorbar(im, ax=ax, fraction=0.046, pad=0.04, shrink=0.8)
    cbar.ax.set_title(r'$E_{\mathrm{tot}}$ (Wh)', fontsize=25, pad=20, loc='left')
    cbar.ax.tick_params(labelsize=25)
    
    add_obstacles_to_2d_plot(ax, obstacles)
    add_optimization_minimum_to_2d_plot(ax, optimization_minimum)
    
    # Add annotation if minimum exists
    if optimization_minimum is not None:
        opt_pos, _ = optimization_minimum
        ax.annotate('minimum', xy=(opt_pos.x, opt_pos.y), xytext=(opt_pos.x, opt_pos.y +  0.75),
                   fontsize=25, ha='center', va='bottom',
                   bbox=dict(boxstyle='round,pad=0.1', facecolor='white', edgecolor='black', linewidth=1))
    
    ax.set_xlabel('$x$ (m)')
    ax.set_ylabel('$y$ (m)')
    #(f'Grid Search Results - Energy Consumption Heatmap ({grid_resolution}x{grid_resolution})', fontsize=25)
    ax.tick_params(labelsize=25)
    
    plt.tight_layout()
    filename = f"{output_dir}/grid_search_{grid_resolution}x{grid_resolution}_energy_heatmap"
    fname = f"grid_search_{grid_resolution}x{grid_resolution}_energy_heatmap"
    #plt.savefig(f"{filename}.png", dpi=150, bbox_inches='tight')
    plt.savefig(f"{filename}.pdf", bbox_inches='tight')
    #print(f"Plot saved to: {filename}.png and {filename}.pdf")
    print(f"- {fname}.pdf")
    plt.close()


def generate_distance_heatmap_plot(
    results: List[Tuple[Pos2, float, float, float]], 
    obstacles: List[Obstacle], 
    grid_resolution: int, 
    optimization_minimum: Optional[Tuple[Pos2, float]] = None,
    output_dir: str = "results"
):
    """Generate 2D heatmap plot for total distance driven with discrete grid (like Rust)"""
    setup_latex_fonts(30)
    
    distance_results = [(pos, total_distance) for pos, _, total_distance, _ in results]
    
    # Extract unique x and y coordinates
    x_vals = sorted(set(pos.x for pos, _ in distance_results))
    y_vals = sorted(set(pos.y for pos, _ in distance_results))
    
    # Create grid matching actual data points
    Z_grid = np.full((len(y_vals), len(x_vals)), np.nan)
    
    for (pos, val) in distance_results:
        i = y_vals.index(pos.y)
        j = x_vals.index(pos.x)
        Z_grid[i, j] = val
    
    fig, ax = plt.subplots(figsize=(10.75, 10))
    
    # Create discrete heatmap (blue to red)
    im = ax.pcolormesh(x_vals, y_vals, Z_grid, cmap='RdYlBu_r', shading='auto')
    cbar = plt.colorbar(im, ax=ax, fraction=0.046, pad=0.04, shrink=0.8)
    cbar.ax.set_title('$d$ (m)', fontsize=25, pad=10, loc='left')
    cbar.ax.tick_params(labelsize=25)
    
    add_obstacles_to_2d_plot(ax, obstacles)
    add_optimization_minimum_to_2d_plot(ax, optimization_minimum)
    
    # Add annotation if minimum exists
    if optimization_minimum is not None:
        opt_pos, _ = optimization_minimum
        ax.annotate('minimum', xy=(opt_pos.x, opt_pos.y), xytext=(opt_pos.x, opt_pos.y +  0.75),
                   fontsize=25, ha='center', va='bottom',
                   bbox=dict(boxstyle='round,pad=0.1', facecolor='white', edgecolor='black', linewidth=1))
    
    ax.set_xlabel('$x$ (m)')
    ax.set_ylabel('$y$ (m)')
    #ax.set_title(f'Grid Search Results - Total Distance Heatmap ({grid_resolution}x{grid_resolution})', fontsize=25)
    ax.tick_params(labelsize=25)
    
    plt.tight_layout()
    filename = f"{output_dir}/grid_search_{grid_resolution}x{grid_resolution}_distance_heatmap"
    fname = f"grid_search_{grid_resolution}x{grid_resolution}_distance_heatmap"
    #plt.savefig(f"{filename}.png", dpi=150, bbox_inches='tight')
    plt.savefig(f"{filename}.pdf", bbox_inches='tight')
    #print(f"Plot saved to: {filename}.png and {filename}.pdf")
    print(f"- {fname}.pdf")
    plt.close()


def generate_charging_distance_heatmap_plot(
    results: List[Tuple[Pos2, float, float, float]], 
    obstacles: List[Obstacle], 
    grid_resolution: int, 
    optimization_minimum: Optional[Tuple[Pos2, float]] = None,
    output_dir: str = "results"
):
    """Generate 2D heatmap plot for charging distance with discrete grid (like Rust)"""
    setup_latex_fonts(30)
    
    charging_results = [(pos, charging_distance) for pos, _, _, charging_distance in results]
    
    # Extract unique x and y coordinates
    x_vals = sorted(set(pos.x for pos, _ in charging_results))
    y_vals = sorted(set(pos.y for pos, _ in charging_results))
    
    # Create grid matching actual data points
    Z_grid = np.full((len(y_vals), len(x_vals)), np.nan)
    
    for (pos, val) in charging_results:
        i = y_vals.index(pos.y)
        j = x_vals.index(pos.x)
        Z_grid[i, j] = val
    
    fig, ax = plt.subplots(figsize=(10.75, 10))
    
    # Create discrete heatmap (blue to red)
    im = ax.pcolormesh(x_vals, y_vals, Z_grid, cmap='RdYlBu_r', shading='auto')
    cbar = plt.colorbar(im, ax=ax, fraction=0.046, pad=0.04, shrink=0.8)
    cbar.ax.set_title(r'$d^{\mathrm{chg}}$ (m)', fontsize=25, pad=10, loc='left')
    cbar.ax.tick_params(labelsize=25)
    
    add_obstacles_to_2d_plot(ax, obstacles)
    add_optimization_minimum_to_2d_plot(ax, optimization_minimum)
    
    # Add annotation if minimum exists
    if optimization_minimum is not None:
        opt_pos, _ = optimization_minimum
        ax.annotate('minimum', xy=(opt_pos.x, opt_pos.y), xytext=(opt_pos.x, opt_pos.y +  0.75),
                   fontsize=25, ha='center', va='bottom',
                   bbox=dict(boxstyle='round,pad=0.1', facecolor='white', edgecolor='black', linewidth=1))
    
    ax.set_xlabel('$x$ (m)')
    ax.set_ylabel('$y$ (m)')
    #ax.set_title(f'Grid Search Results - Charging Distance Heatmap ({grid_resolution}x{grid_resolution})', fontsize=25)
    ax.tick_params(labelsize=25)
    
    plt.tight_layout()
    filename = f"{output_dir}/grid_search_{grid_resolution}x{grid_resolution}_charging_distance_heatmap"
    fname = f"grid_search_{grid_resolution}x{grid_resolution}_charging_distance_heatmap"
    #plt.savefig(f"{filename}.png", dpi=150, bbox_inches='tight')
    plt.savefig(f"{filename}.pdf", bbox_inches='tight')
    #print(f"Plot saved to: {filename}.png and {filename}.pdf")
    print(f"- {fname}.pdf")
    plt.close()

