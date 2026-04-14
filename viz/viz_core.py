"""
Visualization module for grid search and multi-station optimization results.
Translated from Rust to Python using matplotlib.
"""

import numpy as np
import matplotlib.pyplot as plt
from matplotlib import cm
from matplotlib.patches import Polygon as MplPolygon
from mpl_toolkits.mplot3d import Axes3D
from scipy.interpolate import griddata
from dataclasses import dataclass
from typing import List, Tuple, Optional
import os


# ============================================================================
# Data Structures
# ============================================================================

@dataclass
class Pos2:
    """2D position"""
    x: float
    y: float


@dataclass
class Obstacle:
    """Obstacle defined by polygon points"""
    points: List[Pos2]


@dataclass
class GridSearchResults:
    """Data structure to hold grid search results for visualization"""
    results: List[Tuple[Pos2, float, float, float]]  # (position, energy, total_distance, charging_distance)
    grid_resolution: int
    obstacles: List[Obstacle]
    field_bounds: Tuple[float, float, float, float]  # (min_x, max_x, min_y, max_y)


@dataclass
class MultiStationResults:
    """Data structure for multi-station configuration results"""
    optimal_stations: List[Pos2]
    optimal_energy: float
    optimal_distance: float
    suboptimal_configs_energy: List[Tuple[List[Pos2], float]]
    suboptimal_configs_distance: List[Tuple[List[Pos2], float]]
    obstacles: List[Obstacle]
    field_bounds: Tuple[float, float, float, float]


# ============================================================================
# LaTeX-style Font Configuration
# ============================================================================

def setup_latex_fonts(font_size=30):
    """Configure matplotlib to use LaTeX-style fonts"""
    plt.rcParams['text.usetex'] = True  # Enable LaTeX rendering
    plt.rcParams['font.family'] = 'serif'
    plt.rcParams['font.serif'] = ['Computer Modern Roman']
    plt.rcParams['font.size'] = font_size
    plt.rcParams['axes.labelsize'] = font_size
    plt.rcParams['axes.titlesize'] = font_size
    plt.rcParams['xtick.labelsize'] = font_size
    plt.rcParams['ytick.labelsize'] = font_size
    plt.rcParams['legend.fontsize'] = font_size


# ============================================================================
# Main Plot Generation Functions
# ============================================================================

def generate_all_grid_plots(
    results: GridSearchResults,
    optimization_minimum: Optional[Tuple[Pos2, float]] = None,
    output_dir: str = "results"
):
    """Generate all plots for grid search results"""
    os.makedirs(output_dir, exist_ok=True)
    
    generate_3d_plot(
        results.results, 
        results.obstacles, 
        results.grid_resolution, 
        optimization_minimum,
        output_dir
    )
    generate_energy_heatmap_plot(
        results.results, 
        results.obstacles, 
        results.grid_resolution, 
        optimization_minimum,
        output_dir
    )
    generate_distance_heatmap_plot(
        results.results, 
        results.obstacles, 
        results.grid_resolution, 
        optimization_minimum,
        output_dir
    )
    generate_charging_distance_heatmap_plot(
        results.results, 
        results.obstacles, 
        results.grid_resolution, 
        optimization_minimum,
        output_dir
    )


def generate_all_multi_station_plots(
    results: MultiStationResults,
    output_dir: str = "results"
):
    """Generate all plots for multi-station results"""
    os.makedirs(output_dir, exist_ok=True)
    
    generate_multi_station_plot(
        results.optimal_stations,
        results.optimal_energy,
        results.suboptimal_configs_energy,
        results.obstacles,
        results.field_bounds,
        output_dir
    )
    
    generate_multi_station_distance_plot(
        results.optimal_stations,
        results.optimal_distance,
        results.suboptimal_configs_distance,
        results.obstacles,
        results.field_bounds,
        output_dir
    )


# ============================================================================
# 3D Plot
# ============================================================================

def generate_3d_plot(
    results: List[Tuple[Pos2, float, float, float]], 
    obstacles: List[Obstacle], 
    grid_resolution: int, 
    optimization_minimum: Optional[Tuple[Pos2, float]] = None,
    output_dir: str = "results"
):
    """Generate 3D scatter plot"""
    setup_latex_fonts(30)
    
    x_coords = [pos.x for pos, _, _, _ in results]
    y_coords = [pos.y for pos, _, _, _ in results]
    energy_values = [energy for _, energy, _, _ in results]
    
    # Find the optimal (minimum) energy value for obstacle z-level
    min_energy = min(energy_values)
    
    fig = plt.figure(figsize=(12, 8))
    ax = fig.add_subplot(111, projection='3d')
    
    # Plot grid points
    ax.scatter(x_coords, y_coords, energy_values, c='blue', marker='o', s=4, alpha=0.6)
    
    # Add optimization minimum point if provided
    if optimization_minimum is not None:
        opt_pos, opt_value = optimization_minimum
        # White padding cross (larger, underneath)
        ax.scatter([opt_pos.x], [opt_pos.y], [opt_value], 
                  c='white', marker='x', s=400, linewidths=3, 
                  label='_nolegend_', zorder=100)
        # Black cross on top (thinner)
        ax.scatter([opt_pos.x], [opt_pos.y], [opt_value], 
                  c='black', marker='x', s=225, linewidths=2, 
                  label='Optimization Minimum', zorder=101)
    
    # Add obstacle boundaries as 3D traces
    add_obstacles_to_3d_plot(ax, obstacles, min_energy)
    
    ax.set_xlabel('$x$ (m)')
    ax.set_ylabel('$y$ (m)')
    ax.set_zlabel('Energy (Wh)')
    #ax.set_title(f'Grid Search Results - 3D View ({grid_resolution}x{grid_resolution})')
    ax.legend()
    
    plt.tight_layout()
    filename = f"{output_dir}/grid_search_{grid_resolution}x{grid_resolution}_3d"
    # plt.savefig(f"{filename}.png", dpi=150, bbox_inches='tight')
    plt.savefig(f"{filename}.pdf", bbox_inches='tight')
    # print(f"Plot saved to: {filename}.png and {filename}.pdf")
    print(f"Plot saved to: {filename}.pdf")
    plt.close()


def add_obstacles_to_3d_plot(ax, obstacles: List[Obstacle], z_level: float):
    """Add obstacle boundaries to 3D plot"""
    for obstacle in obstacles:
        if len(obstacle.points) < 2:
            continue
        
        # Create closed polygon
        x_coords = [p.x for p in obstacle.points] + [obstacle.points[0].x]
        y_coords = [p.y for p in obstacle.points] + [obstacle.points[0].y]
        z_coords = [z_level] * len(x_coords)
        
        ax.plot(x_coords, y_coords, z_coords, 'k-', linewidth=2, label='Obstacle' if obstacles.index(obstacle) == 0 else '_nolegend_')


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
    #plt.savefig(f"{filename}.png", dpi=150, bbox_inches='tight')
    plt.savefig(f"{filename}.pdf", bbox_inches='tight')
    #print(f"Plot saved to: {filename}.png and {filename}.pdf")
    print(f"Plot saved to: {filename}.pdf")
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
    #plt.savefig(f"{filename}.png", dpi=150, bbox_inches='tight')
    plt.savefig(f"{filename}.pdf", bbox_inches='tight')
    #print(f"Plot saved to: {filename}.png and {filename}.pdf")
    print(f"Plot saved to: {filename}.pdf")
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
    #plt.savefig(f"{filename}.png", dpi=150, bbox_inches='tight')
    plt.savefig(f"{filename}.pdf", bbox_inches='tight')
    #print(f"Plot saved to: {filename}.png and {filename}.pdf")
    print(f"Plot saved to: {filename}.pdf")
    plt.close()


# ============================================================================
# Multi-Station Plots
# ============================================================================

def generate_multi_station_plot(
    optimal_stations: List[Pos2],
    optimal_energy: float,
    suboptimal_configs: List[Tuple[List[Pos2], float]],
    obstacles: List[Obstacle],
    field_bounds: Tuple[float, float, float, float],
    output_dir: str = "results"
):
    """Generate multi-station configuration plot (energy-based)"""
    setup_latex_fonts(30)
    
    fig, ax = plt.subplots(figsize=(10.75, 10))

    # Calculate total number of configs for color mapping
    n_configs = len(suboptimal_configs)

    # Plot optimal configuration
    x_coords = [s.x for s in optimal_stations]
    y_coords = [s.y for s in optimal_stations]
    optimal_color = cm.RdYlBu_r(0) 
    # ax.scatter(x_coords, y_coords, c=cm.hsv(1), marker='*', s=300, cmap='RdYlBu_r',
    #           edgecolors='black', linewidths=1, 
    #           label=f'{optimal_energy/1000:.2f}', zorder=100)
    ax.scatter(x_coords, y_coords, c=[optimal_color], marker='*', s=300,
              edgecolors='black', linewidths=1, 
              label=f'{optimal_energy/1000:.2f}', zorder=100)

    # Plot suboptimal configurations
    for i, (stations, energy) in enumerate(suboptimal_configs):
        x_coords = [s.x for s in stations]
        y_coords = [s.y for s in stations]
        # For each suboptimal config
        #color_value = i / 7  # Normalize to 0-1 range
        color_value = (i+1) / 7
        color = cm.RdYlBu_r(color_value)
        #color = cm.hsv(color_value)  # Get RGB color from colormap
        alpha = 0.9 - (i * 0.05)
        markers = ['o', '^', '<', 'p', 's', 'D']
        ax.scatter(x_coords, y_coords, c=[color], marker=markers[i], s=150, cmap='RdYlBu_r',
                  alpha=0.9, edgecolors='black', linewidths=1, 
                  label=f'{energy/1000:.2f}')
                  #label=f'Cfg.{i+1} ({energy/1000:.2f} kWh)')

    
    
    # Add obstacles and field boundaries
    add_obstacles_to_2d_plot(ax, obstacles)
    #add_field_boundaries_to_plot(ax, field_bounds)
    
    ax.set_xlabel('$x$ (m)')
    ax.set_ylabel('$y$ (m)')
    #ax.set_title(f'Multi-Station Optimization - Energy ({len(optimal_stations)} stations)', fontsize=25)
    ax.tick_params(labelsize=25)
    
    # Set axis limits to match field bounds for consistency with heatmaps
    #min_x, max_x, min_y, max_y = field_bounds
    #ax.set_xlim(min_x, max_x)
    #ax.set_ylim(min_y, max_y)
    ax.set_aspect('equal', adjustable='box')  # Equal aspect ratio like heatmaps
    
    legend = ax.legend(loc='upper center', 
              bbox_to_anchor=(1.1, 0.98),
              fontsize=25,  
              borderpad=0,
              labelspacing=0.35,
              handletextpad=-0.25,
              frameon=False,
              title=r'$E_{\mathrm{tot}}$ (kWh)', 
              title_fontsize=25,
              alignment='left'
              )
    #legend.get_title().set_ha('right') 
    legend._legend_box.sep = 20
    ax.grid(False)
    ax.spines['top'].set_visible(True)
    ax.spines['right'].set_visible(True)
    ax.spines['bottom'].set_color('black')
    ax.spines['left'].set_color('black')
    
    plt.tight_layout()
    filename = f"{output_dir}/multi_station_{len(optimal_stations)}_energy"
    #plt.savefig(f"{filename}.png", dpi=150, bbox_inches='tight')
    plt.savefig(f"{filename}.pdf", bbox_inches='tight')
    #print(f"Plot saved to: {filename}.png and {filename}.pdf")
    print(f"Plot saved to: {filename}.pdf")
    plt.close()


def generate_multi_station_distance_plot(
    optimal_stations: List[Pos2],
    optimal_distance: float,
    suboptimal_configs: List[Tuple[List[Pos2], float]],
    obstacles: List[Obstacle],
    field_bounds: Tuple[float, float, float, float],
    output_dir: str = "results"
):
    """Generate multi-station configuration plot (distance-based)"""
    setup_latex_fonts(30)
    
    fig, ax = plt.subplots(figsize=(10.75, 10))
    
    # # Plot suboptimal configurations
    # for i, (stations, distance) in enumerate(suboptimal_configs):  # Top 5
    #     x_coords = [s.x for s in stations]
    #     y_coords = [s.y for s in stations]
    #     alpha = 0.3 - (i * 0.05)
    #     ax.scatter(x_coords, y_coords, c='lightgreen', marker='o', s=100, 
    #               alpha=alpha, edgecolors='green', linewidths=1, 
    #               label=f'Cfg.{i+1} ({distance:.2f} m)')

    # Plot optimal configuration
    x_coords = [s.x for s in optimal_stations]
    y_coords = [s.y for s in optimal_stations]
    optimal_color = cm.RdYlBu_r(0) 
    # ax.scatter(x_coords, y_coords, c=cm.hsv(1), marker='*', s=300, cmap='RdYlBu_r',
    #           edgecolors='black', linewidths=1, 
    #           label=f'{optimal_energy/1000:.2f}', zorder=100)
    ax.scatter(x_coords, y_coords, c=[optimal_color], marker='*', s=300,
              edgecolors='black', linewidths=1, 
              label=f'{optimal_distance/1000:.3f}', zorder=100)

    # Plot suboptimal configurations
    for i, (stations, distance) in enumerate(suboptimal_configs):
        x_coords = [s.x for s in stations]
        y_coords = [s.y for s in stations]
        # For each suboptimal config
        #color_value = i / 7  # Normalize to 0-1 range
        color_value = (i+1) / 7
        color = cm.RdYlBu_r(color_value)
        #color = cm.hsv(color_value)  # Get RGB color from colormap
        alpha = 0.9 - (i * 0.05)
        markers = ['o', '^', '<', 'p', 's', 'D']
        ax.scatter(x_coords, y_coords, c=[color], marker=markers[i], s=150, cmap='RdYlBu_r',
                  alpha=0.9, edgecolors='black', linewidths=1, 
                  label=f'{distance/1000:.3f}')
                  #label=f'Cfg.{i+1} ({energy/1000:.2f} kWh)')
                  #    
    # # Plot optimal configuration
    # x_coords = [s.x for s in optimal_stations]
    # y_coords = [s.y for s in optimal_stations]
    # ax.scatter(x_coords, y_coords, c='red', marker='*', s=300, 
    #           edgecolors='darkred', linewidths=2, 
    #           label=f'Opt. ({optimal_distance:.2f} m)', zorder=100)
    
    # Add obstacles and field boundaries
    add_obstacles_to_2d_plot(ax, obstacles)
    #add_field_boundaries_to_plot(ax, field_bounds)
    
    ax.set_xlabel('$x$ (m)')
    ax.set_ylabel('$y$ (m)')
    #ax.set_title(f'Multi-Station Optimization - Distance ({len(optimal_stations)} stations)', fontsize=25)
    ax.tick_params(labelsize=25)

    ax.set_aspect('equal', adjustable='box')  # Equal aspect ratio like heatmaps
    
    legend = ax.legend(loc='upper center', 
              bbox_to_anchor=(1.1, 0.98),
              fontsize=25,  
              borderpad=0,
              labelspacing=0.35,
              handletextpad=-0.25,
              frameon=False,
              title=r'$d$ (km)', 
              title_fontsize=25,
              alignment='center'
              )
    #legend.get_title().set_ha('left')  
    legend._legend_box.sep = 20
    ax.grid(False)
    ax.spines['top'].set_visible(True)
    ax.spines['right'].set_visible(True)
    ax.spines['bottom'].set_color('black')
    ax.spines['left'].set_color('black')
    
    plt.tight_layout()
    filename = f"{output_dir}/multi_station_{len(optimal_stations)}_distance"
    #plt.savefig(f"{filename}.png", dpi=150, bbox_inches='tight')
    plt.savefig(f"{filename}.pdf", bbox_inches='tight')
    #print(f"Plot saved to: {filename}.png and {filename}.pdf")
    print(f"Plot saved to: {filename}.pdf")
    plt.close()


# ============================================================================
# Helper Functions for 2D Plots
# ============================================================================

def add_obstacles_to_2d_plot(ax, obstacles: List[Obstacle]):
    """Add obstacle polygons to 2D plot"""
    for i, obstacle in enumerate(obstacles):
        if len(obstacle.points) < 3:
            continue
        
        # Create polygon
        points = [(p.x, p.y) for p in obstacle.points]
        polygon = MplPolygon(points, closed=True, edgecolor='black', facecolor='black', #hatch='///', 
                            linewidth=1.5)
        ax.add_patch(polygon)


def add_optimization_minimum_to_2d_plot(
    ax, 
    optimization_minimum: Optional[Tuple[Pos2, float]],
    marker_size: int = 15,
    marker_color: str = 'black'
):
    """Add optimization minimum marker to 2D plot"""
    if optimization_minimum is not None:
        opt_pos, _ = optimization_minimum
        # Layer 1: Larger white circle with black border (background)
        ax.scatter([opt_pos.x], [opt_pos.y], c='white', marker='o', s=400, 
                  edgecolors='black', linewidths=3.5, zorder=100)
        # Layer 2: Black circle on top
        ax.scatter([opt_pos.x], [opt_pos.y], c='black', marker='o', s=250, 
                  edgecolors='white', linewidths=2.5, label='Optimization Minimum', zorder=101)


def add_field_boundaries_to_plot(ax, field_bounds: Tuple[float, float, float, float]):
    """Add field boundary rectangle to plot"""
    min_x, max_x, min_y, max_y = field_bounds
    boundary_x = [min_x, max_x, max_x, min_x, min_x]
    boundary_y = [min_y, min_y, max_y, max_y, min_y]
    ax.plot(boundary_x, boundary_y, 'k-', linewidth=2, label='Field Boundary')


# ============================================================================
# Interpolation Functions
# ============================================================================

def interpolate_results(
    results: List[Tuple[Pos2, float]], 
    obstacles: List[Obstacle], 
    resolution: int
) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
    """
    Interpolate grid search results to create smooth heatmap
    
    Returns:
        x_grid, y_grid, z_grid for use with contourf
    """
    # Extract coordinates and values
    points = np.array([[pos.x, pos.y] for pos, _ in results])
    values = np.array([value for _, value in results])
    
    # Create interpolation grid
    x_min, x_max = points[:, 0].min(), points[:, 0].max()
    y_min, y_max = points[:, 1].min(), points[:, 1].max()
    
    x_grid = np.linspace(x_min, x_max, resolution)
    y_grid = np.linspace(y_min, y_max, resolution)
    X, Y = np.meshgrid(x_grid, y_grid)
    
    # Interpolate using cubic method
    Z = griddata(points, values, (X, Y), method='cubic')
    
    # Mask points inside obstacles
    margin = 0.0
    for i in range(resolution):
        for j in range(resolution):
            pos = Pos2(X[i, j], Y[i, j])
            if not is_position_valid(pos, obstacles, margin):
                Z[i, j] = np.nan
    
    return X, Y, Z


# ============================================================================
# Geometric Helper Functions
# ============================================================================

def is_position_valid(position: Pos2, obstacles: List[Obstacle], margin: float) -> bool:
    """Check if a position is valid (not inside or too close to obstacles)"""
    for obstacle in obstacles:
        # Check if point is inside polygon
        if is_point_inside_polygon(position, obstacle.points):
            return False
        
        # Check distance to each edge
        for i in range(len(obstacle.points)):
            p1 = obstacle.points[i]
            p2 = obstacle.points[(i + 1) % len(obstacle.points)]
            distance = point_to_line_distance(position, p1, p2)
            if distance < margin:
                return False
    
    return True


def is_point_inside_polygon(point: Pos2, polygon: List[Pos2]) -> bool:
    """Ray casting algorithm to determine if point is inside polygon"""
    if len(polygon) < 3:
        return False
    
    inside = False
    j = len(polygon) - 1
    
    for i in range(len(polygon)):
        pi = polygon[i]
        pj = polygon[j]
        
        if ((pi.y > point.y) != (pj.y > point.y)) and \
           (point.x < (pj.x - pi.x) * (point.y - pi.y) / (pj.y - pi.y) + pi.x):
            inside = not inside
        j = i
    
    return inside


def point_to_line_distance(point: Pos2, line_start: Pos2, line_end: Pos2) -> float:
    """Calculate the shortest distance from a point to a line segment"""
    line_vec_x = line_end.x - line_start.x
    line_vec_y = line_end.y - line_start.y
    point_vec_x = point.x - line_start.x
    point_vec_y = point.y - line_start.y
    
    line_len_sq = line_vec_x ** 2 + line_vec_y ** 2
    if line_len_sq == 0.0:
        return np.sqrt(point_vec_x ** 2 + point_vec_y ** 2)
    
    t = (point_vec_x * line_vec_x + point_vec_y * line_vec_y) / line_len_sq
    t = np.clip(t, 0.0, 1.0)
    
    projection_x = line_start.x + t * line_vec_x
    projection_y = line_start.y + t * line_vec_y
    
    dist_vec_x = point.x - projection_x
    dist_vec_y = point.y - projection_y
    
    return np.sqrt(dist_vec_x ** 2 + dist_vec_y ** 2)


# ============================================================================
# Example Usage
# ============================================================================

if __name__ == "__main__":
    # Example: Create sample data and generate plots
    
    # Define obstacles
    obstacle1 = Obstacle([
        Pos2(2.0, 2.0),
        Pos2(4.0, 2.0),
        Pos2(4.0, 4.0),
        Pos2(2.0, 4.0)
    ])
    
    obstacle2 = Obstacle([
        Pos2(6.0, 6.0),
        Pos2(8.0, 6.0),
        Pos2(8.0, 8.0),
        Pos2(6.0, 8.0)
    ])
    
    obstacles = [obstacle1, obstacle2]
    
    # Generate sample grid search results
    grid_resolution = 20
    results = []
    for i in range(grid_resolution):
        for j in range(grid_resolution):
            x = i * 10.0 / grid_resolution
            y = j * 10.0 / grid_resolution
            pos = Pos2(x, y)
            
            # Skip if inside obstacle
            if not is_position_valid(pos, obstacles, 0.0):
                continue
            
            # Calculate sample values (for demonstration)
            energy = 100 + (x - 5) ** 2 + (y - 5) ** 2
            total_distance = 50 + np.sqrt((x - 5) ** 2 + (y - 5) ** 2) * 10
            charging_distance = 20 + np.sqrt((x - 5) ** 2 + (y - 5) ** 2) * 5
            
            results.append((pos, energy, total_distance, charging_distance))
    
    # Create GridSearchResults
    grid_results = GridSearchResults(
        results=results,
        grid_resolution=grid_resolution,
        obstacles=obstacles,
        field_bounds=(0.0, 10.0, 0.0, 10.0)
    )
    
    # Find minimum energy point
    min_idx = min(range(len(results)), key=lambda i: results[i][1])
    optimization_minimum = (results[min_idx][0], results[min_idx][1])
    
    # Generate all plots
    print("Generating grid search plots...")
    generate_all_grid_plots(grid_results, optimization_minimum)
    
    # Example multi-station results
    optimal_stations = [Pos2(3.0, 3.0), Pos2(7.0, 7.0)]
    suboptimal_configs = [
        ([Pos2(2.5, 3.5), Pos2(7.5, 6.5)], 185.0),
        ([Pos2(3.5, 2.5), Pos2(6.5, 7.5)], 190.0),
        ([Pos2(3.0, 4.0), Pos2(7.0, 6.0)], 195.0),
    ]
    
    multi_results = MultiStationResults(
        optimal_stations=optimal_stations,
        optimal_energy=180.0,
        optimal_distance=45.0,
        suboptimal_configs_energy=suboptimal_configs,
        suboptimal_configs_distance=suboptimal_configs,
        obstacles=obstacles,
        field_bounds=(0.0, 10.0, 0.0, 10.0)
    )
    
    print("Generating multi-station plots...")
    generate_all_multi_station_plots(multi_results)
    
    print("\nAll plots generated successfully!")