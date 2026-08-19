import matplotlib.pyplot as plt
from matplotlib.patches import Polygon as MplPolygon
from typing import List, Tuple, Optional
from viz_models import Pos2, Obstacle

# ================================================================
# LaTeX-style Font Configuration
# ================================================================

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


# ================================================================
# Helper Functions for 2D and 3D Plots
# =================================================================

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
    """Add grid-search optimum marker to 2D plot"""
    if optimization_minimum is not None:
        opt_pos, _ = optimization_minimum
        # Layer 1: Larger white circle with black border (background)
        ax.scatter([opt_pos.x], [opt_pos.y], c='white', marker='o', s=400,
                  edgecolors='black', linewidths=3.5, zorder=100)
        # Layer 2: Black circle on top
        ax.scatter([opt_pos.x], [opt_pos.y], c='black', marker='o', s=250,
                  edgecolors='white', linewidths=2.5, label='Grid search optimum', zorder=101)


def add_ego_best_to_2d_plot(
    ax,
    ego_best: Optional[Tuple[Pos2, float]],
):
    """
    Add EGO's best-found position to a 2D plot, distinguished from the grid
    search optimum (circle, see add_optimization_minimum_to_2d_plot) by a
    star marker rather than color, since the same black/white layering is
    used for both so they still read cleanly on the RdYlBu_r heatmaps.
    """
    if ego_best is not None:
        pos, _ = ego_best
        # Layer 1: Larger white star with black border (background)
        ax.scatter([pos.x], [pos.y], c='white', marker='*', s=650,
                  edgecolors='black', linewidths=3, zorder=102)
        # Layer 2: Black star on top
        ax.scatter([pos.x], [pos.y], c='black', marker='*', s=380,
                  edgecolors='white', linewidths=1.5, label='EGO best', zorder=103)


def add_field_boundaries_to_plot(ax, field_bounds: Tuple[float, float, float, float]):
    """Add field boundary rectangle to plot"""
    min_x, max_x, min_y, max_y = field_bounds
    boundary_x = [min_x, max_x, max_x, min_x, min_x]
    boundary_y = [min_y, min_y, max_y, max_y, min_y]
    ax.plot(boundary_x, boundary_y, 'k-', linewidth=2, label='Field Boundary')

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
