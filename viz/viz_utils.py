import numpy as np
import matplotlib.pyplot as plt
from matplotlib import cm
from matplotlib.patches import Polygon as MplPolygon
from mpl_toolkits.mplot3d import Axes3D
from scipy.interpolate import griddata
from dataclasses import dataclass
from typing import List, Tuple, Optional
import os

@dataclass
class Pos2:
    """2D position"""
    x: float
    y: float


@dataclass
class Obstacle:
    """Obstacle defined by polygon points"""
    points: List[Pos2]

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
# Helper Functions for 2D Plots
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