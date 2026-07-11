import matplotlib.pyplot as plt
from typing import List, Tuple, Optional
from viz_utils import setup_latex_fonts, add_obstacles_to_3d_plot
from viz_models import Pos2, Obstacle

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
    fname = f"grid_search_{grid_resolution}x{grid_resolution}_3d"
    # plt.savefig(f"{filename}.png", dpi=150, bbox_inches='tight')
    plt.savefig(f"{filename}.pdf", bbox_inches='tight')
    # print(f"Plot saved to: {filename}.png and {filename}.pdf")
    print(f"- {fname}.pdf")
    plt.close()
