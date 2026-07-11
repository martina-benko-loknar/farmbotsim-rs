import matplotlib.pyplot as plt
from matplotlib import cm
from typing import List, Tuple
from viz_utils import setup_latex_fonts, add_obstacles_to_2d_plot
from viz_models import Pos2, Obstacle

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
        color_value = (i+1) / (n_configs + 1)
        color = cm.RdYlBu_r(color_value)
        #color = cm.hsv(color_value)  # Get RGB color from colormap
        alpha = 0.9 
        markers = ['o', '^', '<', 'p', 's', 'D']
        marker = markers[i % len(markers)]
        ax.scatter(
            x_coords,
            y_coords,
            color=color,
            marker=marker,
            s=150,
            alpha=alpha,
            edgecolors='black',
            linewidths=1,
            label=f'{energy/1000:.2f}'
        )

    
    
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
    fname = f"multi_station_{len(optimal_stations)}_energy"
    #plt.savefig(f"{filename}.png", dpi=150, bbox_inches='tight')
    plt.savefig(f"{filename}.pdf", bbox_inches='tight')
    #print(f"Plot saved to: {filename}.png and {filename}.pdf")
    print(f"- {fname}.pdf")
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
        alpha = 0.9 
        markers = ['o', '^', '<', 'p', 's', 'D']
        ax.scatter(
            x_coords,
            y_coords,
            color=color,
            marker=markers[i],
            s=150,
            alpha=alpha,
            edgecolors='black',
            linewidths=1,
            label=f'{distance/1000:.3f}'
        )
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
    fname = f"multi_station_{len(optimal_stations)}_distance"
    #plt.savefig(f"{filename}.png", dpi=150, bbox_inches='tight')
    plt.savefig(f"{filename}.pdf", bbox_inches='tight')
    #print(f"Plot saved to: {filename}.png and {filename}.pdf")
    print(f"- {fname}.pdf")
    plt.close()
