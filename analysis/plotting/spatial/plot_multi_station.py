import matplotlib.pyplot as plt
from typing import List, Optional, Tuple
from viz_utils import setup_latex_fonts, add_obstacles_to_2d_plot
from viz_models import Pos2, Obstacle

# ============================================================================
# Multi-Station Plots
# ============================================================================

# Short legend suffixes for the specialist layouts defined in
# station_layouts.rs (StationLayout.name), so the legend reads e.g.
# "0.29 (DC)" instead of a bare number the reader has to match to a marker
# shape via the caption text.
LAYOUT_ABBREVIATIONS = {
    "diagonal_corners": "DC",
    "horizontal_symmetry": "HS",
    "vertical_symmetry": "VS",
    "split_center": "SC",
    "tight_center": "TC",
    "task_centroid": "CT",
}

# Fixed qualitative (Okabe-Ito colorblind-safe) palette, one color per
# heuristic layout -- replaces a previous per-point sample of the RdYlBu_r
# diverging colormap (the same one the energy/distance heatmaps use for a
# *continuous* surface). Reusing a diverging heatmap colormap to color
# discrete, unordered categories doesn't have a principled meaning here
# (there's no ordering across DC/HS/VS/SC/TC/CT to diverge around), isn't
# colorblind-safe, and visually put this figure in a different color
# system from every other EGO-vs-heuristics comparison in the paper
# (baseline_comparison_multiagent.py / baseline_comparison_oat.py both use
# a plain black-vs-blue convention). Keyed by the same layout name used in
# LAYOUT_ABBREVIATIONS so a given heuristic gets the same color regardless
# of the order specialist.layouts happens to list them in (2026-09-02).
LAYOUT_COLORS = {
    "diagonal_corners": "#E69F00",    # orange
    "horizontal_symmetry": "#56B4E9", # sky blue
    "vertical_symmetry": "#009E73",   # bluish green
    "split_center": "#F0E442",        # yellow
    "tight_center": "#0072B2",        # blue -- same hex as the paper's
                                       # other black/blue two-series pairs
    "task_centroid": "#D55E00",       # vermillion
}
# Positional fallback (same palette, cycled by index) for the rare call
# without layout_names -- keeps a fixed, distinct color per point instead
# of falling back to the old colormap sampling.
_LAYOUT_COLOR_FALLBACK = list(LAYOUT_COLORS.values())

# EGO's own marker color -- black, matching the black-star convention used
# for EGO/the anchor everywhere else in the paper (baseline_comparison_
# multiagent.py, baseline_comparison_oat.py), not the reddish/blue extreme
# of a colormap.
EGO_MARKER_COLOR = "black"


def _layout_color(i: int, layout_names: Optional[List[str]]) -> str:
    if layout_names:
        name = layout_names[i]
        if name in LAYOUT_COLORS:
            return LAYOUT_COLORS[name]
    return _LAYOUT_COLOR_FALLBACK[i % len(_LAYOUT_COLOR_FALLBACK)]


def generate_multi_station_plot(
    optimal_stations: List[Pos2],
    optimal_energy: float,
    suboptimal_configs: List[Tuple[List[Pos2], float]],
    obstacles: List[Obstacle],
    field_bounds: Tuple[float, float, float, float],
    output_dir: str = "results",
    prefix: str = None,
    layout_names: Optional[List[str]] = None,
):
    """Generate multi-station configuration plot (energy-based)"""
    setup_latex_fonts(30)

    fig, ax = plt.subplots(figsize=(10.75, 10))

    # Plot optimal configuration
    x_coords = [s.x for s in optimal_stations]
    y_coords = [s.y for s in optimal_stations]
    optimal_label = f'{optimal_energy/1000:.2f} (EGO)' if layout_names else f'{optimal_energy/1000:.2f}'
    ax.scatter(x_coords, y_coords, c=EGO_MARKER_COLOR, marker='*', s=300,
              edgecolors='black', linewidths=1,
              label=optimal_label, zorder=100)

    # Plot suboptimal configurations
    for i, (stations, energy) in enumerate(suboptimal_configs):
        x_coords = [s.x for s in stations]
        y_coords = [s.y for s in stations]
        color = _layout_color(i, layout_names)
        alpha = 0.9
        markers = ['o', '^', '<', 'p', 's', 'D']
        marker = markers[i % len(markers)]
        if layout_names:
            abbrev = LAYOUT_ABBREVIATIONS.get(layout_names[i], layout_names[i])
            label = f'{energy/1000:.2f} ({abbrev})'
        else:
            label = f'{energy/1000:.2f}'
        ax.scatter(
            x_coords,
            y_coords,
            color=color,
            marker=marker,
            s=150,
            alpha=alpha,
            edgecolors='black',
            linewidths=1,
            label=label
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
              bbox_to_anchor=(1.35, 0.98),
              fontsize=25,
              borderpad=0,
              labelspacing=0.35,
              handlelength=1.0,
              handletextpad=0.4,
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
    stem = prefix or f"multi_station_{len(optimal_stations)}"
    filename = f"{output_dir}/{stem}_energy"
    fname = f"{stem}_energy"
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
    output_dir: str = "results",
    prefix: str = None,
    layout_names: Optional[List[str]] = None,
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
    optimal_label = f'{optimal_distance/1000:.3f} (EGO)' if layout_names else f'{optimal_distance/1000:.3f}'
    ax.scatter(x_coords, y_coords, c=EGO_MARKER_COLOR, marker='*', s=300,
              edgecolors='black', linewidths=1,
              label=optimal_label, zorder=100)

    # Plot suboptimal configurations
    for i, (stations, distance) in enumerate(suboptimal_configs):
        x_coords = [s.x for s in stations]
        y_coords = [s.y for s in stations]
        color = _layout_color(i, layout_names)
        alpha = 0.9
        markers = ['o', '^', '<', 'p', 's', 'D']
        if layout_names:
            abbrev = LAYOUT_ABBREVIATIONS.get(layout_names[i], layout_names[i])
            label = f'{distance/1000:.3f} ({abbrev})'
        else:
            label = f'{distance/1000:.3f}'
        ax.scatter(
            x_coords,
            y_coords,
            color=color,
            marker=markers[i % len(markers)],
            s=150,
            alpha=alpha,
            edgecolors='black',
            linewidths=1,
            label=label
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
              bbox_to_anchor=(1.35, 0.98),
              fontsize=25,
              borderpad=0,
              labelspacing=0.35,
              handlelength=1.0,
              handletextpad=0.4,
              frameon=False,
              title=r'$d$ (km)',
              title_fontsize=25,
              alignment='left'
              )
    #legend.get_title().set_ha('left')  
    legend._legend_box.sep = 20
    ax.grid(False)
    ax.spines['top'].set_visible(True)
    ax.spines['right'].set_visible(True)
    ax.spines['bottom'].set_color('black')
    ax.spines['left'].set_color('black')
    
    plt.tight_layout()
    stem = prefix or f"multi_station_{len(optimal_stations)}"
    filename = f"{output_dir}/{stem}_distance"
    fname = f"{stem}_distance"
    #plt.savefig(f"{filename}.png", dpi=150, bbox_inches='tight')
    plt.savefig(f"{filename}.pdf", bbox_inches='tight')
    #print(f"Plot saved to: {filename}.png and {filename}.pdf")
    print(f"- {fname}.pdf")
    plt.close()
