from dataclasses import dataclass
from typing import List, Tuple

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

