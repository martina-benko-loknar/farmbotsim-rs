"""
Shared JSON loading / field / obstacle parsing helpers.

Every experiment export (GridSearchExport, EgoExport, SingleStationExport,
MultiStationExport) shares the same top-level "field" section, so this
parsing logic only needs to live in one place.
"""

import json

from viz_models import Pos2, Obstacle


def load_json_data(json_path):
    """Load exported experiment results from a JSON file."""
    with open(json_path, 'r') as f:
        data = json.load(f)
    return data


def parse_field_config(data):
    """
    Compute field bounds directly from exported obstacle geometry.
    """

    field = data.get("field", {})
    obstacles = field.get("obstacles", [])

    if not obstacles:
        print("WARNING: no obstacle geometry available.")
        return (0.0, 12.0, 0.0, 12.0)

    xs = []
    ys = []

    for obs in obstacles:
        for p in obs:
            xs.append(p["x"])
            ys.append(p["y"])

    min_x = min(xs)
    max_x = max(xs)

    min_y = min(ys)
    max_y = max(ys)

    # small padding
    padding = 1.0

    return (
        min_x - padding,
        max_x + padding,
        min_y - padding,
        max_y + padding
    )


# ============================================================================
# IMPORTANT:
# DO NOT reconstruct geometry from angles / rows in Python.
#
# Rust already computes the real obstacle geometry internally.
# Python should only visualize already-generated obstacle polygons.
# ============================================================================

def parse_obstacles(data):
    """
    Parse already-generated obstacle polygons exported from Rust.

    Expected JSON format:

    "field": {
        "obstacles": [
            [
                {"x": ..., "y": ...},
                {"x": ..., "y": ...},
                ...
            ],
            ...
        ]
    }
    """

    obstacles = []

    field = data.get('field', {})

    obstacles_raw = field.get('obstacles', [])

    if not obstacles_raw:

        print("WARNING:")
        print("No exported obstacle polygons found in JSON.")
        print("Python will show empty field geometry.")
        print(
            "Export final obstacle polygons from Rust "
            "(FieldConfig::get_obstacles())."
        )

        return obstacles

    total_obstacles = 0

    for obs in obstacles_raw:

        points = []

        for p in obs:
            points.append(
                Pos2(p['x'], p['y'])
            )

        obstacles.append(
            Obstacle(points)
        )

        total_obstacles += 1

    print(
        f"Loaded {total_obstacles} obstacle polygons from Rust export."
    )

    return obstacles
