import json
from pathlib import Path
import pandas as pd

# Mirrors ExperimentConfig::field_size_label in src/experiment/config.rs, so
# the label stays derived from field_config_path rather than duplicated.
FIELD_SIZE_LABELS = {
    "configs/field_configs/vineyard/small.json": "S",
    "configs/field_configs/vineyard/medium.json": "M",
    "configs/field_configs/vineyard/large.json": "L",
    "configs/field_configs/vineyard/xlarge.json": "XL",
}

FIELD_SIZE_ORDER = ["S", "M", "L", "XL"]


def field_size_label(field_config_path):
    return FIELD_SIZE_LABELS.get(field_config_path, Path(field_config_path).stem)


def load_single_station_results(folder):

    records = []

    folder = Path(folder)

    for file in folder.glob("*.json"):

        with open(file, "r") as f:
            data = json.load(f)

        config = data["metadata"]["config"]

        ego = data["ego"]
        grid = data["grid_search"]

        record = {

            # File information
            "file": file.name,

            # Experiment parameters
            "seed": data["metadata"]["seed"],
            "fleet_size":
                config["n_agents"],
            "battery_wh":
                config["battery_capacity_wh"],
            "soc_threshold":
                config["soc_threshold_percent"],
            "field_config":
                config["field_config_path"],
            "field_size":
                field_size_label(config["field_config_path"]),

            # EGO result
            "ego_energy_wh":
                ego["summary"]["best_metrics"]["energy_wh"],
            "ego_distance_m":
                ego["summary"]["best_metrics"]["total_distance_m"],
            "ego_evaluations":
                ego["summary"]["total_evaluations"],
            # Wall time EGO itself reports for its optimization run (all
            # init + BO evaluations). Read directly, not summed by us.
            "ego_optimization_time_sec":
                ego["summary"]["optimization_time_sec"],

            # Grid result
            "grid_energy_wh":
                grid["summary"]["best_point"]["metrics"]["energy_wh"],
            "grid_valid_points":
                grid["summary"]["valid_points"],
            "grid_total_points":
                grid["summary"]["total_points"],
            # Grid search has no equivalent "optimization_time_sec" summary
            # field, so we sum each evaluated point's own evaluation_time_sec
            # from the trace instead -- gives a time figure computed the same
            # way (sum of per-evaluation cost) as ego_optimization_time_sec,
            # so the two are comparable.
            "grid_time_sec":
                sum(p["metrics"]["evaluation_time_sec"] for p in grid["trace"]["points"]),

            # Runtime: combined EGO + grid evaluation time as reported by the
            # experiment runner. This is NOT split by method -- it mixes both
            # phases, since run_single_station_experiment always runs EGO
            # then grid search in sequence and times the whole thing. Use
            # ego_optimization_time_sec / grid_time_sec above for a fair
            # per-method comparison (e.g. comparison_fleet.py / comparison_field.py).
            "runtime_sec":
                data["timing"]["total_evaluation_time_sec"],

        }


        records.append(record)


    return pd.DataFrame(records)


def load_single_evaluation_results(folder):
    """
    Load sweep results produced by `run_single_evaluation` (soc_sweep,
    battery_sweep): one JSON per (parameter value, seed), holding a single
    `single_evaluation` block rather than a full grid_search + ego run.

    The baseline file used to anchor the station position (grid_search + ego,
    no `single_evaluation` key) is skipped.
    """

    records = []

    folder = Path(folder)

    for file in folder.glob("*.json"):

        with open(file, "r") as f:
            data = json.load(f)

        if "single_evaluation" not in data:
            continue

        config = data["metadata"]["config"]
        single = data["single_evaluation"]

        record = {

            # File information
            "file": file.name,

            # Experiment parameters
            "seed": data["metadata"]["seed"],
            "fleet_size":
                config["n_agents"],
            "battery_wh":
                config["battery_capacity_wh"],
            "soc_threshold":
                config["soc_threshold_percent"],
            "critical_soc":
                config["critical_soc_percent"],
            "field_config":
                config["field_config_path"],
            "field_size":
                field_size_label(config["field_config_path"]),

            # Single-evaluation result
            "energy_wh":
                single["energy_wh"],
            "distance_m":
                single["total_distance_m"],
            "charging_distance_m":
                single["charging_distance_m"],
            "charging_events":
                single["charging_events"],
            "completed_tasks":
                single["completed_tasks"],

            # Runtime
            "runtime_sec":
                data["timing"]["total_evaluation_time_sec"],

        }

        records.append(record)

    return pd.DataFrame(records)


def load_optimization_traces(folder):
    """
    Load per-run EGO evaluation history and grid-search trace points for
    EGO-vs-grid-search convergence comparisons. Unlike
    load_single_station_results, this keeps the raw per-evaluation traces
    instead of collapsing each run to its summary metrics.
    """

    records = []

    folder = Path(folder)

    for file in folder.glob("*.json"):

        with open(file, "r") as f:
            data = json.load(f)

        if "ego" not in data or "grid_search" not in data:
            continue

        config = data["metadata"]["config"]

        record = {

            # File information
            "file": file.name,

            # Experiment parameters
            "seed": data["metadata"]["seed"],
            "fleet_size":
                config["n_agents"],
            "field_config":
                config["field_config_path"],
            "field_size":
                field_size_label(config["field_config_path"]),

            # Raw traces
            "ego_evaluation_history":
                data["ego"]["trace"]["evaluation_history"],
            "grid_points":
                data["grid_search"]["trace"]["points"],

        }

        records.append(record)

    return pd.DataFrame(records)