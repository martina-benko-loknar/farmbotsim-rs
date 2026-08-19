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
    """
    Load results with an EGO summary, optionally paired with a grid-search
    summary. Historically every file here came from
    run_single_station_experiment (always EGO + grid search), but
    fleet_sweep/field_sweep switched to EGO-only (run_ego_experiment) to
    route around a grid-search hang -- see
    various/task_completion_overshoot_hang_bug.txt in the Rust repo root.
    Files without a `grid_search` block get NaN in the grid_* columns
    rather than being skipped, so fleet/field sweep data still loads (and
    aggregates/plots fine via pandas' NaN-aware mean/std) -- callers that
    care about the EGO-vs-grid comparison specifically (comparison_*.py,
    convergence_*.py) are responsible for checking whether grid data is
    actually present before plotting it.
    """

    records = []

    folder = Path(folder)

    for file in folder.glob("*.json"):

        with open(file, "r") as f:
            data = json.load(f)

        config = data["metadata"]["config"]

        ego = data["ego"]
        grid = data.get("grid_search")

        record = {

            # File information
            "file": file.name,

            # Experiment parameters
            "seed": data["metadata"]["seed"],
            "fleet_size":
                config["n_agents"],
            # Charging slots at the (single) station. Defaults to 1 for
            # files predating this field (n_station_slots didn't exist
            # before the fleet_slots_sweep experiment).
            "n_station_slots":
                config.get("n_station_slots", 1),
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
            "ego_completed_tasks":
                ego["summary"]["best_metrics"]["completed_tasks"],
            # Mission wall-clock time simulated for the best-found layout,
            # not to be confused with ego_optimization_time_sec (real time
            # EGO itself took to search).
            "ego_simulation_time_sec":
                ego["summary"]["best_metrics"]["simulation_time_sec"],
            "ego_evaluations":
                ego["summary"]["total_evaluations"],
            # Wall time EGO itself reports for its optimization run (all
            # init + BO evaluations). Read directly, not summed by us.
            "ego_optimization_time_sec":
                ego["summary"]["optimization_time_sec"],

            # Grid result (NaN when this file has no grid_search block)
            "grid_energy_wh":
                grid["summary"]["best_point"]["metrics"]["energy_wh"] if grid else float("nan"),
            "grid_completed_tasks":
                grid["summary"]["best_point"]["metrics"]["completed_tasks"] if grid else float("nan"),
            "grid_valid_points":
                grid["summary"]["valid_points"] if grid else float("nan"),
            "grid_total_points":
                grid["summary"]["total_points"] if grid else float("nan"),
            # Grid search has no equivalent "optimization_time_sec" summary
            # field, so we sum each evaluated point's own evaluation_time_sec
            # from the trace instead -- gives a time figure computed the same
            # way (sum of per-evaluation cost) as ego_optimization_time_sec,
            # so the two are comparable.
            "grid_time_sec":
                sum(p["metrics"]["evaluation_time_sec"] for p in grid["trace"]["points"]) if grid else float("nan"),

            # Runtime: combined EGO + grid evaluation time as reported by the
            # experiment runner (or just EGO's time, for EGO-only files).
            # This is NOT split by method when grid data is present -- it
            # mixes both phases, since run_single_station_experiment always
            # runs EGO then grid search in sequence and times the whole
            # thing. Use ego_optimization_time_sec / grid_time_sec above for
            # a fair per-method comparison (e.g. comparison_fleet.py /
            # comparison_field.py).
            "runtime_sec":
                data["timing"]["total_evaluation_time_sec"],

        }


        records.append(record)


    return pd.DataFrame(records)


def load_single_evaluation_results(folder):
    """
    Load sweep results produced by `run_single_evaluation` (soc_sweep,
    battery_sweep): one JSON per (parameter value, [condition,] seed),
    holding a single `single_evaluation` block rather than a full
    grid_search + ego run.

    The baseline file used to anchor the station position (grid_search + ego,
    no `single_evaluation` key) is skipped.

    soc_sweep/battery_sweep each run two conditions per parameter value/seed
    (see `run_soc_sweep`'s doc comment in `src/experiment/sweeps/soc.rs`):
    `"spawn_only"` (only spawn-position randomization, the original behavior)
    and `"full_noise"` (initial-SoC + task-duration-jitter randomization also
    on, a robustness check -- see `various/randomness_axes_sweep_design.txt`).
    Read straight from whether `initial_soc_min_percent` is set on the run's
    own config rather than parsed from the filename, so it works for both
    pre-`cond=` files (always `"spawn_only"`, the key is just absent) and new
    ones.
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
            "condition":
                "full_noise" if config.get("initial_soc_min_percent") is not None else "spawn_only",

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


def load_latest_soc_optimum(results_base="../results", profile="vineyard", fleet_size=1):
    """
    Load the most recent Level II SoC-threshold EGO-optimization result
    (`run_soc_threshold_experiment`, `--optimize-soc`) for the given profile
    and fleet size, for annotating soc_sweep sensitivity plots with the
    optimizer's found threshold.

    Independent of `resolve_results_root()`'s "most recent date dir"
    convention: soc_sweep and soc_optimization_exp are typically generated
    on different dates (the sweep is expensive, the optimizer run is cheap
    and re-run ad hoc), so this searches every `results/<date>/` dir under
    `results_base` rather than just the latest one, and returns the entry
    with the latest (date dir, filename) -- filenames are `HHMMSS_...`, so
    that sorts to the most recent run. Returns None if no matching file
    exists (callers should treat the annotation as optional).
    """
    candidates = sorted(
        Path(results_base).glob(
            f"*/raw/{profile}/soc_optimization_exp/*_fleet={fleet_size}_batt=*.json"
        )
    )
    candidates = [f for f in candidates if not f.stem.endswith("_placement")]
    if not candidates:
        return None

    file = candidates[-1]
    with open(file, "r") as f:
        data = json.load(f)

    summary = data["soc"]["summary"]
    return {
        "file": file.name,
        "optimal_threshold_percent": summary["optimal_threshold_percent"],
        "energy_wh": summary["best_metrics"]["energy_wh"],
        "completed_tasks": summary["best_metrics"]["completed_tasks"],
    }


def load_baseline_comparison_results(folder):
    """
    Load results produced by `run_baseline_comparison_sweep`: one JSON per
    (layout, field/battery/fleet combo, seed), same `single_evaluation`
    shape as load_single_evaluation_results but with no `layout` field in
    the JSON itself -- it's only encoded in the filename
    (`HHMMSS_layout={name}_size={..}_fleet={..}_batt={..}_soc={..}_seed={..}.json`),
    so it's parsed out here via regex.
    """
    import re

    records = []

    folder = Path(folder)
    pattern = re.compile(r"layout=([a-z_]+)_size=")

    for file in folder.glob("*.json"):

        with open(file, "r") as f:
            data = json.load(f)

        if "single_evaluation" not in data:
            continue

        match = pattern.search(file.name)
        layout = match.group(1) if match else "unknown"

        config = data["metadata"]["config"]
        single = data["single_evaluation"]

        record = {
            "file": file.name,
            "layout": layout,
            "seed": data["metadata"]["seed"],
            "fleet_size": config["n_agents"],
            "battery_wh": config["battery_capacity_wh"],
            "soc_threshold": config["soc_threshold_percent"],
            "field_config": config["field_config_path"],
            "field_size": field_size_label(config["field_config_path"]),
            "energy_wh": single["energy_wh"],
            "distance_m": single["total_distance_m"],
            "charging_distance_m": single["charging_distance_m"],
            "charging_events": single["charging_events"],
            "completed_tasks": single["completed_tasks"],
            "runtime_sec": data["timing"]["total_evaluation_time_sec"],
        }

        records.append(record)

    return pd.DataFrame(records)


def load_randomization_sweep_results(folder):
    """
    Load results from the initial-SoC and task-duration-jitter randomization
    sweeps (`run_initial_soc_sweep`, `run_task_duration_jitter_sweep`): same
    per-(parameter, seed) `single_evaluation` shape as
    `load_single_evaluation_results`, but each file also runs one of two
    conditions -- `fixed` (today's deterministic behavior: every agent/task
    at its template value) or `randomized` (the axis under test drawn from a
    range) -- encoded in the filename as `..._{condition}_seed={n}.json`
    (`initsoc=` for the SoC axis, `taskjit=` for the task-duration axis; only
    the fixed/randomized value is read, not the key name before it). That
    makes this loader axis-agnostic -- it works unmodified on either sweep's
    output directory, which is the point: the two sweeps are structurally
    identical (same (field_size, fleet_size, seed) grid, same
    fixed-vs-randomized comparison), so one loader and one analysis script
    (`sensitivity_randomization.py`) cover both.

    The baseline file used to anchor the station position (grid_search +
    ego, no `single_evaluation` key) is skipped, same as
    `load_single_evaluation_results`.
    """
    import re

    records = []

    folder = Path(folder)
    pattern = re.compile(r"=(fixed|randomized)_seed=")

    for file in folder.glob("*.json"):

        with open(file, "r") as f:
            data = json.load(f)

        if "single_evaluation" not in data:
            continue

        match = pattern.search(file.name)
        condition = match.group(1) if match else "unknown"

        config = data["metadata"]["config"]
        single = data["single_evaluation"]

        record = {
            "file": file.name,
            "condition": condition,
            "seed": data["metadata"]["seed"],
            "fleet_size": config["n_agents"],
            "battery_wh": config["battery_capacity_wh"],
            "soc_threshold": config["soc_threshold_percent"],
            "field_config": config["field_config_path"],
            "field_size": field_size_label(config["field_config_path"]),
            "energy_wh": single["energy_wh"],
            "distance_m": single["total_distance_m"],
            "charging_distance_m": single["charging_distance_m"],
            "charging_events": single["charging_events"],
            "completed_tasks": single["completed_tasks"],
            # Mission wall-clock time simulated, not analysis wall time --
            # matches comparison_fleet_slots.py's "mission time" metric.
            "simulation_time_sec": single["simulation_time_sec"],
            "runtime_sec": data["timing"]["total_evaluation_time_sec"],
        }

        records.append(record)

    return pd.DataFrame(records)


def load_optimization_traces(folder):
    """
    Load per-run EGO evaluation history, plus grid-search trace points when
    present, for EGO-vs-grid-search convergence comparisons. Unlike
    load_single_station_results, this keeps the raw per-evaluation traces
    instead of collapsing each run to its summary metrics.

    Files without a `grid_search` block (fleet_sweep/field_sweep, which
    switched to EGO-only -- see the note in load_single_station_results)
    still get a record, with `grid_points` set to None, so EGO's own
    convergence curve is available even where grid search wasn't run.
    Callers that want the comparison specifically should check for None.
    """

    records = []

    folder = Path(folder)

    for file in folder.glob("*.json"):

        with open(file, "r") as f:
            data = json.load(f)

        if "ego" not in data:
            continue

        config = data["metadata"]["config"]
        grid = data.get("grid_search")

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
                grid["trace"]["points"] if grid else None,

        }

        records.append(record)

    return pd.DataFrame(records)