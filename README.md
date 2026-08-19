# farmbotsim-rs

**Note:** This project is currently under active development. 
The latest prebuilt version (Windows only) can be downloaded from the Releases page. 

---

**farmbotsim-rs** is a simulation environment designed to upport research and development in agricultural automation. It provides tools for modeling, visualizing, and optimizing robotic farm operations, with a particular focus on charging strategies and productivity analysis.


<details>
<summary><strong>Screenshots of application:</strong></summary>

<img src="media/tool_movement_config_editor.png" alt="movement_config_editor_tool">
<img src="media/tool_battery.png" alt="battery_tool">
<img src="media/tool_agent_config_editor.png" alt="agent_config_editor_tool">
<img src="media/tool_farm_entity_plan_editor.png" alt="farm_entity_plan_editor_tool">
<img src="media/tool_field_config_editor.png" alt="field_config_editor_tool">
<img src="media/tool_scene_config_editor.png" alt="scene_config_editor_tool">
<img src="media/tool_simulation.png" alt="simulation_tool">
<img src="media/tool_path.png" alt="path_tool">
<img src="media/tool_task.png" alt="path_task">
<img src="media/tool_performance_matrix.png" alt="performance_matrix_task">
</details>

## Table of Contents
- [Prerequisites](#prerequisites)
- [Installation](#installation)
- [Usage](#usage)
- [Code Structure Overview](#code-structure-overview)
- [Run Rust Modes](#run-rust-modes)
- [Visualization (Python)](#visualization-python)

## Prerequisites

Before getting started, you need to have **[Rust](https://www.rust-lang.org/tools/install)** installed on your machine.

Additionally, it's recommended to have **[Git](https://git-scm.com/)** installed to clone the repository.

## Installation

Follow these steps to get farmbotsim running locally:

1. Clone the repository:
   ```bash
   git clone https://github.com/Axstr0n/farmbotsim-rs.git
   ```

2. Navigate to the project directory:
    ```bash
    cd farmbotsim-rs
    ```

## Usage
After installation you can run main file like:
```
cargo run
```
This will run whole application. (Note: running first time takes longer to build)

## Building project
To build the project in release mode use:
```bash
cargo build --release
```

## Code Structure Overview

The **farmbotsim-rs** project is organized into several directories that help separate functionality. Here's a high-level breakdown:


`configs/`
- `agent_configs/` - Contains agent configs. (movement + battery)
- `batteries/` - Contains battery configs.
- `farm_entity_plans/` - Contains plans for farm entity growth.
- `field_configs/` - Contains parameters for field config. (field)
- `movement_configs/` - Contains movement configs. (movement)
- `scene_configs/` - Contains parameters for scene config. (field + stations + spawn area)

`general_help/` - Contains markdown and images for overview of project.

`media/`- Contains screenshots of app.

`performance_matrix/` - Stores all evaluations

`results/` - JSON outputs generated from experiments, optimizations, and sweeps, organized by run date (`results/<date>/raw/...`, `results/<date>/figures/...`).

`various/` - Physical calibration data (voltage-drop-vs-slope measurements, seasonal-solar charging curves, slip-model fits) used to build the calibrated battery/terrain models.

`src/` - Contains the core logic of the application:
- `agent_module/` - Contains the agent struct and its associated logic, state machine.
- `app_module/` - Main app functionality.
- `battery_module/` - Containing battery logic (charging/discharging models, including the calibrated slope-dependent and seasonal-solar models).
- `environment/` - Contains all environment structs (Crop, Field, Station, Env, Config, ...).
  - `env_module/` - Contains logic for env.
  - `farm_entity_module/` - Contains logic for farm entity.
  - `spawn_area_module/` - Contains logic for spawn area.
  - `station_module/` - Contains logic for station.
  - `...`
- `experiment/` - Experiment configs, runners, sweeps (`sweeps/`: battery, fleet, field, soc), and result export.
- `movement_module/` - Contains movement logic
- `optimization/` - EGO/Bayesian optimization and grid search.
- `path_finding_module/` - Includes code related to navigation and pathfinding algorithms.
- `rendering/` - Responsible for rendering.
- `results/` - Result data structures shared across experiment types.
- `task_module/` - Includes files for task creation and task handling.
- `terrain/` - Terrain elevation map, slope, and wheel-slip model.
- `tool_module/` - Contains files for app modes (simulation, editor, path, task, ...).
- `units/` - Unit system.
- `utilities/` - Common utilities and helper functions used across the project.
- `cfg.rs` - Contains constants.
- `logger.rs` - Logger for application.
- `main.rs` - Contains entry point into application.

`analysis/` - Python pipeline that aggregates and plots the Rust sweep experiments (sensitivity, scaling, convergence, EGO-vs-grid-search comparison). See `analysis/README.md` for the script-by-script breakdown and known caveats (e.g. which sweeps currently have real seed-to-seed variance vs. none).

`viz/` - Python script (`build_consumption_lut.py`) that builds the discharge-model lookup table from calibration data.

`.gitignore` - Ignores files/folders.

`Cargo.lock` - Records the exact versions of dependencies used for this project.

`Cargo.toml` - Contains dependencies of project.

`README.md` - This file, which contains documentation and instructions for setting up and using the application.

## Run Rust Modes

`main.rs` supports several command-line modes for running experiments, optimizations, and sweeps. When using `cargo run`, always place your program flags after `--` to separate them from Cargo's own options. With no mode flag, `cargo run` launches the GUI application.

All modes accept `--profile <legacy|vineyard>` to select the experiment "story" (paired field set + battery model); default is `legacy`. Add `--viz` to also invoke the matching plotting script in `analysis/plotting/` after the run.

**Single evaluation** runs one experiment configuration and saves results as JSON.
```
cargo run -- --single-evaluation
```

**EGO optimization** performs Bayesian (EGO) station-position optimization for `n` stations (default = 1).
```
cargo run -- --optimize-ego 2
```

**Grid search** performs an exhaustive grid search over station positions, with a customizable grid resolution.
```
cargo run -- --grid-search 50
```

**Single-station study** runs EGO and grid search back-to-back for one station, for direct comparison.
```
cargo run -- --single-station-study
```

**Multi-station study** runs EGO against the specialist heuristic 2-station layouts (diagonal corners, horizontal/vertical symmetry, split-center, tight-center).
```
cargo run -- --multi-station-study
```

**Sweeps** run a battery of experiments across a parameter axis; each has a dedicated `analysis/` script (see below).
```
cargo run -- --battery-sweep
cargo run -- --fleet-sweep
cargo run -- --field-sweep
cargo run -- --soc-sweep
```

## Analysis (Python)

The `analysis/` folder contains the Python pipeline that aggregates and plots the JSON output of the sweep experiments above (mean/std summaries, sensitivity/scaling/convergence/comparison figures). See `analysis/README.md` for the full script-by-script breakdown, the conda environment to use, and important caveats about which sweeps currently have real seed-to-seed variance and which don't.

The `viz/` folder is unrelated: it holds `build_consumption_lut.py`, which builds the discharge-model lookup table from the calibration data in `various/`.


