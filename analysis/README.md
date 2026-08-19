# Sweep analysis pipeline

Aggregates results from the Rust sweep experiments (`src/experiment/sweeps/*.rs`)
into mean/std summaries and figures.

Run scripts from inside `analysis/` with the `arrs_sim` conda env (has
pandas/matplotlib/numpy/scipy; `visualization` also works once `pandas` is
installed there). Each script defaults to the most recently generated
`results/<date>/` folder; pass `--results-root ../results/<date>` to target
a specific one instead (see `results_dir.py`).

| Script | Sweep | What it shows |
|---|---|---|
| `sensitivity_soc.py` | `soc_sweep` | energy/task vs SoC threshold, `spawn_only` vs `full_noise` condition |
| `sensitivity_battery.py` | `battery_sweep` | energy/task vs battery capacity, `spawn_only` vs `full_noise` condition |
| `scaling_fleet.py` | `fleet_sweep` | EGO optimization time (total) vs fleet size |
| `scaling_field.py` | `field_sweep` | EGO optimization time (total) vs field size |
| `convergence_fleet.py` | `fleet_sweep` | EGO's own best-energy-found-so-far vs evaluation count, one curve per fleet size, overlaid on one axes (normalized to \% above each curve's own final value) |
| `convergence_field.py` | `field_sweep` | EGO's own best-energy-found-so-far vs evaluation count, one curve per field size, overlaid on one axes (normalized to \% above each curve's own final value) |
| `comparison_fleet.py` | `fleet_sweep` | energy/task (top) and time/evaluation (bottom) vs fleet size, EGO only (see notes below) |
| `comparison_field.py` | `field_sweep` | energy/task (top) and time/evaluation (bottom) vs field size, EGO only (see notes below) |
| `comparison_fleet_slots.py` | `fleet_sweep` + `fleet_slots_sweep` | 1 charging slot vs slots matched to fleet size: energy/task, mission time |
| `sensitivity_randomization.py` | `initial_soc_sweep` + `task_duration_jitter_sweep` | fixed vs randomized condition, per (field size, fleet size): energy/task, mission time, seed-to-seed std ratio |

Energy figures (`sensitivity_*`, `comparison_*`, `baseline_comparison.py`) report
**energy per completed task** (`energy_wh / completed_tasks`), not total energy,
since task count isn't held fixed across a sweep -- most obviously in
`comparison_*`/`scaling_*`, where field/fleet size directly changes how many
tasks get completed, but also in `sensitivity_*`, where task-count termination
is a `>=` check and can drift slightly between seeds even at fixed field/fleet
size.

Shared building blocks live in `loaders/` (JSON -> DataFrame), `aggregation/`
(mean/std grouping, convergence-curve extraction), and `plotting/` (matplotlib,
LaTeX-styled, matching `results/paper_results*`).

## Notes & caveats

- **`seed` genuinely varies `soc_sweep`/`battery_sweep` results (fixed
  2026-07-23, commit `4fdc379`).** Before that fix, `ExperimentConfig.seed`
  was recorded in the output but never used to seed any RNG, so repeated
  seeds at a fixed SoC threshold / battery capacity were byte-identical
  (std = 0) -- if you're looking at data generated before that commit, that
  old caveat still applies to it. Current data has real seed-to-seed spread
  from spawn position/heading (e.g. `std ≈ 1.2 Wh` across 15 seeds at
  soc_threshold=60% in the 2026-08-13 vineyard run) -- a flat sensitivity
  curve there is a real result now, not a wiring bug.

- **`soc_sweep`/`battery_sweep` run two randomization conditions per
  parameter value/seed: `spawn_only` and `full_noise`** (see `run_soc_sweep`'s
  doc comment in `src/experiment/sweeps/soc.rs`, and
  `various/randomness_axes_sweep_design.txt` for why). `spawn_only` is the
  original sweep -- only spawn-position randomization (via `seed`).
  `full_noise` additionally turns on initial-SoC and task-duration-jitter
  randomization, piggybacking on the same `seed` rather than decoupled
  sub-seeds -- a robustness check on whether the SoC-threshold/battery-capacity
  sensitivity curve found under single-axis noise still holds with every axis
  active together. `sensitivity_soc.py`/`sensitivity_battery.py` plot both as
  overlaid series when `full_noise` data is present;
  `loaders/json_loader.py::load_single_evaluation_results` derives the
  `condition` column from the run's own config (`initial_soc_min_percent`
  set or not), not from the filename, so it also works on pre-`cond=` result
  directories (always `spawn_only`).

  **`soc_sweep`'s `full_noise` initial-SoC range is its own, not shared with
  `battery_sweep`/`initial_soc_sweep`.** The obvious shared range,
  `INITIAL_SOC_RANGE = (50, 90)`, fully contains soc_sweep's own swept
  threshold range (50-80%) -- P(initial_soc < threshold) rises from 0% to
  75% across the sweep, a confound that produces a spuriously flat `full_noise`
  curve (most runs at the high end of the sweep start already below the
  threshold under test, regardless of its value). `soc_sweep` instead draws
  from `SOC_SWEEP_FULL_NOISE_INITIAL_SOC_RANGE = (82, 90)` (above the sweep's
  maximum, not below its minimum -- there's no room below 50% that isn't
  critical-battery territory, see `src/experiment/sweeps/soc.rs`'s doc
  comment and `various/randomness_axes_sweep_design.txt`'s 2026-08-17
  follow-up). `battery_sweep` keeps the shared `INITIAL_SOC_RANGE` -- it
  sweeps battery capacity with `soc_threshold_percent` held fixed, so no
  analogous confound exists there.

- **EGO's own randomness shows up in `fleet_sweep`/`field_sweep`, independent
  of `seed`.** These sweeps run a fresh EGO optimization per file, and its
  initial sampling phase has randomness that isn't tied to the experiment's
  `seed` field either. So, unlike the soc/battery sweeps, seed-to-seed
  variance here is real -- but it's optimizer noise, not simulation noise,
  and won't shrink or become reproducible by fixing `seed`. See the warning
  in `aggregation/convergence.py::stack_curves`.

- **Historical EGO-vs-grid-search result on `fleet_sweep`, no longer
  reproducible from current data:** on the 2026-07-12 fleet_sweep data (back
  when fleet_sweep still ran grid search too, see below), EGO matched or
  beat grid search's final energy (most clearly at fleet size 1, ~24.5k vs
  ~32.6k Wh) while taking meaningfully less wall time at every fleet size,
  despite a similar or lower evaluation count (50 EGO evaluations vs 68
  valid grid points). Can't be re-checked against current `fleet_sweep` data
  (no grid data left to compare against, see below, and `comparison_fleet.py`
  no longer makes this comparison at all); would need `single_station_exp`
  data or a historical run like 2026-07-12's instead.

- **`fleet_sweep`/`field_sweep` are EGO-only as of 2026-07-24** (switched
  from `run_single_station_experiment` to `run_ego_experiment` in the Rust
  side to route around a grid-search hang -- see
  `various/task_completion_overshoot_hang_bug.txt` in the repo root). Their
  raw JSON has no `grid_search` block anymore, and it stays that way going
  forward, not just until the hang bug got fixed (it did, 2026-08-11): 50x50
  grid search doesn't scale the way it does for one fixed field/fleet size,
  which is itself part of the argument for using EGO for these two sweeps.
  **`comparison_{fleet,field}.py`/`convergence_{fleet,field}.py` were all
  rewritten 2026-08-18 to be EGO-only by design** (no grid-fallback
  branching code) rather than the generic NaN/None-tolerant fallback
  `load_single_station_results`/`load_optimization_traces` still provide for
  any future caller that does have mixed grid/EGO data:
  - `comparison_fleet.py`/`comparison_field.py` now plot energy/task (top
    panel) paired with time/evaluation (bottom panel) vs fleet/field size
    (`fleet_energy_time_mechanism.pdf`/`field_energy_time_mechanism.pdf`)
    -- dropped the vestigial evaluations bar entirely (it's a constant, see
    the `scaling_*.py` bullet below). Not a causal "top explains bottom"
    mechanism plot like the SoC/battery ones -- physical cost and
    computational cost are just two different, unrelated notions of "cost"
    paired on one figure for economy of space, both plotted vs. the same
    swept size.
  - `convergence_fleet.py`/`convergence_field.py` now overlay all fleet/
    field sizes' own best-so-far curves on one normalized axes
    (`fleet_convergence_overlay.pdf`/`field_convergence_overlay.pdf`,
    `\%` above each curve's own final value, see
    `aggregation/convergence.py::normalize_pct_above_final`) instead of 4
    separate one-curve-each plots.
  The EGO-vs-grid-search *comparison* story for the paper is told from the
  dedicated single-station calibration data instead
  (`results/2026-07-23/raw/vineyard/{grid_search_exp,ego_exp}/`, already the
  source for the R1.4 response-letter figures in `agro-charging-framework`).

- **`scaling_fleet.py`/`scaling_field.py` plot only total EGO optimization
  time vs fleet/field size (2026-08-18 rework, then simplified further the
  same day).** Originally reworked into a 2-panel total-time +
  time/evaluation plot (for the same "evaluations vs size is a flat line,
  not worth a figure" reason as below), but time/evaluation was then moved
  into `comparison_*.py` instead, paired with energy/task there -- see
  above. So these two scripts are back to a single, simple line:
  `ego_optimization_time_sec` (EGO's total wall-clock -- simulation
  evaluations *and* the Bayesian-optimization machinery, GP surrogate fit +
  acquisition-function optimization once per iteration) vs fleet/field
  size. Output: `fleet_scaling_time.pdf`/`field_scaling_time.pdf`.
  Both sweeps hold the EGO evaluation budget fixed regardless of
  fleet/field size (`DEFAULT_EGO_INITIAL_SAMPLES` +
  `DEFAULT_EGO_MAX_ITERATIONS`, currently 90 either way -- see
  `fleet.rs`/`field.rs`) -- that's what makes `comparison_*.py`'s
  time/evaluation panel a clean isolation of per-evaluation cost rather
  than a mix of "more evaluations" and "costlier evaluations"; both
  `comparison_*.py` scripts log a warning if they ever find the budget
  isn't actually constant in the data they're reading.

- **EGO vs grid search timing is computed two different ways on purpose,
  and the gap between them is itself meaningful.** The top-level
  `timing.total_evaluation_time_sec` in each raw JSON is the *sum of each
  evaluation's own simulation time* (`export.rs`) -- pure simulation cost,
  nothing else. `grid_time_sec` in
  `loaders/json_loader.py::load_single_station_results` is built on that
  same basis for grid search specifically (sum of each evaluated point's
  own `evaluation_time_sec` from the trace), so it's directly comparable
  1:1 -- grid search has no other cost to include. `ego_optimization_time_sec`
  instead reads `ego.summary`'s own `elapsed.as_secs_f64()` around the
  *entire* `optimize()` call (`ego.rs`) -- simulation time *plus* the
  Bayesian-optimization machinery (GP surrogate fit + acquisition-function
  optimization, once per iteration). So `ego_optimization_time_sec` is
  systematically larger than `total_evaluation_time_sec`/`runtime_sec` for
  the same run (e.g. field_sweep/XL, 2026-08-17: 5.92s vs 5.01s, an ~18%
  "BO tax") -- that gap is overhead grid search structurally can't have (no
  adaptive-selection step to pay for), and is a real, already-computable
  answer to "what's the basic difference between how grid search and EGO
  work," without needing a fresh EGO-vs-grid rerun.

- **Why `fleet_sweep` energy/task grows with fleet size, and what
  `fleet_slots_sweep` isolates.** `fleet_sweep` always optimizes a single
  station with 1 charging slot (`StationConfig::default().n_slots`),
  regardless of fleet size -- so more agents means more contention for that
  one slot, and agents queueing for their turn burn `POWER_CONSUMPTION_WAIT`
  (`src/cfg.rs`) the whole time they wait, even though `n_tasks_target`
  holds total completed tasks fixed. `fleet_slots_sweep` reruns the same
  fleet sizes with `n_station_slots` matched 1:1 to `n_agents` (no queueing
  possible), so `comparison_fleet_slots.py`'s "1 slot" vs "slots = fleet
  size" bars isolate that contention effect from any genuine
  travel-distance cost of a bigger fleet.

- **Vineyard's Line-action work velocity was silently clamped to the Leo
  Rover's hardware max_velocity until 2026-08-13.** `configs/farm_entity_
  plans/default_line.json`'s work velocity (2 km/h) exceeded the Leo
  Rover's `max_velocity` (0.4 m/s ≈ 1.44 km/h,
  `configs/movement_configs/leo_rover.json`), and movement is clamped to
  `min(task_velocity, hardware_max_velocity)`
  (`romba_movement.rs::calculate_new_pose_from_inputs`) -- so every vineyard
  Line task actually ran at 0.4 m/s regardless of what the plan said. This
  was harmless for plain sensitivity sweeps (it's just a constant offset)
  but made `task_duration_jitter_sweep`'s velocity-based jitter a complete
  no-op (jittered range stayed entirely above the binding hardware cap).
  Fixed by giving vineyard field configs their own plan,
  `configs/farm_entity_plans/vineyard_line.json`, with work velocity
  `0.25 m/s` (matching `slope_consumption.json`'s `work.reference_speed_mps`
  -- the Leo Rover's actual empirical calibration speed -- and comfortably
  under the hardware cap). Legacy is unaffected (its `default_romba.json`
  hardware max is 3 km/h, above the shared plan's 2 km/h, so it was never
  clamped). **Any vineyard raw JSON from before 2026-08-13 was generated
  under the old, always-hardware-capped work speed and isn't directly
  comparable to data generated after** -- rerun rather than mix.

- **`sensitivity_randomization.py` covers two sweeps with one script and one
  loader.** `initial_soc_sweep` (initial-SoC heterogeneity across the fleet)
  and `task_duration_jitter_sweep` (per-task work-duration jitter) are
  structurally identical: same (field size, fleet size, seed) grid, same
  `fixed` vs `randomized` condition run per combo, same `single_evaluation`
  record shape. `loaders/json_loader.py::load_randomization_sweep_results`
  reads the condition straight out of the filename (`..._{fixed,randomized}
  _seed={n}.json`) without caring which axis produced it, so the same
  loader and script handle both -- see `AXES` in
  `sensitivity_randomization.py` to add a future randomization axis the
  same way. The exported CSV's `energy_std_ratio_randomized_over_fixed`
  column is the sweep's actual question in one number per (field size,
  fleet size): how much seed-to-seed energy/task variance that axis adds
  on top of spawn-position randomization alone (ratio > 1 means it adds
  real variance; ~1 means spawn-position noise already dominates).
