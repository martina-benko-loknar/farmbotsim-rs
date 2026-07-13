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
| `sensitivity_soc.py` | `soc_sweep` | energy vs SoC threshold |
| `sensitivity_battery.py` | `battery_sweep` | energy vs battery capacity |
| `scaling_fleet.py` | `fleet_sweep` | runtime vs fleet size |
| `scaling_field.py` | `field_sweep` | runtime vs field size |
| `convergence_fleet.py` / `convergence_field.py` | `fleet_sweep` / `field_sweep` | EGO vs grid search: best-energy-found-so-far vs evaluation count |
| `comparison_fleet.py` / `comparison_field.py` | `fleet_sweep` / `field_sweep` | EGO vs grid search: evaluation count, final energy, wall time |

Shared building blocks live in `loaders/` (JSON -> DataFrame), `aggregation/`
(mean/std grouping, convergence-curve extraction), and `plotting/` (matplotlib,
LaTeX-styled, matching `results/paper_results*`).

## Notes & caveats

- **`seed` is a no-op in `soc_sweep`/`battery_sweep`.** `ExperimentConfig.seed`
  is recorded in the output but never used to seed any RNG in
  `src/experiment/runner.rs`, so repeated seeds at a fixed SoC threshold /
  battery capacity are byte-identical (std = 0). Don't read anything into a
  flat sensitivity curve there until that's wired up.

- **EGO's own randomness shows up in `fleet_sweep`/`field_sweep`, independent
  of `seed`.** These sweeps run a fresh EGO optimization per file, and its
  initial sampling phase has randomness that isn't tied to the experiment's
  `seed` field either. So, unlike the soc/battery sweeps, seed-to-seed
  variance here is real -- but it's optimizer noise, not simulation noise,
  and won't shrink or become reproducible by fixing `seed`. See the warning
  in `aggregation/convergence.py::stack_curves`.

- **EGO vs grid search timing is computed two different ways on purpose.**
  The top-level `timing.total_evaluation_time_sec` in each raw JSON combines
  *both* phases (`run_single_station_experiment` always runs EGO then grid
  search in sequence and times the whole thing), so it's not a fair
  per-method number. `ego_optimization_time_sec` / `grid_time_sec` in
  `loaders/json_loader.py::load_single_station_results` are built to be
  comparable instead: EGO's is read directly from `ego.summary`, grid's is
  the sum of each evaluated point's own `evaluation_time_sec` from the trace
  (same "sum of per-evaluation cost" basis as EGO's figure).

- **Reading the comparison plots (`comparison_fleet.py` /
  `comparison_field.py`):** on the 2026-07-12 fleet_sweep data, EGO matches
  or beats grid search's final energy (most clearly at fleet size 1, ~24.5k
  vs ~32.6k Wh) while taking meaningfully less wall time at every fleet size,
  despite a similar or lower evaluation count (50 EGO evaluations vs 68 valid
  grid points). That's the shape of result these plots are meant to surface
  -- re-check it holds as more sweep data comes in rather than assuming it's
  a fixed conclusion.
