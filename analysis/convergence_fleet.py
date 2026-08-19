import os
import sys

from loaders.json_loader import load_optimization_traces
from aggregation.convergence import (
    ego_best_energy_per_task_so_far, normalize_pct_above_final, stack_curves,
)
from results_dir import resolve_results_root, resolve_profile

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "plotting"))
from convergence import generate_multi_convergence_plot, FLEET_SIZE_COLORS

SWEEP_NAME = "fleet_sweep"


def main():
    results_root = resolve_results_root()
    profile = resolve_profile()
    RESULTS_DIR = f"{results_root}/raw/{profile}/{SWEEP_NAME}"
    FIGURES_DIR = f"{results_root}/figures/{profile}/{SWEEP_NAME}/convergence"

    df = load_optimization_traces(RESULTS_DIR)
    print(f"Loaded {len(df)} optimization traces from {RESULTS_DIR}")

    os.makedirs(FIGURES_DIR, exist_ok=True)

    # fleet_sweep is EGO-only (run_ego_experiment, no grid_search block --
    # see loaders/json_loader.py::load_single_station_results), by design,
    # not just for now -- see the 2026-08-18 conversation / README for why.
    # There is no grid-search curve to compare EGO against here, so this
    # plots EGO's own convergence *across fleet sizes* instead of "EGO vs
    # grid search" at one fleet size (that comparison lives elsewhere -- the
    # dedicated single-station calibration data already used for the R1.4
    # response-letter figures).
    #
    # Energy per completed task (not raw energy_wh, since fleet size
    # directly changes task-completion dynamics via station-queueing
    # contention -- see README's fleet_slots_sweep note), and each seed's
    # curve normalized to percent-above-its-own-final-value before
    # averaging, so fleet sizes at different absolute energy scales can
    # share one y-axis and be compared directly (same rationale as
    # convergence_field.py).
    curves = []
    for fleet_size in sorted(df["fleet_size"].unique()):
        group = df[df["fleet_size"] == fleet_size]

        per_seed_curves = [
            normalize_pct_above_final(*ego_best_energy_per_task_so_far(h))
            for h in group["ego_evaluation_history"]
        ]
        x, mean, std = stack_curves(per_seed_curves)
        curves.append({
            "label": str(fleet_size), "x": x, "mean": mean, "std": std,
            "color": FLEET_SIZE_COLORS.get(fleet_size),
        })

    generate_multi_convergence_plot(
        curves,
        output_dir=FIGURES_DIR,
        prefix="fleet_convergence_overlay",
        ylabel="\\% above best found ($E_{\\mathrm{tot}}$ / task)",
    )


if __name__ == "__main__":
    main()
