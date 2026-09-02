import os
import sys

from loaders.json_loader import load_optimization_traces, FIELD_SIZE_ORDER
from aggregation.convergence import (
    ego_best_energy_per_task_so_far, normalize_pct_above_final, stack_curves,
)
from results_dir import resolve_results_root, resolve_profile

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "plotting"))
from convergence import (
    generate_multi_convergence_plot, FIELD_SIZE_COLORS,
    PAIRED_FIGURE_MARGINS, PAIRED_FIGURE_FIGSIZE,
)

SWEEP_NAME = "field_sweep"


def main():
    results_root = resolve_results_root()
    profile = resolve_profile()
    RESULTS_DIR = f"{results_root}/raw/{profile}/{SWEEP_NAME}"
    FIGURES_DIR = f"{results_root}/figures/{profile}/{SWEEP_NAME}/convergence"

    df = load_optimization_traces(RESULTS_DIR)
    print(f"Loaded {len(df)} optimization traces from {RESULTS_DIR}")

    os.makedirs(FIGURES_DIR, exist_ok=True)

    # field_sweep is EGO-only (run_ego_experiment, no grid_search block --
    # see loaders/json_loader.py::load_single_station_results): there is no
    # grid-search curve to compare EGO against here, so this plots EGO's
    # own convergence *across field sizes* instead of "EGO vs grid search"
    # at one field size (that comparison lives elsewhere -- the dedicated
    # single-station calibration data already used for the R1.4
    # response-letter figures, see the 2026-08-18 conversation).
    #
    # Energy per completed task (not raw energy_wh, since field size
    # directly changes task count), and each seed's curve normalized to
    # percent-above-its-own-final-value before averaging -- field sizes
    # span wildly different absolute energy scales (S ~0.2 Wh/task vs XL
    # ~0.8 Wh/task), so raw curves can't share one y-axis, but their
    # relative convergence behavior can be compared once each is expressed
    # as "how far above its own eventual best" rather than in Wh.
    curves = []
    for field_size in FIELD_SIZE_ORDER:
        group = df[df["field_size"] == field_size]
        if group.empty:
            continue

        per_seed_curves = [
            normalize_pct_above_final(*ego_best_energy_per_task_so_far(h))
            for h in group["ego_evaluation_history"]
        ]
        x, mean, std = stack_curves(per_seed_curves)
        curves.append({
            "label": field_size, "x": x, "mean": mean, "std": std,
            "color": FIELD_SIZE_COLORS[field_size],
        })

    generate_multi_convergence_plot(
        curves,
        output_dir=FIGURES_DIR,
        prefix="field_convergence_overlay",
        ylabel="\\% above best found ($E_{\\mathrm{tot}}$ / task)",
        # Paired side by side with field_energy_time_mechanism.pdf at the
        # same figsize -- see generate_multi_convergence_plot's `crop`
        # docstring for why both need crop=False to actually match.
        crop=False,
        margins=PAIRED_FIGURE_MARGINS,
        figsize=PAIRED_FIGURE_FIGSIZE,
    )


if __name__ == "__main__":
    main()
