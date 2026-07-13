import os
import sys

from loaders.json_loader import load_optimization_traces, FIELD_SIZE_ORDER
from aggregation.convergence import ego_best_so_far, grid_best_so_far, stack_curves
from results_dir import resolve_results_root

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "plotting"))
from convergence import generate_convergence_comparison_plot

SWEEP_NAME = "field_sweep"


def main():
    results_root = resolve_results_root()
    RESULTS_DIR = f"{results_root}/raw/{SWEEP_NAME}"
    FIGURES_DIR = f"{results_root}/figures/{SWEEP_NAME}/convergence"

    df = load_optimization_traces(RESULTS_DIR)
    print(f"Loaded {len(df)} optimization traces from {RESULTS_DIR}")

    os.makedirs(FIGURES_DIR, exist_ok=True)

    for field_size in FIELD_SIZE_ORDER:
        group = df[df["field_size"] == field_size]
        if group.empty:
            continue

        ego_curves = [ego_best_so_far(h) for h in group["ego_evaluation_history"]]
        grid_curves = [grid_best_so_far(p) for p in group["grid_points"]]

        ego_x, ego_mean, ego_std = stack_curves(ego_curves)
        grid_x, grid_mean, grid_std = stack_curves(grid_curves)

        generate_convergence_comparison_plot(
            ego_x, ego_mean, ego_std,
            grid_x, grid_mean, grid_std,
            output_dir=FIGURES_DIR,
            prefix=f"field={field_size}_convergence",
        )


if __name__ == "__main__":
    main()
