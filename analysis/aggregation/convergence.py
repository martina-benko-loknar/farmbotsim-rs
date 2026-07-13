import numpy as np


def ego_best_so_far(evaluation_history):
    """
    x = evaluation index, y = best energy found so far. `best_energy` in
    each entry is already a running best (see ego.trace.evaluation_history
    in the raw JSON), so no recomputation is needed.
    """
    x = np.array([e["evaluation"] for e in evaluation_history])
    y = np.array([e["best_energy"] for e in evaluation_history])
    return x, y


def grid_best_so_far(points):
    """
    x = evaluation index, y = best energy found so far, treating grid_search
    points in their stored (raster-scan) order as a evaluation sequence.
    Grid search has no real "convergence" -- this is the anytime-performance
    view of "if points were evaluated in this order, how quickly would you
    stumble on the optimum".
    """
    energies = np.array([p["metrics"]["energy_wh"] for p in points])
    y = np.minimum.accumulate(energies)
    x = np.arange(1, len(points) + 1)
    return x, y


def stack_curves(curves):
    """
    curves: list of (x, y) arrays from multiple seeds, expected to share the
    same x. Truncates to the shortest curve if seeds disagree in length.
    Returns (x, mean, std).

    WARNING: unlike the soc/battery single-evaluation sweeps (where
    ExperimentConfig.seed is never wired into any RNG, so runs at a fixed
    parameter value are byte-identical), fleet_sweep/field_sweep runs a
    fresh EGO optimization per file, and the resulting std here is
    genuinely non-zero -- EGO's initial sampling phase has its own
    randomness that is independent of the experiment's `seed` field. So
    the shaded band on a convergence plot reflects the optimizer's own
    randomness, not simulation stochasticity, and does not shrink or
    become reproducible by fixing `seed`.
    """
    lengths = [len(y) for _, y in curves]
    min_len = min(lengths)
    if len(set(lengths)) > 1:
        print(f"warning: convergence curves have mismatched lengths {lengths}, truncating to {min_len}")

    ys = np.stack([y[:min_len] for _, y in curves])
    x = curves[0][0][:min_len]
    return x, ys.mean(axis=0), ys.std(axis=0)
