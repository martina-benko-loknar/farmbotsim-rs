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


def ego_best_energy_per_task_so_far(evaluation_history):
    """
    Same shape as ego_best_so_far, but tracking energy per completed task
    rather than raw energy -- see analysis/README.md's "energy per
    completed task" convention (comparison_*.py/sensitivity_*.py): the
    >= task-count termination check can drift slightly between evaluated
    positions even at fixed field/fleet size, so raw energy isn't always a
    fair apples-to-apples comparison between two candidates the way
    energy/task is.

    Deliberately reuses each record's own `is_new_best` (already gated on
    validity -- a candidate with a station placed inside a field group's
    bounding box can never set a new best, see optimize_station_positions_ego
    in ego.rs) rather than recomputing a running minimum straight from
    `metrics.energy_wh / metrics.completed_tasks`, which would lose that
    gating and could let an invalid candidate's low energy back in.
    """
    x = np.array([e["evaluation"] for e in evaluation_history])
    y = np.empty(len(evaluation_history))

    current_best = np.inf
    for i, e in enumerate(evaluation_history):
        if e["is_new_best"]:
            m = e["metrics"]
            current_best = m["energy_wh"] / m["completed_tasks"]
        y[i] = current_best

    return x, y


def grid_best_energy_per_task_so_far(points):
    """
    Energy-per-completed-task counterpart to grid_best_so_far. Grid search
    never evaluates an invalid position in the first place (see
    generate_valid_grid_points), so a plain running minimum over each
    point's own energy/task -- no extra validity gating needed, unlike the
    EGO version above.
    """
    per_task = np.array(
        [p["metrics"]["energy_wh"] / p["metrics"]["completed_tasks"] for p in points]
    )
    y = np.minimum.accumulate(per_task)
    x = np.arange(1, len(points) + 1)
    return x, y


def normalize_pct_above_final(x, y):
    """
    Rescale a best-so-far curve to percent above its own final (best-found-
    overall) value: 0% at the point the curve reaches its eventual best,
    positive before that, since `y` is a non-increasing running minimum by
    construction (`ego_best_so_far`/`ego_best_energy_per_task_so_far`), so
    `y[-1]` is exactly that run's best-found value.

    Lets convergence curves at wildly different absolute scales -- e.g.
    field_sweep's field sizes S vs XL, ~0.2 vs ~0.8 Wh/task -- share one
    y-axis and be compared directly, since each curve is normalized against
    itself rather than a shared reference value.
    """
    final = y[-1]
    pct = (y - final) / final * 100.0
    return x, pct


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
