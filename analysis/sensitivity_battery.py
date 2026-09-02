import os
import sys

from loaders.json_loader import load_single_evaluation_results
from aggregation.aggregate import summarize
from results_dir import resolve_results_root, resolve_profile, results_root_tag

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "plotting"))
from sensitivity import generate_sensitivity_errorbar_plot, generate_sensitivity_mechanism_plot

SWEEP_NAME = "battery_sweep"
EXPORTS_BASE_DIR = "exports"

# Deployed capacity (leo_rover battery, configs/batteries/leo_rover/config.json)
# -- the vineyard profile's actual battery, one of the swept x-values here.
DEPLOYED_BATTERY_WH = 73.2


def find_charging_events_step_boundaries(spawn_only_summary):
    """
    Find capacity bands where the deterministic "spawn_only"-condition mean
    charging_events count steps to a different integer between two
    consecutive swept capacities -- see the investigation in the 2026-08-18
    conversation: under "spawn_only" (spawn-position noise only), charging_events
    is identical across all 15 seeds at a given capacity, so any change
    between adjacent capacities is a real step, not sampling noise. Not
    hardcoded to specific capacity values so this keeps working if
    battery.rs's swept capacity list changes.

    Returns a list of (xmin, xmax) tuples, one per step found.
    """
    events_by_capacity = (
        spawn_only_summary.set_index("battery_wh")["charging_events_mean"]
        .round().astype(int)
    )
    capacities = events_by_capacity.index.tolist()
    return [
        (prev_wh, cur_wh)
        for prev_wh, cur_wh in zip(capacities, capacities[1:])
        if events_by_capacity[prev_wh] != events_by_capacity[cur_wh]
    ], events_by_capacity


def main():
    results_root = resolve_results_root()
    profile = resolve_profile()
    RESULTS_DIR = f"{results_root}/raw/{profile}/{SWEEP_NAME}"
    FIGURES_DIR = f"{results_root}/figures/{profile}/{SWEEP_NAME}"
    EXPORTS_DIR = f"{EXPORTS_BASE_DIR}/{results_root_tag(results_root)}/{profile}"

    df = load_single_evaluation_results(RESULTS_DIR)
    print(f"Loaded {len(df)} single-evaluation runs from {RESULTS_DIR}")

    # Energy per completed task rather than total energy, since task count
    # can drift slightly between runs (termination is a >= check, not ==).
    df["energy_wh_per_task"] = df["energy_wh"] / df["completed_tasks"]

    # "condition" distinguishes the sweep's two randomization conditions per
    # capacity/seed (see loaders/json_loader.py::load_single_evaluation_results):
    # "spawn_only" (spawn-position noise only, the original sweep) vs "full_noise"
    # (initial-SoC + task-duration-jitter also on -- a robustness check, see
    # various/randomness_axes_sweep_design.txt). Pre-`cond=` result
    # directories only ever have "spawn_only".
    summary = summarize(
        df,
        group_cols=["battery_wh", "condition"],
        value_cols=["energy_wh_per_task", "distance_m", "charging_distance_m", "charging_events", "runtime_sec"],
    ).sort_values("battery_wh")

    print(summary)

    os.makedirs(EXPORTS_DIR, exist_ok=True)
    summary_path = f"{EXPORTS_DIR}/battery_sensitivity_summary.csv"
    summary.to_csv(summary_path, index=False)
    print(f"Saved summary to {summary_path}")

    spawn_only = summary[summary["condition"] == "spawn_only"]
    full_noise = summary[summary["condition"] == "full_noise"]
    has_full_noise = not full_noise.empty

    vlines = [{
        "x": DEPLOYED_BATTERY_WH,
        "label": f"deployed capacity (${DEPLOYED_BATTERY_WH:.1f}$ Wh)",
        "kind": "deployed",
    }]

    # Step boundaries: capacity bands where the deterministic "spawn_only"-
    # condition charging_events count changes -- these explain the
    # non-monotonic energy/task curve (a discrete threshold-crossing
    # timing effect, not noise or a placement-mismatch artifact; see the
    # 2026-08-18 investigation). Computed from the raw per-seed data, not
    # visible from the aggregated energy curve alone.
    step_boundaries, spawn_only_events_by_capacity = find_charging_events_step_boundaries(spawn_only)
    vspans = [
        {
            "xmin": xmin, "xmax": xmax, "kind": "transition",
            "label": "charging-events step" if i == 0 else None,
        }
        for i, (xmin, xmax) in enumerate(step_boundaries)
    ]
    if step_boundaries:
        for xmin, xmax in step_boundaries:
            print(f"Charging-events step (spawn_only condition): {xmin:g}–{xmax:g} Wh "
                  f"({spawn_only_events_by_capacity[xmin]} -> {spawn_only_events_by_capacity[xmax]} events)")
    else:
        print("No charging-events step boundaries found in the spawn_only condition.")

    os.makedirs(FIGURES_DIR, exist_ok=True)
    generate_sensitivity_errorbar_plot(
        x=spawn_only["battery_wh"],
        y_mean=spawn_only["energy_wh_per_task_mean"],
        y_std=spawn_only["energy_wh_per_task_std"],
        xlabel="battery capacity (Wh)",
        ylabel="$E_{\\mathrm{tot}}$ / task (Wh)",
        output_dir=FIGURES_DIR,
        prefix="battery_sensitivity_energy",
        label="spawn_only" if has_full_noise else None,
        x2=full_noise["battery_wh"] if has_full_noise else None,
        y2_mean=full_noise["energy_wh_per_task_mean"] if has_full_noise else None,
        y2_std=full_noise["energy_wh_per_task_std"] if has_full_noise else None,
        label2="full_noise" if has_full_noise else None,
        vlines=vlines,
    )
    # Mechanism figure: energy/task stacked over charging distance and
    # charging events, to show *why* the energy curve is non-monotonic --
    # charging_events is a clean deterministic staircase under "spawn_only"
    # (identical across all 15 seeds per capacity), and the same capacity
    # bands blur into a fractional mean-with-spread under "full_noise" once
    # initial-SoC/task-duration randomization desynchronizes which task
    # trips the charging threshold across seeds.
    generate_sensitivity_mechanism_plot(
        x=spawn_only["battery_wh"],
        panels=[
            {
                "ylabel": "$E_{\\mathrm{tot}}$ / task (Wh)",
                "y_mean": spawn_only["energy_wh_per_task_mean"],
                "y_std": spawn_only["energy_wh_per_task_std"],
                "y2_mean": full_noise["energy_wh_per_task_mean"] if has_full_noise else None,
                "y2_std": full_noise["energy_wh_per_task_std"] if has_full_noise else None,
            },
            {
                "ylabel": "charging distance (m)",
                "y_mean": spawn_only["charging_distance_m_mean"],
                "y_std": spawn_only["charging_distance_m_std"],
                "y2_mean": full_noise["charging_distance_m_mean"] if has_full_noise else None,
                "y2_std": full_noise["charging_distance_m_std"] if has_full_noise else None,
            },
            # Charging-events panel dropped from the published figure -- the
            # staircase and the deployed-capacity plateau value now live in
            # the paper's body text instead of a third panel (2026-09-01).
            # The vspans below (computed from this same charging_events
            # data) still shade the step bands on the two remaining panels,
            # so this data is still visually referenced even without its
            # own panel. Left here commented out in case we want it back.
            # {
            #     "ylabel": "charging events",
            #     "y_mean": spawn_only["charging_events_mean"],
            #     "y_std": spawn_only["charging_events_std"],
            #     "y2_mean": full_noise["charging_events_mean"] if has_full_noise else None,
            #     "y2_std": full_noise["charging_events_std"] if has_full_noise else None,
            # },
        ],
        xlabel="battery capacity (Wh)",
        output_dir=FIGURES_DIR,
        prefix="battery_sensitivity_energy_mechanism",
        label="spawn_only" if has_full_noise else None,
        x2=full_noise["battery_wh"] if has_full_noise else None,
        label2="full_noise" if has_full_noise else None,
        vlines=vlines,
        vspans=vspans,
        # Bottom panel (charging distance) is high on the left, dropping to
        # a low plateau from the deployed capacity onward -- its lower-left
        # corner is clear.
        legend_in_panel=True,
        legend_panel=-1,
        legend_loc="lower left",
        legend_fontsize=13,
        # Exact-canvas output (default figsize (8, 4.2*2)=(8, 8.4)) so
        # soc_convergence.pdf -- a single n=1 panel at figsize (8, 4.2),
        # also crop=False -- renders as exactly half this figure's height,
        # matching one of its panels precisely. See convergence_soc.py.
        crop=False,
    )
    generate_sensitivity_errorbar_plot(
        x=spawn_only["battery_wh"],
        y_mean=spawn_only["runtime_sec_mean"],
        y_std=spawn_only["runtime_sec_std"],
        xlabel="battery capacity (Wh)",
        ylabel="evaluation time (s)",
        output_dir=FIGURES_DIR,
        prefix="battery_sensitivity_runtime",
        label="spawn_only" if has_full_noise else None,
        x2=full_noise["battery_wh"] if has_full_noise else None,
        y2_mean=full_noise["runtime_sec_mean"] if has_full_noise else None,
        y2_std=full_noise["runtime_sec_std"] if has_full_noise else None,
        label2="full_noise" if has_full_noise else None,
    )


if __name__ == "__main__":
    main()
