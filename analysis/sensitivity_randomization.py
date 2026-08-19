import os
import sys
import pandas as pd

from loaders.json_loader import load_randomization_sweep_results, FIELD_SIZE_ORDER
from aggregation.aggregate import summarize
from results_dir import resolve_results_root, resolve_profile, results_root_tag

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "plotting"))
from comparison import generate_grouped_bar_comparison_plot

EXPORTS_BASE_DIR = "exports"

# One shared script for both randomization axes: initial_soc_sweep and
# task_duration_jitter_sweep are structurally identical (same
# (field_size, fleet_size, seed) grid, same fixed-vs-randomized comparison,
# same single_evaluation record shape -- see
# loaders/json_loader.py::load_randomization_sweep_results), so there's
# nothing axis-specific left beyond which sweep directory/output prefix to
# use. Add a row here to cover a future randomization axis the same way.
AXES = [
    ("initial_soc_sweep", "initial_soc"),
    ("task_duration_jitter_sweep", "task_duration_jitter"),
]


def process_axis(results_root, profile, sweep_name, prefix):
    results_dir = f"{results_root}/raw/{profile}/{sweep_name}"
    figures_dir = f"{results_root}/figures/{profile}/{sweep_name}"
    exports_dir = f"{EXPORTS_BASE_DIR}/{results_root_tag(results_root)}/{profile}"

    df = load_randomization_sweep_results(results_dir)
    print(f"\n===== {sweep_name} =====")
    print(f"Loaded {len(df)} runs from {results_dir}")
    if df.empty:
        print("No data -- skipping.")
        return

    df["field_size"] = pd.Categorical(
        df["field_size"], categories=FIELD_SIZE_ORDER, ordered=True
    )

    # Energy per completed task rather than total energy, matching
    # sensitivity_soc.py/comparison_*.py: task-count termination is a >=
    # check, so completed_tasks can drift a little seed-to-seed even at
    # fixed field/fleet size.
    df["energy_wh_per_task"] = df["energy_wh"] / df["completed_tasks"]

    summary = summarize(
        df,
        group_cols=["field_size", "fleet_size", "condition"],
        value_cols=["energy_wh_per_task", "simulation_time_sec"],
    )

    fixed = summary[summary["condition"] == "fixed"].drop(columns="condition")
    randomized = summary[summary["condition"] == "randomized"].drop(columns="condition")

    # Inner join: only (field_size, fleet_size) groups with both conditions
    # present are comparable. Both conditions are run for every combo by
    # construction (run_initial_soc_sweep_for / run_task_duration_jitter_
    # sweep_for always run both), so this should keep every real group.
    merged = fixed.merge(
        randomized, on=["field_size", "fleet_size"], suffixes=("_fixed", "_randomized")
    )

    # field_size is a Categorical (see above), so groupby's default
    # observed=False fills in *every* category x fleet_size combination --
    # including ones with zero rows in df, e.g. a field size the sweep
    # hasn't been run for yet -- as an all-NaN group rather than omitting
    # it. Drop those before plotting/exporting; a genuine sweep that only
    # partially completed should show up as missing groups, not NaN bars.
    merged = merged[(merged["n_fixed"] > 0) & (merged["n_randomized"] > 0)]
    merged = merged.sort_values(["field_size", "fleet_size"])

    if merged.empty:
        print("Missing one of the two conditions for every group -- nothing to compare.")
        return

    # The question these sweeps exist to answer: does this axis add
    # seed-to-seed variance beyond what spawn-position randomization
    # already gives? >1 means yes, for that (field_size, fleet_size) group.
    merged["energy_std_ratio_randomized_over_fixed"] = (
        merged["energy_wh_per_task_std_randomized"] / merged["energy_wh_per_task_std_fixed"]
    )
    print(merged[[
        "field_size", "fleet_size", "n_fixed", "n_randomized",
        "energy_wh_per_task_mean_fixed", "energy_wh_per_task_std_fixed",
        "energy_wh_per_task_mean_randomized", "energy_wh_per_task_std_randomized",
        "energy_std_ratio_randomized_over_fixed",
    ]].to_string(index=False))

    os.makedirs(exports_dir, exist_ok=True)
    summary_path = f"{exports_dir}/{prefix}_sensitivity_summary.csv"
    merged.to_csv(summary_path, index=False)
    print(f"Saved summary to {summary_path}")

    os.makedirs(figures_dir, exist_ok=True)

    # One figure per field size (x-axis = fleet size), rather than one
    # figure with all 12 field/fleet combos crowded onto its x-axis --
    # matches how comparison_fleet_slots.py facets its two-series bars.
    for field_size in FIELD_SIZE_ORDER:
        subset = merged[merged["field_size"] == field_size].sort_values("fleet_size")
        if subset.empty:
            continue
        group_labels = subset["fleet_size"].astype(str).tolist()

        generate_grouped_bar_comparison_plot(
            group_labels,
            subset["energy_wh_per_task_mean_fixed"], subset["energy_wh_per_task_std_fixed"],
            subset["energy_wh_per_task_mean_randomized"], subset["energy_wh_per_task_std_randomized"],
            xlabel="fleet size (/)", ylabel="$E_{\\mathrm{tot}}$ / task (Wh)",
            output_dir=figures_dir, prefix=f"{prefix}_comparison_energy_{field_size}",
            series_a_label="fixed", series_b_label="randomized",
        )

        generate_grouped_bar_comparison_plot(
            group_labels,
            subset["simulation_time_sec_mean_fixed"], subset["simulation_time_sec_std_fixed"],
            subset["simulation_time_sec_mean_randomized"], subset["simulation_time_sec_std_randomized"],
            xlabel="fleet size (/)", ylabel="mission time (s)",
            output_dir=figures_dir, prefix=f"{prefix}_comparison_mission_time_{field_size}",
            series_a_label="fixed", series_b_label="randomized",
        )


def main():
    results_root = resolve_results_root()
    profile = resolve_profile()

    for sweep_name, prefix in AXES:
        process_axis(results_root, profile, sweep_name, prefix)


if __name__ == "__main__":
    main()
