import os
import sys

from loaders.json_loader import load_single_station_results
from aggregation.aggregate import summarize
from results_dir import resolve_results_root, resolve_profile, results_root_tag

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "plotting"))
from sensitivity import generate_sensitivity_errorbar_plot
from convergence import SINGLE_PANEL_MECHANISM_FIGSIZE, SINGLE_PANEL_MECHANISM_MARGINS

# fleet_sweep: 1 charging slot regardless of fleet size (the historical
# setup -- see comparison_fleet.py). fleet_slots_sweep: slots matched 1:1 to
# fleet size (n_station_slots == n_agents), so queueing at the shared
# station is impossible. Comparing the two isolates whether fleet-size
# energy/task growth (see comparison_fleet.py's finding) is queueing
# contention on a single slot, rather than a travel-distance effect.
ONE_SLOT_SWEEP = "fleet_sweep"
MATCHED_SLOTS_SWEEP = "fleet_slots_sweep"
EXPORTS_BASE_DIR = "exports"

VALUE_COLS = ["ego_energy_wh_per_task", "ego_distance_m", "ego_simulation_time_sec"]


def load_and_summarize(results_root, profile, sweep_name):
    results_dir = f"{results_root}/raw/{profile}/{sweep_name}"
    df = load_single_station_results(results_dir)
    print(f"Loaded {len(df)} single-station runs from {results_dir}")
    if df.empty:
        return df

    # Energy per completed task rather than total energy, matching
    # comparison_fleet.py -- task count is fixed by n_tasks_target, not
    # fleet size, so total energy alone would conflate the two.
    df["ego_energy_wh_per_task"] = df["ego_energy_wh"] / df["ego_completed_tasks"]

    return summarize(
        df, group_cols=["fleet_size"], value_cols=VALUE_COLS
    ).sort_values("fleet_size")


def main():
    results_root = resolve_results_root()
    profile = resolve_profile()
    FIGURES_DIR = f"{results_root}/figures/{profile}/{MATCHED_SLOTS_SWEEP}/comparison"
    EXPORTS_DIR = f"{EXPORTS_BASE_DIR}/{results_root_tag(results_root)}/{profile}"

    one_slot = load_and_summarize(results_root, profile, ONE_SLOT_SWEEP)
    matched_slots = load_and_summarize(results_root, profile, MATCHED_SLOTS_SWEEP)

    if one_slot.empty or matched_slots.empty:
        print("Missing data for one of the two sweeps -- nothing to compare.")
        return

    # Both sweeps use the same fleet sizes (1, 2, 3, 4) by construction, so
    # an inner join on fleet_size should keep every row from both sides.
    merged = one_slot.merge(
        matched_slots, on="fleet_size", suffixes=("_1slot", "_matched")
    )
    print(merged)

    os.makedirs(EXPORTS_DIR, exist_ok=True)
    summary_path = f"{EXPORTS_DIR}/fleet_slots_comparison_summary.csv"
    merged.to_csv(summary_path, index=False)
    print(f"Saved summary to {summary_path}")

    os.makedirs(FIGURES_DIR, exist_ok=True)
    fleet_size = merged["fleet_size"]

    # Dots-connected-by-lines with error bars, matching the rest of the
    # paper's two-condition overlays (e.g. the SoC/battery sensitivity
    # sweeps) instead of a grouped bar chart.
    generate_sensitivity_errorbar_plot(
        x=fleet_size,
        y_mean=merged["ego_energy_wh_per_task_mean_1slot"], y_std=merged["ego_energy_wh_per_task_std_1slot"],
        x2=fleet_size,
        y2_mean=merged["ego_energy_wh_per_task_mean_matched"], y2_std=merged["ego_energy_wh_per_task_std_matched"],
        xlabel="fleet size (/)", ylabel="$E_{\\mathrm{tot}}$ / task (Wh)",
        output_dir=FIGURES_DIR, prefix="fleet_slots_comparison_energy",
        label="1 slot", label2="slots = fleet size",
        # figsize/margins reconstructed to land this plot's own axes box at
        # the literal same size as one panel of battery_sensitivity_energy_
        # mechanism.pdf / soc_sensitivity_energy_mechanism.pdf, instead of
        # this function's own larger (8, 6) default -- this is the only
        # figure of the pair actually published, see main.tex. `margins`
        # locks the box itself, rather than letting this plot's own legend
        # push tight_layout to a different box at the same nominal figsize
        # (2026-09-02).
        figsize=SINGLE_PANEL_MECHANISM_FIGSIZE,
        crop=False,
        margins=SINGLE_PANEL_MECHANISM_MARGINS,
    )

    generate_sensitivity_errorbar_plot(
        x=fleet_size,
        y_mean=merged["ego_simulation_time_sec_mean_1slot"], y_std=merged["ego_simulation_time_sec_std_1slot"],
        x2=fleet_size,
        y2_mean=merged["ego_simulation_time_sec_mean_matched"], y2_std=merged["ego_simulation_time_sec_std_matched"],
        xlabel="fleet size (/)", ylabel="mission time (s)",
        output_dir=FIGURES_DIR, prefix="fleet_slots_comparison_mission_time",
        label="1 slot", label2="slots = fleet size",
    )


if __name__ == "__main__":
    main()
