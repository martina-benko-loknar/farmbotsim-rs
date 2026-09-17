import os
import sys

from loaders.json_loader import load_single_evaluation_results
from aggregation.aggregate import summarize
from results_dir import resolve_results_root, resolve_profile, results_root_tag

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "plotting"))
from sensitivity import generate_sensitivity_mechanism_plot

# Companion to sensitivity_soc.py: same soc_sweep raw directory (fleet=1
# and fleet=3 runs coexist there via `--soc-sweep` / `--soc-sweep-fleet`,
# filenames encode fleet size), but filtered to one caller-chosen fleet
# size and without the EGO-optimum vline. sensitivity_soc.py's own
# `load_latest_soc_optimum` call always resolves the *latest* --optimize-soc
# run regardless of the sweep's own fleet size, which would mislabel this
# fleet=3 (or other) sweep with a fleet=1 optimizer result -- see
# TODO.md Phase 8 for why this needed a dedicated script rather than a flag
# on sensitivity_soc.py itself. Only the deployed-default line is shown
# here.
SWEEP_NAME = "soc_sweep"
EXPORTS_BASE_DIR = "exports"
FLEET_SIZE = int(sys.argv[1]) if len(sys.argv) > 1 else 3

DEPLOYED_SOC_THRESHOLD_PERCENT = 60.0


def main():
    results_root = resolve_results_root()
    profile = resolve_profile()
    RESULTS_DIR = f"{results_root}/raw/{profile}/{SWEEP_NAME}"
    FIGURES_DIR = f"{results_root}/figures/{profile}/{SWEEP_NAME}"
    EXPORTS_DIR = f"{EXPORTS_BASE_DIR}/{results_root_tag(results_root)}/{profile}"

    df = load_single_evaluation_results(RESULTS_DIR)
    df = df[df["fleet_size"] == FLEET_SIZE]
    print(f"Loaded {len(df)} single-evaluation runs at fleet_size={FLEET_SIZE} from {RESULTS_DIR}")

    df["energy_wh_per_task"] = df["energy_wh"] / df["completed_tasks"]

    summary = summarize(
        df,
        group_cols=["soc_threshold", "condition"],
        value_cols=["energy_wh_per_task", "distance_m", "charging_distance_m", "runtime_sec"],
    ).sort_values("soc_threshold")

    print(summary)

    os.makedirs(EXPORTS_DIR, exist_ok=True)
    summary_path = f"{EXPORTS_DIR}/soc_sensitivity_summary_fleet{FLEET_SIZE}.csv"
    summary.to_csv(summary_path, index=False)
    print(f"Saved summary to {summary_path}")

    spawn_only = summary[summary["condition"] == "spawn_only"]
    full_noise = summary[summary["condition"] == "full_noise"]
    has_full_noise = not full_noise.empty

    vlines = [{
        "x": DEPLOYED_SOC_THRESHOLD_PERCENT,
        "label": f"default (${DEPLOYED_SOC_THRESHOLD_PERCENT:.0f}\\%$)",
        "kind": "deployed",
    }]

    os.makedirs(FIGURES_DIR, exist_ok=True)
    generate_sensitivity_mechanism_plot(
        x=spawn_only["soc_threshold"],
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
        ],
        xlabel="SoC threshold (\\%)",
        output_dir=FIGURES_DIR,
        prefix=f"soc_sensitivity_energy_mechanism_fleet{FLEET_SIZE}",
        label="spawn_only" if has_full_noise else None,
        x2=full_noise["soc_threshold"] if has_full_noise else None,
        label2="full_noise" if has_full_noise else None,
        vlines=vlines,
        legend_in_panel=True,
        legend_panel=-1,
        legend_loc="upper left",
    )


if __name__ == "__main__":
    main()
