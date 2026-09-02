import os
import sys

from loaders.json_loader import load_single_evaluation_results, load_latest_soc_optimum
from aggregation.aggregate import summarize
from results_dir import resolve_results_root, resolve_profile, results_root_tag

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "plotting"))
from sensitivity import generate_sensitivity_errorbar_plot, generate_sensitivity_mechanism_plot

SWEEP_NAME = "soc_sweep"
EXPORTS_BASE_DIR = "exports"

# Deployed default (ExperimentConfig::for_profile, src/experiment/config.rs)
# -- not read from a sweep file since every soc_sweep run has
# soc_threshold_percent set to the value under test, not the default.
DEPLOYED_SOC_THRESHOLD_PERCENT = 60.0


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
    # threshold/seed (see loaders/json_loader.py::load_single_evaluation_results):
    # "spawn_only" (spawn-position noise only, the original sweep) vs "full_noise"
    # (initial-SoC + task-duration-jitter also on -- a robustness check, see
    # various/randomness_axes_sweep_design.txt). Pre-`cond=` result
    # directories only ever have "spawn_only".
    summary = summarize(
        df,
        group_cols=["soc_threshold", "condition"],
        value_cols=["energy_wh_per_task", "distance_m", "charging_distance_m", "runtime_sec"],
    ).sort_values("soc_threshold")

    print(summary)

    os.makedirs(EXPORTS_DIR, exist_ok=True)
    summary_path = f"{EXPORTS_DIR}/soc_sensitivity_summary.csv"
    summary.to_csv(summary_path, index=False)
    print(f"Saved summary to {summary_path}")

    spawn_only = summary[summary["condition"] == "spawn_only"]
    full_noise = summary[summary["condition"] == "full_noise"]
    has_full_noise = not full_noise.empty

    # Reference lines: the deployed default threshold, and (if a Level II
    # `--optimize-soc` run is available) the EGO-found optimum -- see
    # load_latest_soc_optimum's docstring for why this isn't gated behind
    # resolve_results_root(). Both land close to each other here (deployed
    # 60% vs. EGO ~59.7%), which is itself the point: the deployed default
    # is already near the optimizer's answer.
    vlines = [{
        "x": DEPLOYED_SOC_THRESHOLD_PERCENT,
        "label": f"deployed default (${DEPLOYED_SOC_THRESHOLD_PERCENT:.0f}\\%$)",
        "kind": "deployed",
    }]
    soc_optimum = load_latest_soc_optimum(profile=profile)
    if soc_optimum is not None:
        theta = soc_optimum["optimal_threshold_percent"]
        vlines.append({
            "x": theta,
            "label": f"EGO optimum (${theta:.1f}\\%$)",
            "kind": "optimum",
        })
        print(f"Annotating with EGO SoC-threshold optimum from {soc_optimum['file']}: "
              f"{theta:.2f}% ({soc_optimum['energy_wh']:.1f} Wh, "
              f"{soc_optimum['completed_tasks']} tasks)")
    else:
        print("No --optimize-soc result found -- energy plot will only show the deployed-default line.")

    os.makedirs(FIGURES_DIR, exist_ok=True)
    generate_sensitivity_errorbar_plot(
        x=spawn_only["soc_threshold"],
        y_mean=spawn_only["energy_wh_per_task_mean"],
        y_std=spawn_only["energy_wh_per_task_std"],
        xlabel="SoC threshold (\\%)",
        ylabel="$E_{\\mathrm{tot}}$ / task (Wh)",
        output_dir=FIGURES_DIR,
        prefix="soc_sensitivity_energy",
        label="spawn_only" if has_full_noise else None,
        x2=full_noise["soc_threshold"] if has_full_noise else None,
        y2_mean=full_noise["energy_wh_per_task_mean"] if has_full_noise else None,
        y2_std=full_noise["energy_wh_per_task_std"] if has_full_noise else None,
        label2="full_noise" if has_full_noise else None,
        vlines=vlines,
    )
    # Mechanism figure: energy/task stacked over charging distance, to show
    # *why* the energy curve has the shape it does -- charging distance
    # rises steadily with threshold (more frequent trips) while total
    # distance stays flat, so the energy story is about charging behavior,
    # not routing. See brainstorm discussion, item #2.
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
        prefix="soc_sensitivity_energy_mechanism",
        label="spawn_only" if has_full_noise else None,
        x2=full_noise["soc_threshold"] if has_full_noise else None,
        label2="full_noise" if has_full_noise else None,
        vlines=vlines,
        # Bottom panel (charging distance) rises monotonically across the
        # whole sweep, leaving its upper-left corner clear -- move the
        # legend in there instead of the default above-the-top-panel strip.
        legend_in_panel=True,
        legend_panel=-1,
        legend_loc="upper left",
    )
    generate_sensitivity_errorbar_plot(
        x=spawn_only["soc_threshold"],
        y_mean=spawn_only["runtime_sec_mean"],
        y_std=spawn_only["runtime_sec_std"],
        xlabel="SoC threshold (\\%)",
        ylabel="evaluation time (s)",
        output_dir=FIGURES_DIR,
        prefix="soc_sensitivity_runtime",
        label="spawn_only" if has_full_noise else None,
        x2=full_noise["soc_threshold"] if has_full_noise else None,
        y2_mean=full_noise["runtime_sec_mean"] if has_full_noise else None,
        y2_std=full_noise["runtime_sec_std"] if has_full_noise else None,
        label2="full_noise" if has_full_noise else None,
    )


if __name__ == "__main__":
    main()
