import os
import sys

from loaders.json_loader import load_latest_soc_trace
from aggregation.convergence import ego_best_so_far
from results_dir import resolve_results_root, resolve_profile, results_root_tag

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "plotting"))
from sensitivity import generate_sensitivity_mechanism_plot
from convergence import SINGLE_PANEL_MECHANISM_FIGSIZE, SINGLE_PANEL_MECHANISM_MARGINS

EXPORTS_BASE_DIR = "exports"

# Convergence curve for the Level II SoC-threshold Bayesian optimizer
# itself (optimize_soc_threshold_ego, --optimize-soc), i.e. best
# energy-per-task found so far vs. BO evaluation count -- the analogue of
# convergence_fleet.py/convergence_field.py for the Level I station-
# placement optimizer, mirrored here since SocEvaluationRecord has the same
# `evaluation`/`best_energy` shape ego_best_so_far already expects. Single
# run, no cross-seed std band, so no y_std passed below (a fake std=0 would
# render as nothing but isn't a reason to pass it).
#
# Plotted via generate_sensitivity_mechanism_plot (single n=1 panel) rather
# than the dedicated convergence-plot helper, specifically so this renders
# through the exact same layout engine/figsize convention as this section's
# other mechanism plots (e.g. battery_sensitivity_energy_mechanism) --
# see sensitivity_battery.py's matching figsize=(8, 4.2*2)/crop=False for
# the other half of this pairing.


def main():
    results_root = resolve_results_root()
    profile = resolve_profile()
    FIGURES_DIR = f"{results_root}/figures/{profile}/soc_optimization_exp"
    EXPORTS_DIR = f"{EXPORTS_BASE_DIR}/{results_root_tag(results_root)}/{profile}"

    trace = load_latest_soc_trace(profile=profile)
    if trace is None:
        print(f"No soc_optimization_exp data found for profile={profile} -- nothing to plot.")
        return

    print(f"Loaded SoC-optimization trace from {trace['file']}")
    evaluation_history = trace["evaluation_history"]

    x, y = ego_best_so_far(evaluation_history)

    os.makedirs(EXPORTS_DIR, exist_ok=True)
    os.makedirs(FIGURES_DIR, exist_ok=True)

    generate_sensitivity_mechanism_plot(
        x=x,
        panels=[
            {
                "ylabel": "$E_{\\mathrm{tot}}$ (Wh), best found so far",
                "y_mean": y,
                "y_std": None,
            },
        ],
        xlabel="evaluations",
        output_dir=FIGURES_DIR,
        prefix="soc_convergence",
        # figsize/margins reconstructed so this plot's own axes box lands at
        # the literal same size as one panel of battery_sensitivity_energy_
        # mechanism.pdf / soc_sensitivity_energy_mechanism.pdf -- see
        # SINGLE_PANEL_MECHANISM_FIGSIZE's docstring for the derivation
        # (2026-09-02, superseding the previous "half of (8, 4.2*2)"
        # canvas-only convention, which matched total canvas height but not
        # the visible plot rectangle).
        figsize=SINGLE_PANEL_MECHANISM_FIGSIZE,
        crop=False,
        # Explicit margins (rather than this function's default
        # constrained_layout) so this is the byte-for-byte reference box
        # for the rest of the single-panel family (baseline-comparison-
        # multiagent, fleet-slots-comparison, ...) to match via the same
        # SINGLE_PANEL_MECHANISM_MARGINS, instead of each figure's own
        # content nudging constrained_layout to a slightly different box.
        margins=SINGLE_PANEL_MECHANISM_MARGINS,
    )


if __name__ == "__main__":
    main()
