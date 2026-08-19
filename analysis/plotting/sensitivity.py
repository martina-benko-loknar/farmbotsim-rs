import matplotlib.pyplot as plt
from viz_utils import setup_latex_fonts
from convergence import EGO_COLOR, GRID_COLOR

# ============================================================================
# Sensitivity / Sweep Plots (mean +/- std across seeds)
# ============================================================================

# Reuses the project's black/blue two-series palette (see convergence.py) so
# a "spawn_only"-vs-"full_noise" condition overlay reads consistently with
# the EGO-vs-grid-search convergence plots.
SPAWN_ONLY_COLOR = EGO_COLOR
FULL_NOISE_COLOR = GRID_COLOR


# Reference-line styling for annotating a swept parameter's operating points
# on top of a sensitivity curve -- e.g. the deployed default or an
# externally-found optimum (see add_reference_lines below). Kept distinct
# from SPAWN_ONLY_COLOR/FULL_NOISE_COLOR so reference lines never get
# confused with a data series in the legend.
REFERENCE_LINE_STYLES = {
    "deployed": dict(color="#555555", linestyle=":", linewidth=1.75),
    "optimum": dict(color="#D55E00", linestyle="--", linewidth=1.75),
}


def add_reference_lines(ax, vlines):
    """
    Draw vertical reference lines marking specific x-values on a sensitivity
    plot -- e.g. the deployed default parameter value or an EGO-found
    optimum -- on top of whatever data series are already plotted.

    `vlines` is a list of dicts, each either:
      {"x": <value>, "label": <str>, "kind": "deployed" | "optimum"}
    or with an explicit style overriding `kind`:
      {"x": <value>, "label": <str>, "color": ..., "linestyle": ...}
    Pass None/empty (default in callers) to skip -- existing plots without
    reference points are unaffected.
    """
    for v in vlines or []:
        style = dict(REFERENCE_LINE_STYLES.get(v.get("kind"), {}))
        style.update({k: val for k, val in v.items() if k not in ("x", "label", "kind")})
        ax.axvline(v["x"], label=v.get("label"), **style)


# Shaded-region styling for marking an x-range rather than a single point --
# e.g. the capacity band over which a discrete quantity (charging_events)
# steps from one integer to the next under the "spawn_only" condition. See
# add_reference_spans.
REFERENCE_SPAN_STYLES = {
    "transition": dict(color="#999999", alpha=0.18, linewidth=0),
}


def add_reference_spans(ax, vspans):
    """
    Shade x-ranges on a sensitivity plot -- e.g. the capacity band between
    two swept points where a "spawn_only"-condition step actually happens
    (found from the raw per-seed data, not derivable from the plotted means
    alone).

    `vspans` is a list of dicts, each either:
      {"xmin": <value>, "xmax": <value>, "label": <str>, "kind": "transition"}
    or with an explicit style overriding `kind`. Pass None/empty (default in
    callers) to skip.
    """
    for v in vspans or []:
        style = dict(REFERENCE_SPAN_STYLES.get(v.get("kind"), {}))
        style.update({k: val for k, val in v.items() if k not in ("xmin", "xmax", "label", "kind")})
        ax.axvspan(v["xmin"], v["xmax"], label=v.get("label"), **style)


def generate_sensitivity_errorbar_plot(
    x,
    y_mean,
    y_std,
    xlabel: str,
    ylabel: str,
    output_dir: str = "results",
    prefix: str = "sensitivity",
    label: str = None,
    x2=None,
    y2_mean=None,
    y2_std=None,
    label2: str = None,
    vlines=None,
):
    """
    Plot mean +/- std of a metric against a swept parameter. Pass
    x2/y2_mean/y2_std (default None) to overlay a second series -- e.g.
    soc_sweep/battery_sweep's "spawn_only" vs "full_noise" randomization
    conditions (see loaders/json_loader.py::load_single_evaluation_results)
    -- on the same axes; the legend is only drawn once a second series (or an explicit
    `label` for the first) makes it meaningful.

    `vlines` (default None): reference lines to draw over the data, see
    `add_reference_lines`.
    """
    setup_latex_fonts(20)

    fig, ax = plt.subplots(figsize=(8, 6))

    has_second_series = y2_mean is not None
    color = SPAWN_ONLY_COLOR if has_second_series else "black"

    ax.errorbar(
        x, y_mean, yerr=y_std,
        fmt='o-', color=color, ecolor=color,
        elinewidth=1.5, capsize=5, markersize=7, linewidth=1.5,
        label=label,
    )

    if has_second_series:
        ax.errorbar(
            x2, y2_mean, yerr=y2_std,
            fmt='o-', color=FULL_NOISE_COLOR, ecolor=FULL_NOISE_COLOR,
            elinewidth=1.5, capsize=5, markersize=7, linewidth=1.5,
            label=label2,
        )

    add_reference_lines(ax, vlines)

    ax.set_xlabel(xlabel)
    ax.set_ylabel(ylabel)
    ax.tick_params(labelsize=20)
    ax.grid(True, linewidth=0.5, alpha=0.5)
    if has_second_series or label or vlines:
        ax.legend()

    plt.tight_layout()
    filename = f"{output_dir}/{prefix}"
    plt.savefig(f"{filename}.pdf", bbox_inches='tight')
    print(f"- {prefix}.pdf")
    plt.close()


def generate_sensitivity_mechanism_plot(
    x,
    panels,
    xlabel: str,
    output_dir: str = "results",
    prefix: str = "sensitivity_mechanism",
    label: str = None,
    x2=None,
    label2: str = None,
    vlines=None,
    vspans=None,
):
    """
    Stacked, N-panel version of generate_sensitivity_errorbar_plot sharing
    one x-axis: the usual top metric (e.g. energy/task) stacked over one or
    more further metrics (e.g. charging distance, charging events) meant to
    explain *why* the top metric moves the way it does. Same
    spawn_only/full_noise overlay, vlines, and (new) vspans support as the
    single-panel plot.

    `panels` is a list of dicts, top to bottom:
      {"ylabel": <str>, "y_mean": ..., "y_std": ...,
       "y2_mean": ... (optional), "y2_std": ... (optional)}
    Whether the second ("full_noise") series is drawn is decided once, from the
    first panel's y2_mean -- pass y2_mean/y2_std on every panel together, or
    on none.

    `vspans` (default None): shaded x-range annotations, see
    `add_reference_spans` -- e.g. a "spawn_only"-condition step boundary found in
    the raw per-seed data that isn't visible from the plotted means alone.
    Drawn on every panel; the label is only kept on the top panel's legend.
    """
    setup_latex_fonts(20)

    n = len(panels)
    fig, axes = plt.subplots(
        n, 1, figsize=(8, 4.2 * n), sharex=True,
        gridspec_kw={"height_ratios": [1] * n},
        constrained_layout=True,
    )
    if n == 1:
        axes = [axes]

    has_second_series = panels[0].get("y2_mean") is not None
    color = SPAWN_ONLY_COLOR if has_second_series else "black"

    for i, (ax, panel) in enumerate(zip(axes, panels)):
        is_top = i == 0
        ax.errorbar(
            x, panel["y_mean"], yerr=panel["y_std"],
            fmt='o-', color=color, ecolor=color,
            elinewidth=1.5, capsize=5, markersize=7, linewidth=1.5,
            label=label if is_top else None,
        )
        if has_second_series:
            ax.errorbar(
                x2, panel["y2_mean"], yerr=panel["y2_std"],
                fmt='o-', color=FULL_NOISE_COLOR, ecolor=FULL_NOISE_COLOR,
                elinewidth=1.5, capsize=5, markersize=7, linewidth=1.5,
                label=label2 if is_top else None,
            )
        add_reference_lines(ax, vlines if is_top else [
            {**v, "label": None} for v in (vlines or [])
        ])
        add_reference_spans(ax, vspans if is_top else [
            {**v, "label": None} for v in (vspans or [])
        ])
        ax.set_ylabel(panel["ylabel"])
        ax.tick_params(labelsize=20)
        ax.grid(True, linewidth=0.5, alpha=0.5)

    axes[-1].set_xlabel(xlabel)
    if has_second_series or label or vlines or vspans:
        # Placed above the top panel rather than inside any corner: with up
        # to 4 entries (spawn_only/full_noise plus vlines/vspans), no in-axes corner
        # is reliably clear across every panel here -- battery_sensitivity_
        # energy_mechanism's data spans nearly the whole plot area both left
        # (spawn_only) and right (full_noise), so any corner box clips something.
        # constrained_layout=True reflows to make room automatically.
        axes[0].legend(
            loc="lower center", bbox_to_anchor=(0.5, 1.02),
            ncol=2, borderaxespad=0, fontsize=16,
        )

    filename = f"{output_dir}/{prefix}"
    plt.savefig(f"{filename}.pdf", bbox_inches='tight')
    print(f"- {prefix}.pdf")
    plt.close()
