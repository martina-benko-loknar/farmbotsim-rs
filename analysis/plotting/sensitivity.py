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
    figsize=(8, 6),
    crop: bool = True,
    margins: dict = None,
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

    `figsize` (default (8, 6)): pass e.g. (8, 4.2) to match the single-panel
    mechanism-plot family (generate_sensitivity_mechanism_plot's default
    height-per-panel, see soc_convergence.pdf / convergence_soc.py) instead
    of this function's own historical default.

    `crop` (default True): crop the saved PDF to its tight content bbox, as
    usual. Pass False together with a `figsize` matching another figure's to
    render at the literal same canvas size regardless of content -- see
    generate_sensitivity_mechanism_plot's `crop` docstring for why cropping
    alone can leave nominally-same-figsize plots at different sizes.

    `margins` (default None): explicit {left, right, top, bottom} fractional
    axes margins (`fig.subplots_adjust`), applied instead of
    `plt.tight_layout()`. Pass e.g. `SINGLE_PANEL_MECHANISM_MARGINS` (see
    plotting/convergence.py) so this plot's axes box lands exactly where a
    companion figure's does regardless of content -- tight_layout alone
    still sizes the box to this plot's own content (a legend, more/fewer
    tick digits), which drifts from a same-figsize companion plot's box.
    """
    setup_latex_fonts(20)

    fig, ax = plt.subplots(figsize=figsize)

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

    if margins:
        fig.subplots_adjust(**margins)
    else:
        plt.tight_layout()
    filename = f"{output_dir}/{prefix}"
    plt.savefig(f"{filename}.pdf", bbox_inches='tight' if crop else None)
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
    figsize=None,
    legend_in_panel=False,
    legend_panel=-1,
    legend_loc="upper left",
    legend_fontsize=16,
    crop: bool = True,
    margins: dict = None,
    label_fontsize: int = None,
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

    `figsize` (default None): overrides the usual (8, 4.2 * n_panels) sizing
    -- e.g. a 1-panel call site pairing this plot side by side with another
    figure of a fixed height (like generate_..._convergence's (8, 6)) can
    pass that height explicitly so the two don't end up visibly mismatched.

    `crop` (default True): crop the saved PDF to its tight content bbox, as
    usual. Pass False together with an explicit `figsize` matching the
    paired figure's -- content-dependent tight-cropping (a legend, or its
    absence, on one side) can otherwise leave two same-figsize plots at
    slightly different aspect ratios, so they render at visibly different
    heights once LaTeX scales both to the same subfigure width. With
    crop=False the saved page is the full, uncropped figsize canvas, so two
    plots sharing a figsize are pixel-identical regardless of content.

    `legend_in_panel` (default False): draw the combined legend inside one
    panel's own axes (single column, standard matplotlib corner placement
    via `legend_loc`/`legend_panel`, sized via `legend_fontsize`) instead of
    the default above-the-top-panel strip. Only sensible when that corner
    actually stays clear of the panel's data across its full box width, not
    just at a glance -- a long label can still reach into a "mostly empty"
    corner (see battery_sensitivity_energy_mechanism's shortened vspan
    label and reduced legend_fontsize, needed for exactly this reason).

    `margins` (default None): explicit {left, right, top, bottom} fractional
    axes margins (see `fig.subplots_adjust`), applied instead of the default
    constrained_layout. Pass the same dict to a paired figure (e.g.
    generate_multi_convergence_plot's `PAIRED_FIGURE_MARGINS`) so both get
    the literal same axes box regardless of content -- constrained_layout
    alone still sizes the box to each plot's own content (a legend on one,
    none on the other), which is what leaves two same-figsize paired plots
    visibly mismatched. Only meaningful for n=1 (matching a single-axes
    companion plot); constrained_layout stays on whenever `margins` is None,
    same as before.

    `label_fontsize` (default None): font size for the x/y axis labels only
    (tick labels stay at whatever `setup_latex_fonts` set). Use to make a
    figure's axis labels stand out beyond the shared base size without
    changing every other calibrated figure that shares that base.
    """
    setup_latex_fonts(20)

    n = len(panels)
    fig, axes = plt.subplots(
        n, 1, figsize=figsize or (8, 4.2 * n), sharex=True,
        gridspec_kw={"height_ratios": [1] * n},
        constrained_layout=margins is None,
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
        ax.set_ylabel(panel["ylabel"], fontsize=label_fontsize)
        ax.tick_params(labelsize=20)
        ax.grid(True, linewidth=0.5, alpha=0.5)

    axes[-1].set_xlabel(xlabel, fontsize=label_fontsize)
    if has_second_series or label or vlines or vspans:
        if legend_in_panel:
            # Caller has confirmed the target panel/corner is actually
            # clear of data -- see the `legend_in_panel` docstring above.
            # Handles/labels live on axes[0] (only the top panel's artists
            # are labeled, per the is_top logic above), so gather them from
            # there even when placing the legend on a different panel.
            handles, labels_ = axes[0].get_legend_handles_labels()
            axes[legend_panel].legend(
                handles, labels_, loc=legend_loc, fontsize=legend_fontsize,
            )
        else:
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

    if margins:
        fig.subplots_adjust(**margins)

    filename = f"{output_dir}/{prefix}"
    plt.savefig(f"{filename}.pdf", bbox_inches='tight' if crop else None)
    print(f"- {prefix}.pdf")
    plt.close()
