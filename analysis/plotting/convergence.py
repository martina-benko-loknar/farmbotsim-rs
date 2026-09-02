import matplotlib.pyplot as plt
from viz_utils import setup_latex_fonts

# Okabe-Ito colorblind-safe pair: black (EGO) vs blue (grid search).
EGO_COLOR = "black"
GRID_COLOR = "#0072B2"

# 4-class sequential blue ramp (ColorBrewer "Blues", skipping the near-white
# lightest class for visibility), shared by field_sweep's and fleet_sweep's
# convergence overlays -- both field size and fleet size are ordinal
# variables (S < M < L < XL; 1 < 2 < 3 < 4), so a single-hue ramp
# communicates that ordering; a qualitative/categorical palette would imply
# the sizes are unordered, which they aren't. fleet_sweep previously used a
# separate green ramp so the two overlays would stay visually distinct if
# they ever appeared near each other -- reverted to sharing this same blue
# ramp instead (2026-09-02): they never actually appear in the same figure,
# and the paper's other two-series comparisons all use blue as the one
# recurring "second series" hue (GRID_COLOR above, SPAWN_ONLY/FULL_NOISE in
# sensitivity.py), so a second, unrelated hue family (green) here read as
# an extra, unexplained color system rather than reinforcing that
# convention.
_SEQUENTIAL_BLUE_RAMP = ["#bdd7e7", "#6baed6", "#3182bd", "#08519c"]
FIELD_SIZE_COLORS = dict(zip(["S", "M", "L", "XL"], _SEQUENTIAL_BLUE_RAMP))
FLEET_SIZE_COLORS = dict(zip([1, 2, 3, 4], _SEQUENTIAL_BLUE_RAMP))

# Explicit shared margins for the field/fleet-sweep paired figures (the
# energy/time-mechanism plot from generate_sensitivity_mechanism_plot vs.
# this module's own generate_multi_convergence_plot, both crop=False at the
# same figsize). Auto layout (constrained_layout / tight_layout) sizes the
# axes box to its own content -- a legend on one, none on the other -- so
# even with matching figsize and canvas the two plots' actual axes
# rectangles came out different sizes (visibly so once LaTeX scales both to
# the same subfigure width, see the 2026-09-02 conversation). Fixed
# fractional margins sidestep that: passing this dict to both plots'
# `margins` argument gives them the literal same axes box regardless of
# content.
#
# Tightened (2026-09-02) to trim the whitespace between the axes box and
# the subfigure edge -- left/right/top/bottom re-measured down to roughly
# the minimum that still clears the rotated y-label + y-tick digits (left),
# the topmost data/tick label (top), the rightmost x-tick label's overhang
# (right), and the x-label + x-tick row (bottom) at the shared
# setup_latex_fonts(20) base, instead of the more generous margins left
# over from earlier tuning passes. `left` initially went to 0.10 but
# clipped fleet_energy_time_mechanism.pdf's rotated y-label -- fleet's own
# y-ticks ("1.00", "0.95", ...) are wider than field's ("0.8", "0.6", ...),
# and this same margin is shared by both (see the docstring above for why),
# so it has to clear the wider of the two; 0.12 (close to
# SINGLE_PANEL_MECHANISM_MARGINS' left, independently tuned for a similarly
# wide "1.05"-style tick label) does. PAIRED_FIGURE_FIGSIZE's height is
# re-solved alongside so the box's own width:height ratio still matches
# fleet_slots_comparison_energy.pdf's (~1.86, see
# SINGLE_PANEL_MECHANISM_MARGINS/SINGLE_PANEL_MECHANISM_FIGSIZE) --
# shrinking margins alone, without re-deriving the height, would have
# drifted the ratio.
PAIRED_FIGURE_MARGINS = dict(left=0.12, right=0.9813, top=0.9674, bottom=0.1629)

# Paired with PAIRED_FIGURE_MARGINS above. Box width = (0.9813-0.12)*8 =
# 6.8904in; target box height = 6.8904/1.86 = 3.7045in (matching
# fleet_slots_comparison_energy.pdf's box ratio); this figsize height
# (4.6045in) is solved so (0.9674-0.1629)*4.6045 lands on that 3.7045in
# target (2026-09-02).
PAIRED_FIGURE_FIGSIZE = (8, 4.6045)

# Explicit axes-box size/margins for the standalone single-panel figures
# that should read as one family with battery_sensitivity_energy_mechanism.
# pdf / soc_sensitivity_energy_mechanism.pdf's own panels (generate_
# sensitivity_mechanism_plot, n=2, each panel's box independently confirmed
# ~6.99in x 3.755in at 300dpi, 2026-09-02 measurement) -- those two 2-panel
# figures are the reference and are NOT generated via this dict; everything
# else in the family (soc_convergence.pdf, baseline_comparison_multiagent_
# energy.pdf, fleet_slots_comparison_energy.pdf) matches them by using it.
#
# A standalone single-panel figure can't just reuse the reference panel's
# OWN top/bottom margins: an inner panel of a 2-panel figure has ~0 margin
# above it (nothing above panel 1) and needs no room below it for an
# xlabel/tick row (that's panel 2's job, since sharex hides panel 1's own
# x-ticks) -- a standalone figure needs both. So instead: same absolute
# left/right margins (same 8in-wide canvas, so these transfer as identical
# fractions), same absolute box HEIGHT, but reconstructed top/bottom margins
# sized for a self-contained plot (top margin matching the reference's
# above-panel-1 margin; bottom margin matching the reference's below-panel-2
# margin, which already includes its xlabel + tick row at the same 20pt
# font). That pins figsize height to 4.5133in instead of the old "half of
# 8.4" convention (4.2in) -- see SINGLE_PANEL_MECHANISM_FIGSIZE.
SINGLE_PANEL_MECHANISM_FIGSIZE = (8, 4.5133)
SINGLE_PANEL_MECHANISM_MARGINS = dict(left=0.1217, right=0.995, top=0.9911, bottom=0.1592)


def generate_convergence_comparison_plot(
    ego_x, ego_mean, ego_std=None,
    grid_x=None, grid_mean=None, grid_std=None,
    output_dir: str = "results",
    prefix: str = "convergence",
    ylabel: str = "$E_{\\mathrm{tot}}$ (Wh), best found so far",
    markers=False,
):
    """
    Best-energy-found-so-far vs number of evaluations for EGO. Also draws
    the grid-search curve when grid_x/grid_mean/grid_std are given -- pass
    None (default) for EGO-only data, e.g. fleet_sweep/field_sweep, which
    no longer runs grid search alongside EGO.

    ego_std/grid_std are only meaningful when ego_mean/grid_mean are
    aggregated across multiple seeds (see comparison_fleet.py /
    convergence_fleet.py) -- pass None (default) for a single run, e.g.
    single_station_viz.py, which has no seed-to-seed spread to shade. The
    shaded band is skipped entirely rather than drawn at a fake std=0
    (which happens to render as nothing, but isn't a reason to pass it) --
    the goal is that any gray on this plot legitimately comes from
    ax.grid(...), not from a std band pretending there's variance to show.

    `markers` (default False): draw a dot at each evaluation, matching this
    paper's sweep/sensitivity plots (dots-connected-by-lines). Off by
    default since a long, mostly-flat trace (e.g. field_sweep/fleet_sweep's
    ~90-evaluation curves) turns into a redundant row of dots on the
    plateau -- opt in for short traces where that's not an issue (e.g.
    convergence_soc.py's 26 evaluations).
    """
    setup_latex_fonts(20)

    fig, ax = plt.subplots(figsize=(8, 6))
    fmt = "o-" if markers else "-"

    ax.plot(ego_x, ego_mean, fmt, color=EGO_COLOR, linewidth=1.5, markersize=7, label="EGO")
    if ego_std is not None:
        ax.fill_between(
            ego_x, ego_mean - ego_std, ego_mean + ego_std,
            color=EGO_COLOR, alpha=0.15, linewidth=0,
        )

    if grid_mean is not None:
        ax.plot(grid_x, grid_mean, fmt, color=GRID_COLOR, linewidth=1.5, markersize=7, label="Grid search")
        if grid_std is not None:
            ax.fill_between(
                grid_x, grid_mean - grid_std, grid_mean + grid_std,
                color=GRID_COLOR, alpha=0.15, linewidth=0,
            )

    ax.set_xlabel("evaluations")
    ax.set_ylabel(ylabel)
    ax.tick_params(labelsize=20)
    ax.grid(True, linewidth=0.5, alpha=0.5)
    if grid_mean is not None:
        # Only meaningful once there's a second curve to disambiguate --
        # a lone "EGO" legend entry on a single-series plot (e.g.
        # convergence_soc.py, no grid-search comparison) is clutter, and
        # inconsistent with this paper's convention of omitting the legend
        # for an unambiguous single series (see e.g.
        # generate_sensitivity_errorbar_plot).
        ax.legend()

    plt.tight_layout()
    filename = f"{output_dir}/{prefix}"
    plt.savefig(f"{filename}.pdf", bbox_inches="tight")
    print(f"- {prefix}.pdf")
    plt.close()


def generate_multi_convergence_plot(
    curves,
    output_dir: str = "results",
    prefix: str = "convergence_overlay",
    xlabel: str = "evaluations",
    ylabel: str = "\\% above best found",
    crop: bool = True,
    margins: dict = None,
    label_fontsize: int = None,
    figsize=(8, 6),
):
    """
    Overlay several best-so-far convergence curves on one axes -- e.g. one
    per field size -- each with its own shaded cross-seed std band. Unlike
    `generate_convergence_comparison_plot`, this isn't an EGO-vs-grid-search
    plot: every curve here is drawn the same way (no "-vs-" framing), meant
    for comparing EGO's own behavior across a swept parameter instead.

    `curves` is a list of dicts: {"label": str, "x": array, "mean": array,
    "std": array (optional), "color": str (optional, falls back to
    matplotlib's default cycle)}.

    `crop` (default True): crop the saved PDF to its tight content bbox, as
    usual. Pass False when this figure is paired side by side in the paper
    with another figure at the same nominal `figsize` (e.g.
    comparison_field.py's energy/time-mechanism plot) -- content-dependent
    tight-cropping (this plot's legend vs. the paired plot's lack of one)
    otherwise leaves the two at slightly different aspect ratios even
    though they share a figsize, so they render at visibly different
    heights once LaTeX scales both to the same subfigure width. With
    crop=False the saved page is the full, uncropped figsize canvas, so two
    plots sharing a figsize are pixel-identical in size regardless of what's
    inside them.

    `margins` (default None): explicit {left, right, top, bottom} fractional
    axes margins (see `fig.subplots_adjust`), applied instead of
    `plt.tight_layout()`. Pass the same dict to the paired figure (e.g.
    `PAIRED_FIGURE_MARGINS` above) so both get the literal same axes box
    regardless of content -- tight_layout alone still sizes the box to each
    plot's own content (a legend here, none there), which is what left the
    paired figures visibly mismatched in the first place.

    `label_fontsize` (default None): font size for the x/y axis labels only
    (tick labels stay at whatever `setup_latex_fonts` set). Use to make a
    figure's axis labels stand out beyond the shared base size without
    changing every other calibrated figure that shares that base.

    `figsize` (default (8, 6)): pass e.g. `PAIRED_FIGURE_FIGSIZE` to match a
    paired figure's canvas -- see that constant's docstring above.
    """
    setup_latex_fonts(20)

    fig, ax = plt.subplots(figsize=figsize)

    for curve in curves:
        color = curve.get("color")
        ax.plot(curve["x"], curve["mean"], "-", color=color, linewidth=1.5, label=curve["label"])
        if curve.get("std") is not None:
            ax.fill_between(
                curve["x"], curve["mean"] - curve["std"], curve["mean"] + curve["std"],
                color=color, alpha=0.15, linewidth=0,
            )

    ax.set_xlabel(xlabel, fontsize=label_fontsize)
    ax.set_ylabel(ylabel, fontsize=label_fontsize)
    ax.tick_params(labelsize=20)
    ax.grid(True, linewidth=0.5, alpha=0.5)
    ax.legend()

    if margins:
        fig.subplots_adjust(**margins)
    else:
        plt.tight_layout()
    filename = f"{output_dir}/{prefix}"
    plt.savefig(f"{filename}.pdf", bbox_inches="tight" if crop else None)
    print(f"- {prefix}.pdf")
    plt.close()
