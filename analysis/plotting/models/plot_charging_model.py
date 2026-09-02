"""
Prototype: CC-CV charging model figure for the "Charging model" paragraph of
the paper's Experimental Setup subsection
(../agro-charging-framework/revisions/AE_2026_R1/main.tex, sec:experimental-setup).

Data source (real, not re-fit here): configs/batteries/leo_rover/config.json, loaded
directly so the plotted curve cannot drift from what the sim actually evaluates
(src/battery_module/charging/cc_cv.rs::CcCvChargingModel::compute_charge).

Model recap (as implemented, integrated analytically rather than stepped like the sim
does -- the two agree up to step-size discretization error):
  - Below cv_fraction*capacity: constant *power* cc_power (the paper's "CC" phase is a
    constant-power approximation, not literal constant current -- noted in the caption
    since it's worth being precise about given the model is named after the true-CC-CV
    behavior it approximates).
  - Above cv_fraction*capacity: exponential taper toward capacity with time constant
    cv_time_constant, i.e. remaining_gap(t) = remaining_gap(t_cv) * exp(-(t-t_cv)/tau).
  - "Full" is declared once the remaining gap drops below FULL_CHARGE_EPSILON (0.1%) of
    capacity, matching CcCvChargingModel::FULL_CHARGE_EPSILON (the exponential only
    approaches capacity asymptotically and never reaches it exactly).

NOTE on power continuity at the CC/CV transition: the CV phase's instantaneous power is
remaining_gap(t)/tau (the derivative of the exponential), so at the transition it equals
(1-cv_fraction)*capacity/tau -- a quantity that has no relation to cc_power unless tau is
deliberately chosen as (1-cv_fraction)*capacity/cc_power. leo_rover/config.json's
cv_time_constant is now set exactly this way (2440 s, derived from capacity=73.2 Wh,
cv_fraction=0.8, cc_power=21.6 W), so the power curve (no longer separately plotted, see
below) is continuous -- see src/battery_module/charging/cc_cv.rs's CcCvChargingModel doc
comment. Before this fix tau was 600 s (an assumed generic CV time constant, independent
of cc_power), which produced a real ~4x power jump at the transition; that is no longer
the case here, but is worth remembering if cv_time_constant is ever changed without
re-deriving it.

SINGLE-PANEL REVISION: unlike the discharge model (voltage-drop and energy-cost curves
are the *same* curve, related by one global scalar -- see plot_discharge_model.py), SoC
and charging power here are NOT a rescaling of each other: SoC(t) is the running integral
of power(t), and during the CC phase power is literally constant while SoC ramps linearly,
so no scalar k gives SoC(t) = k*power(t) globally (only piecewise, within the CV phase
alone, where SoC(t) = 100 - k*power(t) does hold by construction of tau above). So a twin
y-axis merge -- valid for the discharge model -- would misrepresent this one: it would
need two differently-shaped lines sharing independently-scaled axes, which is exactly the
dual-axis pattern that invites spurious visual correlations. Decided instead to drop the
power panel entirely and annotate the two governing parameters (P_cc, tau) directly on
the SoC curve, since the reader doesn't need the power curve's shape -- just its two
defining numbers, which the text/table already name. The power-panel code below is kept
commented out (not deleted) in case we want it back.
"""

import json
import re

import matplotlib.pyplot as plt
import numpy as np

REPO = "/home/martinabl/Projects/farmbotsim-rs"
CONFIG_PATH = f"{REPO}/configs/batteries/leo_rover/config.json"

FULL_CHARGE_EPSILON = 0.001  # CcCvChargingModel::FULL_CHARGE_EPSILON

OUT_STEM = "charging_model"

ENERGY_COLOR = "#0072B2"  # Okabe-Ito blue
POWER_COLOR = "#E69F00"   # Okabe-Ito orange (only used by the commented-out power panel below)
# Only the CV phase is shaded (the exponential-taper regime near full charge); the CC
# phase stays white. Gray matches the transition-band shading in analysis/plotting/
# sensitivity.py's REFERENCE_SPAN_STYLES["transition"] (battery_sensitivity_energy_
# mechanism.pdf's charging-events step bands), for the same "gray = a highlighted
# region" convention across the paper -- see plot_discharge_model.py's matching choice
# for the uphill span (2026-09-02).
CV_SPAN_COLOR = "#999999"
CV_SPAN_ALPHA = 0.18


def parse_quantity(s):
    """'21.6 W' -> 21.6, '600 s' -> 600.0, '73.2 Wh' -> 73.2 (unit is dropped;
    every quantity in this config happens to already be in base SI-ish units
    -- W, s, Wh -- so no conversion is needed)."""
    return float(re.match(r"[-+0-9.eE]+", s.strip()).group())


def setup_style():
    plt.rcParams["text.usetex"] = True
    plt.rcParams["font.family"] = "serif"
    plt.rcParams["font.serif"] = ["Computer Modern Roman"]
    base_fs = 15
    plt.rcParams["font.size"] = base_fs
    plt.rcParams["axes.labelsize"] = base_fs
    plt.rcParams["xtick.labelsize"] = base_fs - 2
    plt.rcParams["ytick.labelsize"] = base_fs - 2
    plt.rcParams["legend.fontsize"] = base_fs - 3


def main():
    with open(CONFIG_PATH) as f:
        cfg = json.load(f)

    capacity_wh = parse_quantity(cfg["capacity"])
    cc_power_w = parse_quantity(cfg["cc_power"])
    cv_fraction = cfg["cv_fraction"]
    tau_s = parse_quantity(cfg["cv_time_constant"])

    cv_energy_wh = capacity_wh * cv_fraction
    t_cv_s = cv_energy_wh / cc_power_w * 3600.0  # CC phase duration, at constant power

    # CV phase: solve for when the remaining gap first drops below the "full" epsilon.
    remaining_at_cv = capacity_wh - cv_energy_wh
    t_full_s = t_cv_s + tau_s * np.log(remaining_at_cv / (capacity_wh * FULL_CHARGE_EPSILON))

    t_s = np.linspace(0.0, t_full_s, 2000)
    cc_mask = t_s < t_cv_s

    energy_wh = np.empty_like(t_s)
    energy_wh[cc_mask] = cc_power_w * t_s[cc_mask] / 3600.0
    remaining = remaining_at_cv * np.exp(-(t_s[~cc_mask] - t_cv_s) / tau_s)
    energy_wh[~cc_mask] = capacity_wh - remaining

    # power_w = np.empty_like(t_s)
    # power_w[cc_mask] = cc_power_w
    # power_w[~cc_mask] = remaining / tau_s * 3600.0  # -d(remaining)/dt, remaining in Wh

    t_min = t_s / 60.0

    setup_style()
    # Single panel now (was: fig, (ax_e, ax_p) = plt.subplots(2, 1, figsize=(6.5, 6.8),
    # sharex=True) -- see SINGLE-PANEL REVISION note in the module docstring). Figsize
    # height chosen to match plot_discharge_model.py's panel aspect ratio (~1.95 w/h) for
    # visual consistency between the two subfigures.
    fig, ax_e = plt.subplots(figsize=(6.5, 3.46))

    for ax in (ax_e,):
        ax.axvspan(t_cv_s / 60.0, t_full_s / 60.0, color=CV_SPAN_COLOR, alpha=CV_SPAN_ALPHA, linewidth=0, zorder=0)
        ax.axvline(t_cv_s / 60.0, color="black", linewidth=0.8, linestyle=":", zorder=1)
        ax.grid(alpha=0.25, linewidth=0.5)

    ax_e.plot(t_min, 100.0 * energy_wh / capacity_wh, color=ENERGY_COLOR, linewidth=2.0)
    ax_e.axhline(100.0 * cv_fraction, color="black", linewidth=0.7, linestyle="--", zorder=1)
    ax_e.set_ylabel("SoC [\\%]")
    ax_e.set_xlabel("time [min]")
    ax_e.set_xlim(0, t_full_s / 60.0)
    ax_e.set_ylim(0, 103)
    ax_e.text(
        0.04, 0.94, "CC phase (constant power)\n"
        rf"$P_{{\mathrm{{cc}}}} = {cc_power_w:g}$ W", transform=ax_e.transAxes,
        ha="left", va="top", fontsize=11, color="0.35",
    )
    ax_e.text(
        0.96, 0.5, "CV phase (exponential taper)\n"
        rf"$\tau = {tau_s:g}$ s", transform=ax_e.transAxes,
        ha="right", va="top", fontsize=11, color="0.35",
    )
    # ax_e.set_title(
    #     rf"Charging model: $P_{{\mathrm{{cc}}}}={cc_power_w:g}$ W, "
    #     rf"$\mathrm{{SoC}}_{{\mathrm{{cv}}}}={100*cv_fraction:g}\%$, "
    #     rf"$\tau={tau_s:g}$ s"
    # )

    # power_at_transition_w = remaining_at_cv / tau_s * 3600.0
    # continuous = np.isclose(power_at_transition_w, cc_power_w, rtol=1e-3)
    #
    # ax_p.plot(t_min, power_w, color=POWER_COLOR, linewidth=2.0)
    # ax_p.axhline(cc_power_w, color="black", linewidth=0.7, linestyle="--", zorder=1)
    # if continuous:
    #     ax_p.text(
    #         t_cv_s / 60.0 + 0.04 * t_full_s / 60.0, cc_power_w * 1.12,
    #         "continuous at transition\n"
    #         r"($\tau$ derived from $(1-\mathrm{SoC}_{\mathrm{cv}}) \cdot C / P_{\mathrm{cc}}$)",
    #         ha="left", va="bottom", fontsize=9, color="0.25",
    #     )
    # else:
    #     ax_p.annotate(
    #         rf"$\times{power_at_transition_w / cc_power_w:.1f}$ jump at transition"
    #         "\n(CV-phase power is discontinuous here --\nnot an artifact, see module docstring)",
    #         xy=(t_cv_s / 60.0, power_at_transition_w),
    #         xytext=(t_cv_s / 60.0 + 0.06 * t_full_s / 60.0, power_at_transition_w * 0.92),
    #         ha="left", va="top", fontsize=9, color="0.25",
    #         arrowprops=dict(arrowstyle="-", color="0.5", linewidth=0.7),
    #     )
    #
    # ax_p.text(
    #     0.3, 0.825, rf" $P_{{\mathrm{{cc}}}}={cc_power_w:g}$ W", transform=ax_p.transAxes,
    #     ha="right", va="top", fontsize=12, color="0.35",
    # )
    # ax_p.annotate("continuous at transition\n"
    #             r"($\tau$ derived from $(1-\mathrm{SoC}_{\mathrm{cv}}) \frac{C}{P_{\mathrm{cc}}}$)", xy=(t_cv_s / 60.0, power_at_transition_w), xytext=(30,-40),
    #         textcoords='offset points', ha='left', va='center',
    #         #bbox=dict(boxstyle='round,pad=0.2', fc='gray', alpha=0.25),
    #         arrowprops=dict(arrowstyle='->', connectionstyle='arc3,rad=-0.25',color='black'),
    #         fontsize=11, color="0.35")
    # ax_p.set_ylabel("charging power [W]")
    # ax_p.set_xlabel("time [min]")
    # ax_p.set_xlim(0, t_full_s / 60.0)
    # ax_p.set_ylim(0, max(power_at_transition_w, cc_power_w) * (1.35 if continuous else 1.1))

    fig.tight_layout()
    fig.savefig(f"{OUT_STEM}.pdf", bbox_inches="tight")
    fig.savefig(f"{OUT_STEM}.png", dpi=200, bbox_inches="tight")
    print(
        f"wrote {OUT_STEM}.pdf / .png "
        f"(t_cv={t_cv_s/60:.1f} min, t_full={t_full_s/60:.1f} min)"
    )


if __name__ == "__main__":
    main()
