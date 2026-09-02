"""
Prototype: physics discharge model figure for the "Discharge model" paragraph
of the paper's Experimental Setup subsection
(../agro-charging-framework/revisions/AE_2026_R1/main.tex, sec:experimental-setup).

Data source (real, not re-fit here): configs/movement_configs/consumption/slope_consumption.json,
loaded directly so the plotted curves cannot drift from what the sim actually evaluates
(src/battery_module/discharging/slope_consumption.rs::SlopeConsumptionModel::voltage_drop_per_m).

Single panel with a twin y-axis (left: V/m, right: Wh/m), rather than two stacked panels.
This is safe (not just visually convenient) because the Wh/m curve is a *pure scalar multiple*
of the V/m curve -- same wh_scale for travel and work, no offset -- so the two axes show the
exact same lines, just relabeled:
    dV/m = c0 + c1 * theta_deg   for theta >= 0 (uphill)
    dV/m = c0                    for theta <  0 (downhill, flat)
    Wh/m = dV/m * wh_scale
where wh_scale is the same units-conversion constant used by the sim
(src/battery_module/discharging/physics_model.rs::PhysicsDischargeModel::calibrate_wh_scale):
    wh_scale = LEO_ROVER_NOMINAL_DRIVING_POWER_W / (travel.intercept_v_per_m * travel.reference_speed_mps * 3600)
Only the V/m curves are actually plotted; the right axis's ylim is set to left_ylim * wh_scale
so it reads off the same lines in the other unit.

NOT plotted here (out of scope for this prototype, flag if wanted): the wheel-slip
correction v_robot = v_wheel * (1 - s) (src/terrain/slip.rs) -- a separate correction
applied to distance travelled, not to this voltage-drop curve.
"""

import json

import matplotlib.pyplot as plt
import numpy as np

REPO = "/home/martinabl/Projects/farmbotsim-rs"
CONFIG_PATH = f"{REPO}/configs/movement_configs/consumption/slope_consumption.json"

# src/cfg.rs::LEO_ROVER_NOMINAL_DRIVING_POWER_W -- 73.2 Wh pack / 4h nominal driving = 18.3 W.
NOMINAL_DRIVING_POWER_W = 18.3
SECONDS_PER_HOUR = 3600.0

OUT_STEM = "discharge_model"

TRAVEL_COLOR = "#0072B2"  # Okabe-Ito blue, matches analysis/plotting/convergence.py::GRID_COLOR
WORK_COLOR = "black"      # was Okabe-Ito vermillion ("#D55E00") -- switched to black so the
                          # two curves follow the paper's usual black/blue two-series
                          # convention instead of introducing a third, one-off hue (2026-09-02).
# Only the uphill region is shaded (the more "active" regime, where cost actually
# increases with slope); downhill (flat) stays white. Gray matches the transition-band
# shading in analysis/plotting/sensitivity.py's REFERENCE_SPAN_STYLES["transition"]
# (battery_sensitivity_energy_mechanism.pdf's charging-events step bands), for the same
# "gray = a highlighted region" convention across the paper (2026-09-02).
UPHILL_SPAN_COLOR = "#999999"
UPHILL_SPAN_ALPHA = 0.18


def voltage_drop_per_m(model, slope_deg):
    slope_deg = np.clip(slope_deg, -model["max_abs_slope_deg"], model["max_abs_slope_deg"])
    uphill = model["intercept_v_per_m"] + model["slope_coeff"] * slope_deg
    return np.where(slope_deg >= 0.0, uphill, model["intercept_v_per_m"])


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
    travel, work = cfg["travel"], cfg["work"]

    wh_scale = NOMINAL_DRIVING_POWER_W / (
        travel["intercept_v_per_m"] * travel["reference_speed_mps"] * SECONDS_PER_HOUR
    )

    max_slope = max(travel["max_abs_slope_deg"], work["max_abs_slope_deg"])
    slope_deg = np.linspace(-max_slope, max_slope, 400)

    setup_style()
    fig, ax_v = plt.subplots(figsize=(6.5, 3.46))
    ax_e = ax_v.twinx()

    ax_v.axvspan(0.0, max_slope, color=UPHILL_SPAN_COLOR, alpha=UPHILL_SPAN_ALPHA, linewidth=0, zorder=0)
    ax_v.axvline(0.0, color="black", linewidth=0.8, linestyle=":", zorder=1)
    ax_v.set_xlim(-max_slope, max_slope)
    ax_v.grid(alpha=0.25, linewidth=0.5)
    ax_e.grid(False)

    for model, color, label in [(travel, TRAVEL_COLOR, "travel"), (work, WORK_COLOR, "work")]:
        v_drop = voltage_drop_per_m(model, slope_deg)
        ax_v.plot(
            slope_deg, v_drop, color=color, linewidth=2.0,
            label=rf"{label} ($v_\mathrm{{ref}}={model['reference_speed_mps']:g}$ m/s)",
        )

    # Right axis reads off the same lines, just rescaled by the units-conversion constant --
    # no separate Wh/m curves are plotted.
    ax_e.set_ylim(np.array(ax_v.get_ylim()) * wh_scale)

    ax_v.set_ylabel(r"voltage-drop curve [V/m]")
    ax_e.set_ylabel("calibrated energy cost [Wh/m]")
    ax_v.set_xlabel(r"terrain slope $\theta [^{\circ}]$")
    ax_v.legend(loc="upper left", frameon=False)

    ax_v.text(
        0.15, 0.06, "downhill: flat", transform=ax_v.transAxes,
        ha="left", va="bottom", fontsize=12, color="0.35",
    )
    ax_v.text(
        0.8, 0.06, "uphill: linear", transform=ax_v.transAxes,
        ha="right", va="bottom", fontsize=12, color="0.35",
    )

    # Fitted-line equations dropped from the plot itself -- the same numbers are already
    # given in the surrounding text, so annotating them here too was redundant (2026-09-02).
    # ax_v.text(
    #     0.55, 0.93, r"$y = 0.0121 \frac{V}{m} + 0.0003 \frac{V}{m ^{\circ}} \cdot \theta$", transform=ax_v.transAxes,
    #     ha="left", va="top", fontsize=11, color=TRAVEL_COLOR,
    # )
    # ax_v.text(
    #     0.55, 0.76, r"$y = 0.0097 \frac{V}{m} + 0.0002 \frac{V}{m ^{\circ}} \cdot \theta$", transform=ax_v.transAxes,
    #     ha="left", va="top", fontsize=11, color=WORK_COLOR,
    # )

    fig.tight_layout()
    fig.savefig(f"{OUT_STEM}.pdf", bbox_inches="tight")
    fig.savefig(f"{OUT_STEM}.png", dpi=200, bbox_inches="tight")
    print(f"wrote {OUT_STEM}.pdf / .png (wh_scale={wh_scale:.5f} Wh per V*m)")


if __name__ == "__main__":
    main()
