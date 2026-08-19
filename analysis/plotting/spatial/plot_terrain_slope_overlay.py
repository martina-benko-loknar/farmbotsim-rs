"""
Prototype: vineyard aerial photo + translucent slope-tile overlay + XL-field row grid.

Data sources (all real, from the farmbotsim-rs repo):
  - Elevation LUT (240-cell regular lat/lon grid, empirical fit from N=3566 raw GNSS
    fixes across 3 bags): configs/scene_configs/vineyard_scene/baggy-altitude-empirical-lut.csv
  - Row geometry: configs/field_configs/vineyard/xlarge.json (the "XL" field used because its
    two row-blocks match the real vineyard's row count/spacing/gap most closely)
  - Basemap: Esri World Imagery XYZ tiles (public tile service, zoom 18 -- zoom 19 has
    "Map data not yet available" for this location, 18 has real usable resolution, ~0.6 m/px)

Coordinate frame: everything is plotted in local planar meters (x = east offset, y = north
offset from the GPS survey's SW corner), which is the *same* frame the Rust sim uses
(HeightMap.origin_x/origin_y, TerrainLoader::from_gps_csv) and the *same* frame the field
config's left_top_pos/row coordinates are defined in -- i.e. this is not a separate
georeferencing exercise, it reuses the sim's own coordinate system so the row grid lands
exactly where the sim thinks the rows are.

NOTE on "slope" here: this is a per-cell finite-difference gradient magnitude between
*neighboring grid cells* (a smoothed, field-wide picture), which is different from the
physics discharge model's slope (theta = atan(dh/dd) between the two points an agent
occupies at the start/end of one simulation step -- a local, per-step quantity). Said
explicitly in the caption so this figure isn't misread as "the slope the model sees".
"""

import csv
import io
import json
import math
import re
import subprocess

import matplotlib.pyplot as plt
import matplotlib.patheffects as patheffects
import numpy as np
from matplotlib.patches import Polygon
from matplotlib.collections import PatchCollection
from PIL import Image

REPO = "/home/martinabl/Projects/farmbotsim-rs"
CSV_PATH = f"{REPO}/configs/scene_configs/vineyard_scene/baggy-altitude-empirical-lut.csv"
FIELD_CONFIG_PATH = f"{REPO}/configs/field_configs/vineyard/xlarge.json"
OUT_STEM = "terrain_slope_overlay_prototype"

R_EARTH = 6_371_000.0

# ============================================================================
# 1. Load the elevation LUT, replicating TerrainLoader::from_gps_csv exactly
#    (src/terrain/loader.rs) so the grid this figure shows is the grid the
#    sim actually builds -- same origin, same cell size, same snapping.
# ============================================================================

def load_lut(path):
    """Mirrors read_csv in src/terrain/loader.rs: rows where lat/lon/alt don't all
    parse are silently dropped (Rust's `.ok()` + `if let (Some,Some,Some)`), which
    leaves that grid cell at HeightMap's zero-initialized default elevation (0 m)
    rather than any measured/fitted value. One such row exists in this file
    (empty `target` at lat=45.95340179, count=0) -- flagged in the printed summary
    below since it's a real, if minor, data gap in the sim's own terrain loader."""
    rows = []
    n_dropped = 0
    with open(path) as f:
        lines = [l for l in f if not l.startswith("#")]
    reader = csv.DictReader(lines)
    for r in reader:
        try:
            rows.append(dict(
                lat=float(r["x1_center"]),
                lon=float(r["x2_center"]),
                alt=float(r["target"]),
                count=int(r["count"]),
            ))
        except ValueError:
            n_dropped += 1
    if n_dropped:
        print(f"NOTE: {n_dropped} CSV row(s) had an unparseable field (e.g. empty "
              f"target) and were dropped, matching the Rust loader's behavior -- "
              f"that grid cell silently defaults to elevation 0 m in the sim.")
    return rows


def first_nonzero_diff(values_sorted):
    for a, b in zip(values_sorted, values_sorted[1:]):
        d = abs(b - a)
        if d > 1e-6:
            return d
    return 0.0


def build_height_map(rows):
    min_lat = min(r["lat"] for r in rows)
    min_lon = min(r["lon"] for r in rows)

    def lon_to_x(lon, lon0, lat0):
        return math.radians(lon - lon0) * R_EARTH * math.cos(math.radians(lat0))

    def lat_to_y(lat, lat0):
        return math.radians(lat - lat0) * R_EARTH

    xs = [lon_to_x(r["lon"], min_lon, min_lat) for r in rows]
    ys = [lat_to_y(r["lat"], min_lat) for r in rows]

    cell_w = first_nonzero_diff(sorted(xs))
    cell_h = first_nonzero_diff(sorted(ys))

    min_x, max_x = min(xs), max(xs)
    min_y, max_y = min(ys), max(ys)

    width = round((max_x - min_x) / cell_w) + 1
    height = round((max_y - min_y) / cell_h) + 1

    elev = np.zeros((height, width))
    count = np.zeros((height, width), dtype=int)

    for r, x, y in zip(rows, xs, ys):
        gx = round((x - min_x) / cell_w)
        gy = round((y - min_y) / cell_h)
        if 0 <= gx < width and 0 <= gy < height:
            elev[gy, gx] = r["alt"]
            count[gy, gx] = r["count"]

    return dict(
        width=width, height=height,
        cell_w=cell_w, cell_h=cell_h,
        elev=elev, count=count,
        origin_x=min_x, origin_y=min_y,
        anchor_lat=min_lat, anchor_lon=min_lon,
    )


def local_slope_deg(hm):
    """Per-cell field-wide gradient magnitude (finite differences), in degrees.
    NOT the sim's per-step slope -- see module docstring."""
    elev, w, h = hm["elev"], hm["width"], hm["height"]
    dx, dy = hm["cell_w"], hm["cell_h"]
    slope = np.zeros((h, w))
    for gy in range(h):
        for gx in range(w):
            if gx == 0:
                ddx = (elev[gy, gx + 1] - elev[gy, gx]) / dx
            elif gx == w - 1:
                ddx = (elev[gy, gx] - elev[gy, gx - 1]) / dx
            else:
                ddx = (elev[gy, gx + 1] - elev[gy, gx - 1]) / (2 * dx)
            if gy == 0:
                ddy = (elev[gy + 1, gx] - elev[gy, gx]) / dy
            elif gy == h - 1:
                ddy = (elev[gy, gx] - elev[gy - 1, gx]) / dy
            else:
                ddy = (elev[gy + 1, gx] - elev[gy - 1, gx]) / (2 * dy)
            slope[gy, gx] = math.degrees(math.atan(math.hypot(ddx, ddy)))
    return slope


# ============================================================================
# 2. Load XL field row geometry (configs/field_configs/vineyard/xlarge.json),
#    replicating the row-path construction in environment/field_config.rs
#    (VariantFieldConfig::Line path-building loop) exactly.
# ============================================================================

def parse_quantity(s):
    """'45.000 m' -> 45.0, '90.000 deg' -> 90.0"""
    return float(re.match(r"[-\d.]+", s.strip()).group())


def rotate(vx, vy, angle_deg):
    a = math.radians(angle_deg)
    c, s = math.cos(a), math.sin(a)
    return vx * c - vy * s, vx * s + vy * c


def load_row_segments(path):
    with open(path) as f:
        cfg = json.load(f)

    segments = []
    for entry in cfg["configs"]:
        line = entry["Line"]
        lx, ly = line["left_top_pos"]["x"], line["left_top_pos"]["y"]
        angle = parse_quantity(line["angle"])
        n_lines = line["n_lines"]
        length = parse_quantity(line["length"])
        spacing = parse_quantity(line["line_spacing"])

        for i in range(n_lines):
            dx1, dy1 = rotate(i * spacing, 0.0, angle)
            dx2, dy2 = rotate(i * spacing, length, angle)
            p1 = (lx + dx1, ly + dy1)
            p2 = (lx + dx2, ly + dy2)
            segments.append((p1, p2))
    return segments


# ============================================================================
# 3. Fetch an Esri World Imagery basemap covering the surveyed area (+padding)
#    and georeference it into the SAME local-meter frame as the height map.
#    (urllib can't resolve DNS in this sandbox; curl can -- shell out to it.)
# ============================================================================

def deg2num(lat_deg, lon_deg, zoom):
    lat_rad = math.radians(lat_deg)
    n = 2.0 ** zoom
    x = (lon_deg + 180.0) / 360.0 * n
    y = (1.0 - math.log(math.tan(lat_rad) + 1 / math.cos(lat_rad)) / math.pi) / 2.0 * n
    return x, y


def num2deg(xtile, ytile, zoom):
    n = 2.0 ** zoom
    lon_deg = xtile / n * 360.0 - 180.0
    lat_rad = math.atan(math.sinh(math.pi * (1 - 2 * ytile / n)))
    return math.degrees(lat_rad), lon_deg


def fetch_basemap(lat_min, lat_max, lon_min, lon_max, zoom=18, pad_frac=0.15):
    pad_lat = (lat_max - lat_min) * pad_frac
    pad_lon = (lon_max - lon_min) * pad_frac
    lat_min, lat_max = lat_min - pad_lat, lat_max + pad_lat
    lon_min, lon_max = lon_min - pad_lon, lon_max + pad_lon

    xf0, yf0 = deg2num(lat_max, lon_min, zoom)  # NW
    xf1, yf1 = deg2num(lat_min, lon_max, zoom)  # SE
    x0, x1 = int(math.floor(xf0)), int(math.floor(xf1))
    y0, y1 = int(math.floor(yf0)), int(math.floor(yf1))

    canvas = Image.new("RGB", ((x1 - x0 + 1) * 256, (y1 - y0 + 1) * 256))
    for xt in range(x0, x1 + 1):
        for yt in range(y0, y1 + 1):
            url = (f"https://server.arcgisonline.com/ArcGIS/rest/services/"
                   f"World_Imagery/MapServer/tile/{zoom}/{yt}/{xt}")
            out = subprocess.run(["curl", "-s", "-m", "10", url], capture_output=True)
            tile = Image.open(io.BytesIO(out.stdout)).convert("RGB")
            canvas.paste(tile, ((xt - x0) * 256, (yt - y0) * 256))

    lat_nw, lon_nw = num2deg(x0, y0, zoom)
    lat_se, lon_se = num2deg(x1 + 1, y1 + 1, zoom)
    # extent as (lon_min, lon_max, lat_min, lat_max)
    return canvas, (lon_nw, lon_se, lat_se, lat_nw)


def lonlat_extent_to_meters(extent, lat0, lon0):
    lon_min, lon_max, lat_min, lat_max = extent

    def to_xy(lon, lat):
        x = math.radians(lon - lon0) * R_EARTH * math.cos(math.radians(lat0))
        y = math.radians(lat - lat0) * R_EARTH
        return x, y

    x_min, y_min = to_xy(lon_min, lat_min)
    x_max, y_max = to_xy(lon_max, lat_max)
    return x_min, x_max, y_min, y_max


# ============================================================================
# 4. Build the figure
# ============================================================================

def main(mode="none"):
    """mode:
      "none"   -- every cell colored by slope, no measured/interpolated distinction
                  at all (max legibility of the slope pattern itself).
      "subtle" -- same, but interpolated/extrapolated cells (count==0) get a thin
                  dotted edge instead of the default hairline -- discoverable on
                  close inspection, not something that competes for attention.
    """
    rows = load_lut(CSV_PATH)
    hm = build_height_map(rows)
    slope = local_slope_deg(hm)
    row_segments = load_row_segments(FIELD_CONFIG_PATH)

    lat_min = hm["anchor_lat"]
    lon_min = hm["anchor_lon"]
    lat_max = max(r["lat"] for r in rows)
    lon_max = max(r["lon"] for r in rows)

    print(f"Elevation grid: {hm['width']} x {hm['height']} cells "
          f"({hm['cell_w']:.2f} m x {hm['cell_h']:.2f} m), "
          f"{int(hm['count'].sum() > 0)} ...")
    n_measured = int((hm["count"] > 0).sum())
    n_total = hm["width"] * hm["height"]
    print(f"Measured cells (count>0): {n_measured}/{n_total}")
    print(f"Field extent: {(hm['width']-1)*hm['cell_w']:.1f} m x "
          f"{(hm['height']-1)*hm['cell_h']:.1f} m")
    print(f"Row segments loaded: {len(row_segments)}")

    print("Fetching basemap tiles (Esri World Imagery, zoom 18)...")
    # Fetch generously (tile-quantized, so this padding is a floor, not exact) so the
    # cropped view below always has real image data at its edges; the actual on-screen
    # margin around the field is controlled separately by CROP_MARGIN_FRAC.
    basemap, extent_lonlat = fetch_basemap(lat_min, lat_max, lon_min, lon_max, zoom=18, pad_frac=0.25)
    extent_m = lonlat_extent_to_meters(extent_lonlat, lat0=lat_min, lon0=lon_min)
    print(f"Basemap image: {basemap.size}, extent (m): "
          f"x=[{extent_m[0]:.1f},{extent_m[1]:.1f}] y=[{extent_m[2]:.1f},{extent_m[3]:.1f}]")

    # Field/terrain bounding box (exact, in meters) -- the crop and the extras below
    # are all sized off *this*, not off the tile-quantized basemap extent.
    fx0, fy0 = hm["origin_x"], hm["origin_y"]
    fx1 = hm["origin_x"] + (hm["width"] - 1) * hm["cell_w"]
    fy1 = hm["origin_y"] + (hm["height"] - 1) * hm["cell_h"]
    CROP_MARGIN_FRAC = 0.12
    # Top and bottom get the same (small) margin: neither holds an in-map decoration
    # any more (scale bar dropped, north arrow moved into the colorbar's margin
    # column -- see below), so both just need a little breathing room around the
    # grid extent, not the full left/right crop margin.
    VERT_MARGIN_FRAC = 0.045
    mx_pad = CROP_MARGIN_FRAC * (fx1 - fx0)
    my_pad = VERT_MARGIN_FRAC * (fy1 - fy0)
    xlim = (fx0 - mx_pad, fx1 + mx_pad)
    ylim = (fy0 - my_pad, fy1 + my_pad)

    # -- style, loosely matching analysis/plotting/viz_utils.py house style --
    plt.rcParams["text.usetex"] = True
    plt.rcParams["font.family"] = "serif"
    plt.rcParams["font.serif"] = ["Computer Modern Roman"]
    base_fs = 15
    plt.rcParams["font.size"] = base_fs
    plt.rcParams["axes.labelsize"] = base_fs
    plt.rcParams["xtick.labelsize"] = base_fs - 2
    plt.rcParams["ytick.labelsize"] = base_fs - 2
    plt.rcParams["legend.fontsize"] = base_fs - 2

    fig_w = 7.0
    fig_h = fig_w * (ylim[1] - ylim[0]) / (xlim[1] - xlim[0])
    fig, ax = plt.subplots(figsize=(fig_w, fig_h))

    ax.imshow(np.asarray(basemap), extent=extent_m, origin="upper", zorder=0)

    # -- translucent slope tiles at the sim's actual grid resolution --
    # Every cell (measured or interpolated/extrapolated) already carries a real
    # elevation value from the LUT's empirical fit, so every cell gets colored by
    # slope the same way -- the measured/interpolated split is not a coloring
    # decision, it's at most a subtle edge-style cue (mode="subtle") so the slope
    # pattern itself is what the reader's eye lands on.
    cmap = plt.get_cmap("inferno")
    vmin, vmax = 0.0, np.percentile(slope, 98)  # robust to a couple of extreme edge cells
    norm = plt.Normalize(vmin=vmin, vmax=vmax)

    patches, colors, edge_styles = [], [], []

    for gy in range(hm["height"]):
        for gx in range(hm["width"]):
            x0 = hm["origin_x"] + (gx - 0.5) * hm["cell_w"]
            y0 = hm["origin_y"] + (gy - 0.5) * hm["cell_h"]
            poly = Polygon([
                (x0, y0), (x0 + hm["cell_w"], y0),
                (x0 + hm["cell_w"], y0 + hm["cell_h"]), (x0, y0 + hm["cell_h"]),
            ])
            patches.append(poly)
            colors.append(cmap(norm(slope[gy, gx])))
            edge_styles.append(hm["count"][gy, gx] > 0)

    if mode == "hatched":
        # original design: unmeasured cells get NO slope fill at all, just a
        # hatch pattern, so the measured/interpolated split is the dominant visual
        # cue rather than a subtle one -- kept around as the default for now.
        measured_patches = [p for p, m in zip(patches, edge_styles) if m]
        measured_colors = [c for c, m in zip(colors, edge_styles) if m]
        unmeasured_patches = [p for p, m in zip(patches, edge_styles) if not m]

        pc = PatchCollection(measured_patches, facecolor=measured_colors,
                              edgecolor="white", linewidth=0.15, alpha=0.4, zorder=2)
        ax.add_collection(pc)

        pc_un = PatchCollection(unmeasured_patches, facecolor="none",
                                 edgecolor="white", linewidth=0.5, hatch="////",
                                 alpha=0.5, zorder=2)
        ax.add_collection(pc_un)

        # measured-cell centers (documents real GNSS sample density)
        mx, my = [], []
        for gy in range(hm["height"]):
            for gx in range(hm["width"]):
                if hm["count"][gy, gx] > 0:
                    mx.append(hm["origin_x"] + gx * hm["cell_w"])
                    my.append(hm["origin_y"] + gy * hm["cell_h"])
        ax.scatter(mx, my, s=3, c="white", edgecolors="black", linewidths=0.25,
                   alpha=0.8, zorder=3, label="GNSS-sampled\ncells")
    elif mode == "subtle":
        # thin dotted edge on interpolated/extrapolated cells only; measured cells
        # get the same near-invisible hairline as before
        edgecolors = ["white" if m else "white" for m in edge_styles]
        linewidths = [0.15 if m else 0.45 for m in edge_styles]
        linestyles = ["solid" if m else "dotted" for m in edge_styles]
        pc = PatchCollection(patches, facecolor=colors, edgecolor=edgecolors,
                              linewidth=linewidths, linestyle=linestyles,
                              alpha=0.55, zorder=2)
        ax.add_collection(pc)
    else:
        pc = PatchCollection(patches, facecolor=colors, edgecolor="white",
                              linewidth=0.15, alpha=0.55, zorder=2)
        ax.add_collection(pc)

    # -- XL field row geometry --
    row_color = "#00E5FF"
    row_fx = [patheffects.withStroke(linewidth=1.6, foreground="black")]
    for k, (p1, p2) in enumerate(row_segments):
        ax.plot([p1[0], p2[0]], [p1[1], p2[1]], color=row_color, linewidth=0.9,
                alpha=0.95, zorder=4, path_effects=row_fx,
                label="vineyard\nrows" if k == 0 else None)

    # -- field/terrain bounding box --
    ax.plot([fx0, fx1, fx1, fx0, fx0], [fy0, fy0, fy1, fy1, fy0],
            "k-", linewidth=1.3, zorder=4, label="terrain grid\nextent")

    # No in-map scale bar: the grid cell size (printed above, "Elevation grid: ...")
    # already fixes the scale via the visible tile grid itself, and belongs in the
    # figure caption/text rather than as an on-image label.

    ax.set_xlim(*xlim)
    ax.set_ylim(*ylim)
    ax.set_aspect("equal")
    ax.set_xlabel("$x$ (m)")
    ax.set_ylabel("$y$ (m)")

    # Colorbar top-anchored and shrunk so the entries legend can sit directly
    # below it in the same right-hand margin, instead of overlapping the photo.
    # Built from ax's *actual* rendered box (post aspect="equal" adjustment) via
    # an explicit fig.add_axes rather than fig.colorbar(..., ax=ax, shrink=...),
    # which sizes/anchors against ax's pre-adjustment box and made the colorbar
    # overshoot above the map. A forced draw finalizes that adjustment first.
    fig.canvas.draw()
    ax_pos = ax.get_position()
    cbar_frac, cbar_pad, cbar_shrink = 0.046, 0.04, 0.55
    # The compass symbol sits flush to the top of the margin column (level with
    # the map's top edge), then margin_gap separates compass<->colorbar and,
    # further down, colorbar<->box-legend by the same amount, so the vertical
    # rhythm down this right-hand margin column is even.
    icon_n_len = 0.045 * ax_pos.height   # compass tip length, north half
    icon_s_len = 0.026 * ax_pos.height   # compass tip length, south half
    compass_h = icon_n_len + icon_s_len  # total height the compass icon occupies
    margin_gap = 0.045 * ax_pos.height
    cbar_w = cbar_frac * ax_pos.width
    cbar_h = cbar_shrink * ax_pos.height
    cbar_x0 = ax_pos.x1 + cbar_pad * ax_pos.width
    cbar_y0 = ax_pos.y1 - compass_h - margin_gap - cbar_h
    cax = fig.add_axes([cbar_x0, cbar_y0, cbar_w, cbar_h])
    cbar = fig.colorbar(plt.cm.ScalarMappable(norm=norm, cmap=cmap), cax=cax)
    cbar.set_label(r"local terrain slope $|\nabla h|$ (deg)")

    # -- compass symbol -- placed above the colorbar with a margin_gap gap between
    # them, in figure coordinates (not over the photo, so no path-effect stroke is
    # needed here). A plain two-triangle compass-rose needle instead of an
    # arrow+"N" label: dark north-pointing half, light south-pointing half,
    # meeting at a shared base -- the elongated tip reads as north under the
    # standard north-up map convention, with no text needed.
    cx_fig = cbar_x0 + cbar_w / 2
    cy_fig = ax_pos.y1 - icon_n_len
    half_w = 0.011
    compass_kwargs = dict(edgecolor="black", linewidth=0.6, closed=True,
                           zorder=6, transform=fig.transFigure)
    fig.add_artist(Polygon([(cx_fig, cy_fig + icon_n_len), (cx_fig + half_w, cy_fig),
                             (cx_fig - half_w, cy_fig)],
                            facecolor="black", **compass_kwargs))
    fig.add_artist(Polygon([(cx_fig, cy_fig - icon_s_len), (cx_fig + half_w, cy_fig),
                             (cx_fig - half_w, cy_fig)],
                            facecolor="white", **compass_kwargs))

    legend_y_axes_frac = (cbar_y0 - margin_gap - ax_pos.y0) / ax_pos.height
    # handleheight=2 makes each entry's handle box as tall as the two-line labels
    # below, so the marker/line/patch sits centered against both lines rather
    # than pinned to the first one; multialignment left-aligns "cells"/"rows"/
    # "extent" under the start of their label's first line.
    leg = ax.legend(loc="upper left", bbox_to_anchor=(1.02, legend_y_axes_frac),
                     borderaxespad=0, framealpha=0.85, fontsize=base_fs - 4,
                     handleheight=2, labelspacing=0.8)
    for text in leg.get_texts():
        text.set_multialignment("left")

    # Title commented out for now -- re-enable by uncommenting this block.
    # subtitle = r"Esri World Imagery basemap $\cdot$ slope tiles at native GNSS-survey resolution"
    # if mode == "hatched":
    #     subtitle += r" $\cdot$ hatched cells = no direct GNSS sample"
    # elif mode == "subtle":
    #     subtitle += r" $\cdot$ {\footnotesize dotted edge = interpolated/extrapolated cell}"
    # ax.set_title(
    #     r"\textbf{Vineyard terrain slope over XL-field row layout}" + "\n"
    #     r"{\small " + subtitle + "}",
    #     fontsize=base_fs, pad=10,
    # )

    # No fig.tight_layout() here: it fights the manually-positioned colorbar axes
    # (added via fig.add_axes, not fig.colorbar(ax=...)) and squeezed/clipped the
    # legend. savefig(bbox_inches="tight") below already crops the final figure.
    out_stem = f"{OUT_STEM}_{mode}"
    fig.savefig(f"{out_stem}.pdf", bbox_inches="tight")
    fig.savefig(f"{out_stem}.png", dpi=200, bbox_inches="tight")
    print(f"Saved {out_stem}.pdf / .png")


if __name__ == "__main__":
    # "hatched" is the current default (kept for now); "none" and "subtle" are the
    # de-emphasized-distinction variants explored alongside it -- swap/extend this
    # tuple to regenerate any of them.
    for mode in ("hatched",):
        main(mode)
