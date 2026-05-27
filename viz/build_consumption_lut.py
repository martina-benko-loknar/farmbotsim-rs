import csv
import numpy as np
from scipy.interpolate import interp1d

# Configuration
INPUT_CSV = "configs/movement_configs/consumption/raw_measurements.csv"
OUTPUT_CSV = "configs/movement_configs/consumption/fitted_lut.csv"

START_ANGLE = -10
END_ANGLE = 10
STEP = 1

# Load raw measurements

data = {}

with open(INPUT_CSV, newline="") as csvfile:

    reader = csv.reader(csvfile)

    for row in reader:
        # Skip comments / empty lines
        if not row or row[0].startswith("#"):
            continue

        slope_deg = float(row[0].strip())
        speed_mps = float(row[1].strip())
        voltage_drop = float(row[2].strip())

        if speed_mps not in data:
            data[speed_mps] = {}

        data[speed_mps][slope_deg] = voltage_drop

# -------------------------------------------------
# Build interpolated LUT
# -------------------------------------------------

angles_deg = np.arange(
    START_ANGLE,
    END_ANGLE + STEP,
    STEP
)

rows = []

for speed, values in data.items():
    # Sort by slope
    sorted_items = sorted(values.items())

    slopes = np.array([x[0] for x in sorted_items])
    consumptions = np.array([x[1] for x in sorted_items])

    # Linear interpolation function
    interp_fn = interp1d(
        slopes,
        consumptions,
        kind="linear",
        fill_value="extrapolate"
    )

    # Interpolate all requested angles
    for angle_deg in angles_deg:
        interpolated = round(float(interp_fn(angle_deg)), 5)
        angle_rad = round(np.deg2rad(angle_deg), 6)
        rows.append([
            angle_rad,
            speed,
            interpolated
        ])

# -------------------------------------------------
# Save fitted LUT
# -------------------------------------------------

with open(OUTPUT_CSV, "w", newline="") as csvfile:
    writer = csv.writer(csvfile)
    writer.writerow([
        "# slope_rad",
        "speed_mps",
        "voltage_drop_per_m"
    ])
    for row in rows:
        writer.writerow(row)

print(f"Saved LUT to: {OUTPUT_CSV}")
print(f"Generated {len(rows)} LUT entries.")