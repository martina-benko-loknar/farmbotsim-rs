import json
from pathlib import Path

# results/paper_results_v2/field/field{N}/grid_search_50x50_results.json all
# record field_config_path as the generic "configs/field_configs/default.json"
# -- the actual per-field geometry only lives in the escaped field_config_raw
# string. This pulls that string back out into standalone files, formatted
# the same way as configs/field_configs/vineyard/*.json.

SOURCE_DIR = Path("../results/paper_results_v2/field")
OUTPUT_DIR = Path("../configs/field_configs/legacy")
FIELDS = ["field1", "field2", "field3", "field4"]


def main():
    OUTPUT_DIR.mkdir(parents=True, exist_ok=True)

    for field in FIELDS:
        source_path = SOURCE_DIR / field / "grid_search_50x50_results.json"

        with open(source_path) as f:
            data = json.load(f)

        raw = data["field_config"]["field_config_raw"]
        json.loads(raw)  # validate before writing

        output_path = OUTPUT_DIR / f"{field}.json"
        output_path.write_text(raw)
        print(f"- {output_path}")


if __name__ == "__main__":
    main()
