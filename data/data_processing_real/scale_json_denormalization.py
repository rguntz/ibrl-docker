import json
import os
import argparse

EXCLUDE_KEYS = {"Grip"}

def scale_json_file(input_path, target_value):
    # Load JSON
    with open(input_path, "r") as f:
        data = json.load(f)

    scaled_data = {}

    for key, values in data.items():
        # Keep excluded keys unchanged
        if key in EXCLUDE_KEYS:
            scaled_data[key] = values
            continue

        min_val = values["min"]
        max_val = values["max"]

        max_abs = max(abs(min_val), abs(max_val))

        # Avoid division by zero
        if max_abs == 0:
            scaled_data[key] = values
            continue

        scale_factor = target_value / max_abs

        scaled_data[key] = {
            "min": min_val * scale_factor,
            "max": max_val * scale_factor
        }

    # Create new filename
    base, ext = os.path.splitext(input_path)
    output_path = f"{base}_scaled_bc{ext}"

    # Save scaled JSON
    with open(output_path, "w") as f:
        json.dump(scaled_data, f, indent=4)

    print(f"Scaled file saved to: {output_path}")


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("json_file", help="Path to input JSON file")
    parser.add_argument("target", type=float, help="Target absolute max value")

    args = parser.parse_args()

    scale_json_file(args.json_file, args.target)
