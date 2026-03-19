import json

# Load the data
data = []
with open(
    "/home/qtf5422/Desktop/AIRE/residual-offpolicy-rl/arm=trossen_dual/task=entire_ecu_assembly/meta/episodes_stats.jsonl",
    "r"
) as f:
    for line in f:
        data.append(json.loads(line))

# Define the video keys to rename
video_keys = {"wrist_video", "video", "wrist_video_2", "video_2"}

# Transform each episode entry
for entry in data:
    stats = entry["stats"]

    # 1. Rename "state" → "observation.state"
    if "state" in stats:
        stats["observation.state"] = stats.pop("state")

    # 2. Rename video keys with "observation.images." prefix
    keys_to_rename = [k for k in stats if k in video_keys]
    for k in keys_to_rename:
        new_key = f"observation.images.{k}"
        stats[new_key] = stats.pop(k)

# Optional: Write back to a new file
output_path = "/home/qtf5422/Desktop/AIRE/residual-offpolicy-rl/arm=trossen_dual/task=entire_ecu_assembly/meta/episodes_stats.jsonl"
with open(output_path, "w", encoding="utf-8") as f:
    for entry in data:
        f.write(json.dumps(entry) + "\n")

print(f"Renamed keys and saved to {output_path}")