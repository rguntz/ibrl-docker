import json

def rename_keys_in_features(features_dict):
    """Rename specific keys inside the 'features' dict."""
    renamed = {}
    video_keys = {"wrist_video", "video", "wrist_video_2", "video_2"}
    
    for key, value in features_dict.items():
        if key == "state":
            new_key = "observation.state"
        elif key in video_keys:
            new_key = f"observation.images.{key}"
        else:
            new_key = key
        renamed[new_key] = value
    return renamed





# ---- load JSON file ----
file_path = "/home/qtf5422/Desktop/AIRE/residual-offpolicy-rl/arm=trossen_dual/task=entire_ecu_assembly/meta/info.json"
with open(file_path, "r") as f:
    data = json.load(f)

# ---- modify the 'features' section ----
if "features" in data and isinstance(data["features"], dict):
    data["features"] = rename_keys_in_features(data["features"])

with open(file_path, "w") as f:
    json.dump(data, f, indent=2)

print(f"Successfully updated and overwritten: {file_path}")
