import json

def inspect_structure_json(obj, indent=0):
    prefix = "  " * indent

    if isinstance(obj, dict):
        print(f"{prefix}dict with keys:")
        for key, value in obj.items():
            print(f"{prefix}- {key}: {type(value).__name__}")
            inspect_structure_json(value, indent + 1)

    elif isinstance(obj, list):
        print(f"{prefix}list of length {len(obj)}")
        if len(obj) > 0:
            inspect_structure_json(obj[0], indent + 1)

    else:
        print(f"{prefix}{type(obj).__name__}")


# ---- load JSON file ----
with open("/home/qtf5422/Desktop/AIRE/residual-offpolicy-rl/arm=trossen_dual/task=entire_ecu_assembly/meta/info.json", "r") as f:
    data = json.load(f)

# ---- inspect structure ----
inspect_structure_json(data)

