import json

def inspect_structure_jsonl(obj, indent=0):
    prefix = "  " * indent
    if isinstance(obj, dict):
        print(f"{prefix}dict with keys:")
        for k, v in obj.items():
            print(f"{prefix}- {k}: {type(v).__name__}")
            inspect_structure_jsonl(v, indent + 1)
    elif isinstance(obj, list):
        print(f"{prefix}list of length {len(obj)}")
        if obj:
            inspect_structure_jsonl(obj[0], indent + 1)
    else:
        print(f"{prefix}{type(obj).__name__}")

data = []
with open(
    "/home/qtf5422/Desktop/AIRE/residual-offpolicy-rl/arm=trossen_dual/task=entire_ecu_assembly/meta/episodes_stats.jsonl",
    "r"
) as f:
    for line in f:
        data.append(json.loads(line))

# ---- inspect structure ----
inspect_structure_jsonl(data)