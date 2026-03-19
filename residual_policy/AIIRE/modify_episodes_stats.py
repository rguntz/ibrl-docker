import os
import json
import numpy as np
import pandas as pd
from tqdm import tqdm

# ----------------------------
# CONFIGURATION
# ----------------------------
DATASET_ROOT = "/home/qtf5422/Desktop/AIRE/residual-offpolicy-rl/arm=trossen_dual/task=entire_ecu_assembly"
DATA_DIR = os.path.join(DATASET_ROOT, "data", "chunk-000")
META_DIR = os.path.join(DATASET_ROOT, "meta")

INPUT_PATH = os.path.join(META_DIR, "episodes_stats.jsonl")
OUTPUT_PATH = os.path.join(META_DIR, "episodes_stats.jsonl") # Changed name to avoid overwriting during run

# The feature to add
# NEW_FEATURES = [
#     "next.done"
# ]

NEW_FEATURES = [
    "state", 
    "action"
]

# ----------------------------
# HELPER
# ----------------------------

def compute_feature_stats(arr, count):
    """
    Compute min, max, mean, std, count for an array.
    Ensures output values are always lists to maintain consistency 
    between vector features (shape N, D) and scalar features (shape N,).
    """
    # Ensure arr is numpy array
    arr = np.asarray(arr, dtype=np.float64)
    
    # If 1D (scalar per timestep), reshape to (N, 1) so axis=0 operations return (1,) arrays
    if arr.ndim == 1:
        arr = arr.reshape(-1, 1)
    
    # Calculate stats along axis=0 (timesteps)
    # Result shapes will be (D,) where D is feature dimension (1 for scalar, 7 for action, etc.)
    min_val = arr.min(axis=0)
    max_val = arr.max(axis=0)
    mean_val = arr.mean(axis=0)
    std_val = arr.std(axis=0, ddof=0)
    
    return {
        "min": min_val.tolist(),
        "max": max_val.tolist(),
        "mean": mean_val.tolist(),
        "std": std_val.tolist(),
        "count": [int(count)], # Ensure count is also a list for consistency
    }

# ----------------------------
# MAIN
# ----------------------------

def main():
    if not os.path.exists(INPUT_PATH):
        print(f"❌ Input file not found: {INPUT_PATH}")
        return

    with open(INPUT_PATH, "r") as in_f:
        lines = in_f.readlines()

    print(f"Processing {len(lines)} episodes...")

    with open(OUTPUT_PATH, "w") as out_f:
        for line in tqdm(lines, desc="Updating stats"):
            record = json.loads(line)
            episode_index = record["episode_index"]
            stats = record["stats"]

            # Load corresponding Parquet file
            parquet_path = os.path.join(DATA_DIR, f"episode_{episode_index:06d}.parquet")
            
            if not os.path.exists(parquet_path):
                print(f"⚠️  Missing Parquet for episode {episode_index}, skipping.")
                out_f.write(json.dumps(record) + "\n")
                continue

            try:
                df = pd.read_parquet(parquet_path)
            except Exception as e:
                print(f"⚠️  Error reading Parquet for episode {episode_index}: {e}")
                out_f.write(json.dumps(record) + "\n")
                continue

            num_timesteps = len(df)

            # Add stats for each new feature
            for feat in NEW_FEATURES:
                if feat not in df.columns:
                    print(f"⚠️  Feature '{feat}' missing in episode {episode_index}")
                    continue
                
                # Handle data extraction
                # If the column contains lists/arrays, stack them. 
                # If the column contains scalars, converting to numpy array directly works.
                first_val = df[feat].iloc[0]
                
                if isinstance(first_val, (list, np.ndarray)):
                    # Case: Vector feature (e.g., action, state)
                    arr = np.stack(df[feat].values, axis=0).astype(np.float64)
                else:
                    # Case: Scalar feature (e.g., next.done, reward)
                    arr = df[feat].values.astype(np.float64)
                
                stats[feat] = compute_feature_stats(arr, num_timesteps)

            # Write updated record
            out_f.write(json.dumps(record) + "\n")

    print(f"✅ Updated stats written to {OUTPUT_PATH}")
    print("💡 To finalize, run:")
    print(f"  mv {OUTPUT_PATH} {INPUT_PATH}")

if __name__ == "__main__":
    main()