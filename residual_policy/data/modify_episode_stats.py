import os
import json
import numpy as np
import pandas as pd
from tqdm import tqdm

# ----------------------------
# CONFIGURATION
# ----------------------------
DATASET_ROOT = "/home/qtf5422/.cache/huggingface/lerobot/ankile/dexmg-two-arm-coffee"

DATA_ROOT = os.path.join(DATASET_ROOT, "data")   # now points to /data
META_DIR = os.path.join(DATASET_ROOT, "meta")

INPUT_PATH = os.path.join(META_DIR, "episodes_stats.jsonl")
OUTPUT_PATH = os.path.join(META_DIR, "episodes_stats.jsonl")

NEW_FEATURES = [
    "regression",
]

# ----------------------------
# FIND ALL CHUNK DIRECTORIES
# ----------------------------
CHUNK_DIRS = sorted([
    os.path.join(DATA_ROOT, d)
    for d in os.listdir(DATA_ROOT)
    if d.startswith("chunk-")
])

print("Detected chunks:")
for c in CHUNK_DIRS:
    print("  ", c)


# ----------------------------
# HELPER
# ----------------------------
def compute_feature_stats(arr, count):
    arr = np.asarray(arr, dtype=np.float64)

    if arr.ndim == 1:
        arr = arr.reshape(-1, 1)

    return {
        "min": arr.min(axis=0).tolist(),
        "max": arr.max(axis=0).tolist(),
        "mean": arr.mean(axis=0).tolist(),
        "std": arr.std(axis=0, ddof=0).tolist(),
        "count": [int(count)],
    }


def find_parquet_file(episode_index):
    """
    Search for the episode parquet file across all chunk directories.
    """
    filename = f"episode_{episode_index:06d}.parquet"

    for chunk in CHUNK_DIRS:
        path = os.path.join(chunk, filename)
        if os.path.exists(path):
            return path

    return None


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

            parquet_path = find_parquet_file(episode_index)

            if parquet_path is None:
                print(f"⚠️  Parquet not found for episode {episode_index}")
                out_f.write(json.dumps(record) + "\n")
                continue

            try:
                df = pd.read_parquet(parquet_path)
            except Exception as e:
                print(f"⚠️  Error reading {parquet_path}: {e}")
                out_f.write(json.dumps(record) + "\n")
                continue

            num_timesteps = len(df)

            for feat in NEW_FEATURES:

                if feat not in df.columns:
                    print(f"⚠️  Feature '{feat}' missing in episode {episode_index}")
                    continue

                first_val = df[feat].iloc[0]

                if isinstance(first_val, (list, np.ndarray)):
                    arr = np.stack(df[feat].values, axis=0).astype(np.float64)
                else:
                    arr = df[feat].values.astype(np.float64)

                stats[feat] = compute_feature_stats(arr, num_timesteps)

            out_f.write(json.dumps(record) + "\n")

    print(f"\n✅ Updated stats written to {OUTPUT_PATH}")
    print("💡 Finalize with:")
    print(f"mv {OUTPUT_PATH} {INPUT_PATH}")


if __name__ == "__main__":
    main()