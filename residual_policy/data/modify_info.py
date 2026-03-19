import json
import os
import numpy as np
import pandas as pd

# ----------------------------
# CONFIGURATION
# ----------------------------
DATASET_ROOT = "/home/qtf5422/.cache/huggingface/lerobot/ankile/dexmg-two-arm-coffee"

META_DIR = os.path.join(DATASET_ROOT, "meta")
DATA_DIR = os.path.join(DATASET_ROOT, "data")

INFO_PATH = os.path.join(META_DIR, "info.json")

NEW_FEATURE = "regression"

# ----------------------------
# HELPERS
# ----------------------------

def find_example_parquet():
    """
    Find one parquet file to inspect the schema.
    """
    for root, _, files in os.walk(DATA_DIR):
        for f in files:
            if f.endswith(".parquet"):
                return os.path.join(root, f)
    return None


def infer_feature_schema(df, feature_name):
    """
    Infer dtype, shape and names from a dataframe column.
    """
    sample = df[feature_name].iloc[0]

    # detect vector feature
    if isinstance(sample, (list, np.ndarray)):
        arr = np.array(sample)

        shape = [int(arr.shape[0])]
        names = [f"{feature_name}_{i}" for i in range(arr.shape[0])]

        dtype = str(arr.dtype)

    else:
        shape = [1]
        names = None
        dtype = str(np.array(sample).dtype)

    # normalize dtype (similar to other features)
    if "float" in dtype:
        dtype = "float32"
    elif "int" in dtype:
        dtype = "int64"
    elif "bool" in dtype:
        dtype = "bool"

    return {
        "dtype": dtype,
        "shape": shape,
        "names": names
    }


# ----------------------------
# MAIN
# ----------------------------

def main():

    if not os.path.exists(INFO_PATH):
        print("❌ info.json not found")
        return

    parquet_file = find_example_parquet()

    if parquet_file is None:
        print("❌ No parquet file found")
        return

    print("Using example parquet:", parquet_file)

    df = pd.read_parquet(parquet_file)

    if NEW_FEATURE not in df.columns:
        print(f"❌ Feature '{NEW_FEATURE}' not found in parquet files")
        return

    schema = infer_feature_schema(df, NEW_FEATURE)

    with open(INFO_PATH, "r") as f:
        info = json.load(f)

    if NEW_FEATURE in info["features"]:
        print(f"⚠️ Feature '{NEW_FEATURE}' already exists")
        return

    info["features"][NEW_FEATURE] = schema

    with open(INFO_PATH, "w") as f:
        json.dump(info, f, indent=4)

    print("✅ Feature added to info.json")
    print(json.dumps(schema, indent=4))


if __name__ == "__main__":
    main()