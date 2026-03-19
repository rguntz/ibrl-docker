import os
import json
import glob
import pyarrow.parquet as pq
import numpy as np

def dtype_to_json_dtype(dtype):
    """Map numpy/pandas dtype to JSON-compatible dtype string."""
    if np.issubdtype(dtype, np.floating):
        if dtype == np.float64:
            return "float64"
        elif dtype == np.float32:
            return "float32"
        else:
            return "float32"  # fallback
    elif np.issubdtype(dtype, np.integer):
        if dtype == np.int64:
            return "int64"
        elif dtype == np.int32:
            return "int32"
        else:
            return "int64"
    else:
        return str(dtype)

def infer_shape_from_series(sample_value):
    """Infer shape of a scalar or array-like value."""
    if hasattr(sample_value, '__len__') and not isinstance(sample_value, str):
        # Assume it's a list or np array
        arr = np.array(sample_value)
        return arr.shape
    else:
        return (1,)

def main(dataset_root):
    info_path = os.path.join(dataset_root, "meta", "info.json")
    
    if not os.path.exists(info_path):
        raise FileNotFoundError(f"info.json not found at {info_path}")
    
    with open(info_path, 'r') as f:
        info = json.load(f)

    # Get all parquet files
    parquet_pattern = os.path.join(dataset_root, "data", "chunk-*", "episode_*.parquet")
    parquet_files = glob.glob(parquet_pattern)
    
    if not parquet_files:
        raise ValueError("No Parquet files found!")

    # Collect all unique column names (excluding 'arm', 'task')
    all_columns = set()
    sample_data = {}
    
    for pf in parquet_files[:5]:  # Sample first few files for efficiency
        table = pq.read_table(pf)
        df = table.to_pandas()
        cols = [c for c in df.columns if c not in {'arm', 'task'}]
        all_columns.update(cols)
        # Store sample values for shape inference
        for col in cols:
            if col not in sample_data and not df[col].isna().all():
                sample_val = df[col].dropna().iloc[0]
                sample_data[col] = sample_val

    # Existing feature keys in info.json
    existing_features = set(info.get("features", {}).keys())

    # Columns to add: those not already in features
    columns_to_add = all_columns - existing_features

    # Add new features
    for col in sorted(columns_to_add):
        if col in {'arm', 'task'}:
            continue  # safety skip

        sample_val = sample_data.get(col)
        if sample_val is None:
            print(f"⚠️ Skipping column '{col}' (no valid sample found)")
            continue

        # Infer dtype
        if isinstance(sample_val, (list, np.ndarray)):
            dtype = dtype_to_json_dtype(np.asarray(sample_val).dtype)
            shape = infer_shape_from_series(sample_val)
        elif isinstance(sample_val, (int, np.integer)):
            dtype = "int64"
            shape = (1,)
        elif isinstance(sample_val, (float, np.floating)):
            dtype = "float64"
            shape = (1,)
        else:
            # Fallback: treat as float64 scalar
            dtype = "float64"
            shape = (1,)

        # Decide prefix: action vs observation
        if col.startswith("action") or col in {"actions", "actions_2"}:
            key = f"action.{col}" if not col.startswith("action.") else col
        else:
            key = f"observation.{col}"

        # Avoid overwriting
        if key in info["features"]:
            continue

        info["features"][key] = {
            "dtype": dtype,
            "shape": list(shape),
            "names": ["state"] if len(shape) == 1 else None
        }

        print(f"✅ Added feature: {key} | dtype={dtype}, shape={shape}")

    # Write back updated info.json
    with open(info_path, 'w') as f:
        json.dump(info, f, indent=2)

    print(f"\n✨ Updated {info_path} with {len(columns_to_add)} new features.")

if __name__ == "__main__":
    import argparse
    parser = argparse.ArgumentParser(description="Update LeRobot info.json with Parquet column schema.")
    parser.add_argument("dataset_root", help="Path to the dataset root directory (containing meta/ and data/)")
    args = parser.parse_args()

    main(args.dataset_root)