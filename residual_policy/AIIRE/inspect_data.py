import os
import pandas as pd
import pyarrow.parquet as pq
import numpy as np
from collections import Counter

def min_max_gripper() : 
        
    # --------------------------------------------------
    # GLOBAL MIN/MAX OF 7TH JOINT ACROSS ALL FILES
    # --------------------------------------------------
    import glob

    print("\n" + "=" * 80)
    print("GLOBAL MIN/MAX OF 7TH JOINT (INDEX 6) ACROSS ALL FILES")
    print("=" * 80)

    # Get all .parquet files in the same chunk directory
    chunk_dir = os.path.dirname(PARQUET_PATH)
    parquet_files = sorted(glob.glob(os.path.join(chunk_dir, "*.parquet")))

    print(f"Found {len(parquet_files)} Parquet files in {chunk_dir}")

    all_7th_joint_values = []

    for i, fp in enumerate(parquet_files):
        print(f"Processing [{i+1}/{len(parquet_files)}]: {os.path.basename(fp)}")
        try:
            df_temp = pd.read_parquet(fp)
            if "state" not in df_temp.columns:
                print(f"  ⚠️ Warning: 'state' column missing in {fp}")
                continue

            # Extract 7th joint (index 6) from each row's 'state'
            for _, row in df_temp.iterrows():
                state = row["state"]
                if isinstance(state, (list, tuple, np.ndarray)):
                    state_arr = np.array(state)
                    if len(state_arr) > 6:
                        all_7th_joint_values.append(float(state_arr[6]))
                    else:
                        print(f"  ⚠️ Skipping row: state too short ({len(state_arr)}) in {fp}")
                else:
                    print(f"  ⚠️ Skipping row: 'state' not array-like in {fp}")
        except Exception as e:
            print(f"  ❌ Error processing {fp}: {e}")

    if all_7th_joint_values:
        global_min = min(all_7th_joint_values)
        global_max = max(all_7th_joint_values)
        print(f"\n✅ Global 7th joint (index 6) statistics:")
        print(f"   Min: {global_min:.6f}")
        print(f"   Max: {global_max:.6f}")
        print(f"   Total timesteps processed: {len(all_7th_joint_values)}")
    else:
        print("❌ No valid 7th joint data found across all files.")

def count_total_steps():
    """
    Count the total number of timesteps (rows) across all .parquet episode files
    in the same directory as PARQUET_PATH, using only Parquet metadata.
    """
    import glob

    print("\n" + "=" * 80)
    print("TOTAL TIMESTEP COUNT ACROSS ALL EPISODES")
    print("=" * 80)

    chunk_dir = os.path.dirname(PARQUET_PATH)
    parquet_files = sorted(glob.glob(os.path.join(chunk_dir, "*.parquet")))

    print(f"Found {len(parquet_files)} Parquet files in {chunk_dir}")

    total_steps = 0
    for i, fp in enumerate(parquet_files):
        try:
            pq_file = pq.ParquetFile(fp)
            num_rows = pq_file.metadata.num_rows
            total_steps += num_rows
        except Exception as e:
            print(f"  ❌ Error reading {fp}: {e}")

    print(f"\n✅ Total timesteps across all episodes: {total_steps}")
    return total_steps


# --------------------------------------------------
# CONFIG
# --------------------------------------------------
PARQUET_PATH = "/home/qtf5422/Desktop/AIRE/residual-offpolicy-rl/arm=trossen_dual/task=entire_ecu_assembly/data/chunk-000/episode_001580.parquet"
MAX_UNIQUE_CHECK = 50
MAX_PRINT_ARRAY = 20

# --------------------------------------------------
# BASIC FILE INFO
# --------------------------------------------------
print("=" * 80)
print("FILE INFO")
print("=" * 80)

if not os.path.exists(PARQUET_PATH):
    raise FileNotFoundError(f"File not found: {PARQUET_PATH}")

file_size_mb = os.path.getsize(PARQUET_PATH) / 1024**2
print(f"Path: {PARQUET_PATH}")
print(f"Size: {file_size_mb:.2f} MB")

# --------------------------------------------------
# PARQUET METADATA (LOW-LEVEL)
# --------------------------------------------------
print("\n" + "=" * 80)
print("PARQUET METADATA")
print("=" * 80)

pq_file = pq.ParquetFile(PARQUET_PATH)
meta = pq_file.metadata

print(f"Num rows: {meta.num_rows}")
print(f"Num row groups: {meta.num_row_groups}")
print(f"Created by: {meta.created_by}")
print("\nSchema:")
print(meta.schema)

# --------------------------------------------------
# LOAD INTO PANDAS
# --------------------------------------------------
print("\n" + "=" * 80)
print("LOADING DATA")
print("=" * 80)

df = pd.read_parquet(PARQUET_PATH)
print(f"Loaded DataFrame with shape: {df.shape}")

# --------------------------------------------------
# COLUMN-BY-COLUMN ANALYSIS
# --------------------------------------------------
print("\n" + "=" * 80)
print("COLUMN ANALYSIS")
print("=" * 80)

for col in df.columns:
    print("\n" + "-" * 80)
    print(f"COLUMN: {col}")
    print("-" * 80)

    series = df[col]
    dtype = series.dtype

    print(f"dtype: {dtype}")
    print(f"nulls: {series.isna().sum()}")

    # Detect nested / array-like entries
    sample_non_null = series.dropna().iloc[0] if not series.dropna().empty else None

    if isinstance(sample_non_null, (list, tuple, np.ndarray)):
        arr = np.array(sample_non_null)
        print(f"type: ARRAY")
        print(f"array shape (first row): {arr.shape}")
        print(f"array dtype: {arr.dtype}")
        print(f"first values: {arr.flatten()[:MAX_PRINT_ARRAY]}")

    elif isinstance(sample_non_null, dict):
        print(f"type: DICT / STRUCT")
        print(f"keys: {list(sample_non_null.keys())}")

    else:
        print(f"type: SCALAR")

        if pd.api.types.is_numeric_dtype(series):
            print(f"min: {series.min()}")
            print(f"max: {series.max()}")
            print(f"mean: {series.mean()}")

        unique_count = series.nunique(dropna=True)
        print(f"unique values: {unique_count}")

        if unique_count <= MAX_UNIQUE_CHECK:
            print("value counts:")
            print(series.value_counts(dropna=True).head())

    print("example value:")
    print(sample_non_null)

# --------------------------------------------------
# FIRST TIMESTEP DECODE
# --------------------------------------------------
print("\n" + "=" * 80)
print("FIRST TIMESTEP (ROW 0)")
print("=" * 80)

row0 = df.iloc[0]
for col in df.columns:
    val = row0[col]
    print(f"\n{col}:")
    if isinstance(val, (list, tuple, np.ndarray)):
        arr = np.array(val)
        print(f"  array shape: {arr.shape}")
        print(f"  first values: {arr.flatten()[:MAX_PRINT_ARRAY]}")
    else:
        print(f"  value: {val}")

print("\nInspection complete ✅")



print("==================================================")

col = "state"
print("df columns : ", df.columns)
task = df["task_index"]
print("task : ", task)

