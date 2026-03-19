import os
import pandas as pd

folder = "/home/qtf5422/Desktop/AIRE/residual-offpolicy-rl/arm=trossen_dual/task=entire_ecu_assembly/data/chunk-000"

# List all Parquet files
parquet_files = [f for f in os.listdir(folder) if f.endswith(".parquet")]

if not parquet_files:
    raise RuntimeError(f"No Parquet files found in {folder}")

for f in parquet_files:
    path = os.path.join(folder, f)
    df = pd.read_parquet(path)
    
    if "next.done" not in df.columns:
        raise ValueError(f"'next.done' column missing in file: {f}")
    
    # Check if all values are non-empty / non-NaN
    if df["next.done"].isnull().any():
        raise ValueError(f"'next.done' column has NaN values in file: {f}")
    
    if df["next.done"].size == 0:
        raise ValueError(f"'next.done' column is empty in file: {f}")

print("All parquet files have a valid 'next.done' column ✅")