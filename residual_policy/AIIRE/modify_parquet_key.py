import os
import pandas as pd
from tqdm import tqdm

CHUNK_DIR = "/home/qtf5422/Desktop/AIRE/residual-offpolicy-rl/arm=trossen_dual/task=entire_ecu_assembly/data/chunk-000"

# Get list of all .parquet files
parquet_files = [f for f in os.listdir(CHUNK_DIR) if f.endswith(".parquet")]
parquet_files.sort()  # Optional: ensures consistent order

print(f"Found {len(parquet_files)} parquet files. Processing...")

for filename in tqdm(parquet_files, desc="Renaming 'state' column"):
    filepath = os.path.join(CHUNK_DIR, filename)
    
    df = pd.read_parquet(filepath)
    
    if "state" in df.columns and "observation.state" not in df.columns:
        df = df.rename(columns={"state": "observation.state"})
        df.to_parquet(filepath, index=False)
    # No need to print per-file messages to avoid cluttering tqdm bar