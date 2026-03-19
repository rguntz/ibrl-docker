#!/usr/bin/env python3
"""
Add bimanual action column to parquet files.

This script processes all parquet files in a directory that already contain:
  - action_end_effector_2 (left arm action)
  - action_end_effector   (right arm action)

It adds a new column:
  - robot0_action: [left_action..., right_action...] (concatenated)

Original data is preserved; new column is appended.
"""

import numpy as np
import pandas as pd
from pathlib import Path
from tqdm import tqdm


# ==================================================
# CONFIGURATION
# ==================================================
class Config:
    INPUT_DIR = "/home/qtf5422/Desktop/AIRE/residual-offpolicy-rl/arm=trossen_dual/task=entire_ecu_assembly/data/chunk-000"
    
    # Input action columns (must exist)
    LEFT_ACTION_COL = "action_end_effector_2"
    RIGHT_ACTION_COL = "action_end_effector"
    
    # Output column name
    ROBOT0_ACTION_COL = "action"


# ==================================================
# PROCESSING FUNCTION
# ==================================================
def process_parquet_file(filepath, config):
    try:
        df = pd.read_parquet(filepath)

        # Check required columns
        if config.LEFT_ACTION_COL not in df.columns or config.RIGHT_ACTION_COL not in df.columns:
            print(f"  ⚠️ Skipping {filepath.name}: missing action columns")
            return False

        # Stack actions
        left_act = np.stack(df[config.LEFT_ACTION_COL].values)   # (T, D1)
        right_act = np.stack(df[config.RIGHT_ACTION_COL].values) # (T, D2)

        # Concatenate along last dimension
        robot0_action = np.concatenate([left_act, right_act], axis=1)  # (T, D1+D2)

        # Add as list of arrays to preserve nested structure in parquet
        df[config.ROBOT0_ACTION_COL] = list(robot0_action)

        # Save back (overwrite with new column)
        df.to_parquet(filepath, engine='pyarrow', compression='snappy')
        return True

    except Exception as e:
        print(f"  ❌ Error processing {filepath.name}: {e}")
        return False


# ==================================================
# MAIN LOGIC
# ==================================================
def process_directory(input_dir, config):
    input_path = Path(input_dir)
    if not input_path.exists():
        print(f"❌ Directory not found: {input_dir}")
        return

    parquet_files = sorted(input_path.glob("*.parquet"))
    if not parquet_files:
        print(f"❌ No parquet files found in: {input_dir}")
        return

    print(f"Found {len(parquet_files)} parquet files")
    print("=" * 60)

    success_count = 0
    for filepath in tqdm(parquet_files, desc="Adding robot0_action"):
        if process_parquet_file(filepath, config):
            success_count += 1

    print("=" * 60)
    print(f"✅ Successfully updated: {success_count}/{len(parquet_files)} files")
    print("=" * 60)


def main():
    config = Config()
    print("=" * 60)
    print("ADDING BIMANUAL ACTION COLUMN")
    print("=" * 60)
    print(f"Input dir: {config.INPUT_DIR}")
    print(f"From: {config.LEFT_ACTION_COL}, {config.RIGHT_ACTION_COL}")
    print(f"To:   {config.ROBOT0_ACTION_COL}")
    print("=" * 60)

    process_directory(config.INPUT_DIR, config)


if __name__ == "__main__":
    main()