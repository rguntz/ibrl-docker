#!/usr/bin/env python3
"""
Adds a 'next.done' column to existing parquet files based on 'video_results.json'.
Logic:
1. Load video_results.json to determine which episodes are "successful" (true).
2. For each parquet file:
   - If the episode ID is NOT in the JSON -> Crash (KeyError).
   - If the episode ID is in JSON and value is True:
     - Set 'next.done' = 0 for all steps except the last.
     - Set 'next.done' = 1 for the last step.
   - If the episode ID is in JSON and value is False:
     - Set 'next.done' = 0 for ALL steps.
"""

import os
import json
import numpy as np
import pandas as pd
from pathlib import Path
from tqdm import tqdm


# ==================================================
# CONFIGURATION
# ==================================================
class Config:
    # Directory containing the parquet files
    DATA_DIR = "/home/qtf5422/Desktop/AIRE/residual-offpolicy-rl/arm=trossen_dual/task=entire_ecu_assembly/data/chunk-000"
    
    # Path to the JSON file containing success flags
    JSON_FILE = "/home/qtf5422/Desktop/AIRE/residual-offpolicy-rl/BMW/video_results.json"
    
    # Column name to add
    NEXT_DONE_COL = "next.done"


def load_success_map(json_path):
    """Loads the JSON and returns a dict mapping episode_id (int) -> bool."""
    if not os.path.exists(json_path):
        raise FileNotFoundError(f"JSON file not found: {json_path}")
    
    with open(json_path, 'r') as f:
        data = json.load(f)
    
    success_map = {}
    for filename, is_success in data.items():
        # Expected format: "episode_000916.mp4"
        if not filename.startswith("episode_") or not filename.endswith(".mp4"):
            continue
        
        # Extract the number part
        # Remove "episode_" prefix and ".mp4" suffix
        id_str = filename.replace("episode_", "").replace(".mp4", "")
        try:
            ep_id = int(id_str)
            success_map[ep_id] = bool(is_success)
        except ValueError:
            print(f"Warning: Could not parse episode ID from filename: {filename}")
            
    return success_map


def extract_episode_id_from_parquet(filepath):
    """
    Extracts the episode ID integer from a parquet filename.
    Assumes naming convention like: .../episode_000099.parquet
    """
    name = filepath.stem  # e.g., "episode_000099"
    if not name.startswith("episode_"):
        raise ValueError(f"Unexpected filename format (missing 'episode_'): {filepath.name}")
    
    id_str = name.replace("episode_", "")
    try:
        return int(id_str)
    except ValueError:
        raise ValueError(f"Could not parse episode ID from filename: {filepath.name}")


def process_parquet_file(input_path, output_path, success_map):
    try:
        # 1. Extract Episode ID
        ep_id = extract_episode_id_from_parquet(input_path)
        
        # 2. Check JSON Map (Crash if missing as requested)
        if ep_id not in success_map:
            # This will raise a KeyError causing the script to crash
            raise KeyError(f"Episode ID {ep_id} ({input_path.name}) not found in video_results.json")
        
        is_successful_episode = success_map[ep_id]
        
        # 3. Read Parquet
        df = pd.read_parquet(input_path)
        T = len(df)
        
        if T == 0:
            print(f"  ⚠️ Skipping {input_path.name}: Empty dataframe")
            return False

        # 4. Create next.done array
        # Default to 0
        next_done = np.zeros(T, dtype=np.int8)
        
        # If the episode is marked as successful in JSON, set the LAST step to 1
        if is_successful_episode:
            next_done[-1] = 1
        
        # 5. Assign to DataFrame
        # We store as a list of scalars or a numpy array depending on how your pipeline expects it.
        # Usually for RL datasets, a flat column of scalars is preferred over arrays of shape (1,).
        df[Config.NEXT_DONE_COL] = next_done

        # 6. Save
        # Overwriting the file or saving to new location? 
        # The prompt implies modifying the structure. Let's save to output_path.
        output_path.parent.mkdir(parents=True, exist_ok=True)
        df.to_parquet(output_path, engine='pyarrow', compression='snappy')
        
        status = "Success (done=1 at end)" if is_successful_episode else "Success (all done=0)"
        print(f"  ✅ {input_path.name}: {status}")
        return True

    except KeyError as ke:
        # Re-raise KeyError specifically so the main loop stops and you see the crash
        raise ke
    except Exception as e:
        print(f"  ❌ Error processing {input_path.name}: {e}")
        return False


# ==================================================
# MAIN
# ==================================================
def main():
    data_dir = Path(Config.DATA_DIR)
    json_path = Path(Config.JSON_FILE)

    if not data_dir.exists():
        raise FileNotFoundError(f"Data directory not found: {data_dir}")

    # Load the ground truth JSON first
    print(f"Loading success map from: {json_path}")
    success_map = load_success_map(json_path)
    print(f"Loaded {len(success_map)} episode statuses.")
    print("-" * 60)

    parquet_files = sorted(data_dir.glob("*.parquet"))
    # Filter only episode files to avoid processing unrelated parquets if any
    parquet_files = [f for f in parquet_files if f.stem.startswith("episode_")]
    
    if not parquet_files:
        raise ValueError(f"No episode parquet files found in {data_dir}")

    print(f"Processing {len(parquet_files)} episode files...")
    
    success_count = 0
    for fp in tqdm(parquet_files, desc="Adding next.done"):
        # We overwrite in place or save to same dir? 
        # To be safe, we write to the same path (overwriting) or a temp file then move.
        # Given the context of data prep scripts, overwriting is common, but let's write to the same path.
        if process_parquet_file(fp, fp, success_map):
            success_count += 1

    print("-" * 60)
    print(f"✅ Done: {success_count}/{len(parquet_files)} files processed.")


if __name__ == "__main__":
    main()