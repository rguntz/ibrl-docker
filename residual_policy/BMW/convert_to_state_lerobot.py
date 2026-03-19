#!/usr/bin/env python3
"""
Convert existing end-effector columns (rotvec-based) in parquet files
to a unified 16D state vector using quaternions (xyzw), and save to new files.
Also renames any pre-existing 'state' column to 'joint_states'.
"""

import os
import numpy as np
import pandas as pd
from scipy.spatial.transform import Rotation as R
from pathlib import Path
from tqdm import tqdm


# ==================================================
# CONFIGURATION
# ==================================================
class Config:
    INPUT_DIR = "/home/qtf5422/Desktop/AIRE/residual-offpolicy-rl/arm=trossen_dual/task=entire_ecu_assembly/data/chunk-000"
    OUTPUT_DIR = "/home/qtf5422/Desktop/AIRE/residual-offpolicy-rl/arm=trossen_dual/task=entire_ecu_assembly/data/chunk-000"

    # Input column names (already computed by your first script)
    LEFT_EE_COL = "state_end_effector_2"   # left arm EE: [x,y,z, rx,ry,rz, grip]
    RIGHT_EE_COL = "state_end_effector"    # right arm EE: [x,y,z, rx,ry,rz, grip]

    # Output column name
    NEW_STATE_COL = "state"


# ==================================================
# CONVERSION FUNCTION
# ==================================================
def rotvec_to_quat_xyzw(rotvec):
    """Convert (3,) rotation vector to (4,) quaternion in xyzw format."""
    return R.from_rotvec(rotvec).as_quat()  # returns [x, y, z, w]


def process_parquet_file(input_path, output_path):
    try:
        df = pd.read_parquet(input_path)

        # Ensure required EE columns exist
        if Config.LEFT_EE_COL not in df.columns or Config.RIGHT_EE_COL not in df.columns:
            print(f"  ⚠️ Skipping {input_path.name}: missing EE columns")
            return False

        # Rename existing 'state' column to 'joint_states' if it exists
        if Config.NEW_STATE_COL in df.columns:
            df = df.rename(columns={Config.NEW_STATE_COL: "joint_states"})

        # Stack EE arrays
        ee_left = np.stack(df[Config.LEFT_EE_COL].values)    # (T, 7)
        ee_right = np.stack(df[Config.RIGHT_EE_COL].values)  # (T, 7)

        T = ee_left.shape[0]

        # Validate shape
        assert ee_left.shape == (T, 7), f"Left EE shape mismatch: {ee_left.shape}"
        assert ee_right.shape == (T, 7), f"Right EE shape mismatch: {ee_right.shape}"

        # Prepare output array: (T, 16)
        new_state = np.zeros((T, 16))

        for t in range(T):
            # Left arm
            l_pos = ee_left[t, :3]
            l_rotvec = ee_left[t, 3:6]
            l_grip = ee_left[t, 6]
            l_quat = rotvec_to_quat_xyzw(l_rotvec)

            # Right arm
            r_pos = ee_right[t, :3]
            r_rotvec = ee_right[t, 3:6]
            r_grip = ee_right[t, 6]
            r_quat = rotvec_to_quat_xyzw(r_rotvec)

            # Assemble: [l_pos(3), l_quat(4), l_grip(1), r_pos(3), r_quat(4), r_grip(1)]
            new_state[t] = np.concatenate([
                l_pos, l_quat, [l_grip],
                r_pos, r_quat, [r_grip]
            ])

        # Assign the new 16D state to the 'state' column
        df[Config.NEW_STATE_COL] = list(new_state)

        # Save to new file
        output_path.parent.mkdir(parents=True, exist_ok=True)
        df.to_parquet(output_path, engine='pyarrow', compression='snappy')
        return True

    except Exception as e:
        print(f"  ❌ Error processing {input_path.name}: {e}")
        return False


# ==================================================
# MAIN
# ==================================================
def main():
    input_dir = Path(Config.INPUT_DIR)
    output_dir = Path(Config.OUTPUT_DIR)

    if not input_dir.exists():
        raise FileNotFoundError(f"Input directory not found: {input_dir}")

    parquet_files = sorted(input_dir.glob("*.parquet"))
    if not parquet_files:
        raise ValueError(f"No .parquet files found in {input_dir}")

    print(f"Processing {len(parquet_files)} files...")
    print(f"Input:  {input_dir}")
    print(f"Output: {output_dir}")
    print("-" * 60)

    success_count = 0
    for fp in tqdm(parquet_files, desc="Converting EE → state16"):
        out_fp = output_dir / fp.name  # same filename, new dir
        if process_parquet_file(fp, out_fp):
            success_count += 1

    print("-" * 60)
    print(f"✅ Done: {success_count}/{len(parquet_files)} files processed.")


if __name__ == "__main__":
    main()