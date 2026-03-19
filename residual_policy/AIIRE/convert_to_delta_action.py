import os
import pandas as pd
import numpy as np
from pathlib import Path
from scipy.spatial.transform import Rotation as R

# --------------------------------------------------
# Helper Functions
# --------------------------------------------------

def angle_axis_to_rotation_matrix(angle_axis):
    angle = np.linalg.norm(angle_axis)
    if angle < 1e-8:
        return np.eye(3)
    axis = angle_axis / angle
    return R.from_rotvec(axis * angle).as_matrix()

def rotation_matrix_to_angle_axis(matrix):
    return R.from_matrix(matrix).as_rotvec()

def convert_absolute_action_to_delta_inplace(df):
    """
    Converts 'action' column from absolute to delta (except grippers).
    Modifies df in-place.
    """
    new_actions = []
    for idx in range(len(df)):
        state = np.array(df.iloc[idx]["state"], dtype=np.float64)
        action_abs = np.array(df.iloc[idx]["action_absolute"], dtype=np.float64)

        # Parse state (16 elements)
        l_pos_s = state[0:3]
        l_quat_s = state[3:7]  # x, y, z, w
        # l_grip_s = state[7]  # not used
        r_pos_s = state[8:11]
        r_quat_s = state[11:15]  # x, y, z, w
        # r_grip_s = state[15]  # not used

        # Parse action (14 elements)
        l_pos_a = action_abs[0:3]
        l_aa_a = action_abs[3:6]
        l_grip_a = action_abs[6]
        r_pos_a = action_abs[7:10]
        r_aa_a = action_abs[10:13]
        r_grip_a = action_abs[13]

        # Delta positions
        l_pos_delta = l_pos_a - l_pos_s
        r_pos_delta = r_pos_a - r_pos_s

        # Current rotations (from state quaternions)
        # Note: scipy expects [x, y, z, w]
        l_rot_s = R.from_quat(l_quat_s).as_matrix()
        r_rot_s = R.from_quat(r_quat_s).as_matrix()

        # Target rotations (from action angle-axis)
        l_rot_a = angle_axis_to_rotation_matrix(l_aa_a)
        r_rot_a = angle_axis_to_rotation_matrix(r_aa_a)

        # Relative rotation: R_delta = R_target @ R_current^T
        l_rot_delta = l_rot_a @ l_rot_s.T
        r_rot_delta = r_rot_a @ r_rot_s.T

        # Back to angle-axis
        l_aa_delta = rotation_matrix_to_angle_axis(l_rot_delta)
        r_aa_delta = rotation_matrix_to_angle_axis(r_rot_delta)

        # Assemble new action
        delta_action = np.concatenate([
            l_pos_delta,
            l_aa_delta,
            [l_grip_a],
            r_pos_delta,
            r_aa_delta,
            [r_grip_a]
        ])

        new_actions.append(delta_action)

    df["action"] = new_actions

# --------------------------------------------------
# Main Processing Loop
# --------------------------------------------------

CHUNK_DIR = "/home/qtf5422/Desktop/AIRE/residual-offpolicy-rl/arm=trossen_dual/task=entire_ecu_assembly/data/chunk-000"
parquet_files = sorted(Path(CHUNK_DIR).glob("*.parquet"))

print(f"Found {len(parquet_files)} Parquet file(s) in {CHUNK_DIR}")

for i, parquet_path in enumerate(parquet_files, 1):
    print(f"\n[{i}/{len(parquet_files)}] Processing: {parquet_path.name}")
    df = pd.read_parquet(parquet_path)

    if "state" not in df.columns or "action" not in df.columns:
        print(f"  ⚠️ Skipping: missing 'state' or 'action' column")
        continue

    # Rename original action column to action_absolute
    df.rename(columns={"action": "action_absolute"}, inplace=True)
    
    # Perform conversion (will create new "action" column)
    convert_absolute_action_to_delta_inplace(df)

    # Save back
    df.to_parquet(parquet_path, index=False)
    print(f"  ✅ Updated and saved.")

print("\n✅ All files processed successfully!")