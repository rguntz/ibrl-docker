#!/usr/bin/env python3
"""
Playback recorded demonstrations on real Trossen robots using Trossen_env.
Plays all Parquet files in a given directory.
"""

import numpy as np
import pandas as pd
import argparse
import time
from pathlib import Path

from env.trossen_env import Trossen_env
from scipy.spatial.transform import Rotation as R

# =========================
# DEFAULTS
# =========================
DEFAULT_DATA_DIR = "/home/qtf5422/Desktop/AIRE/residual-offpolicy-rl/arm=trossen_dual/task=entire_ecu_assembly/data/chunk-000"
GRIPPER_MAX = 0.044

import numpy as np
import pandas as pd
from scipy.spatial.transform import Rotation as R


# =========================
# LOAD ACTIONS FROM PARQUET
# =========================
def load_actions_from_parquet(parquet_path):
    print("calling the load action from action")
    print(f"Loading Parquet file: {parquet_path}")
    df = pd.read_parquet(parquet_path)

    action = np.array(df["action"].tolist(), dtype=np.float32)      # (T, 7) → right
    print("action.shape[0]", action.shape[0])
    assert action.shape[1] == 14

    print(f"Loaded {action.shape[0]} steps from Parquet")
    return action, df


# =========================
# RESET TO INITIAL JOINT POSE FROM PARQUET
# =========================
def reset_to_position_dataset(env, df):
    # Extract first timestep (row 0)
    first_right  = np.array(df["joint_states"].iloc[0], dtype=np.float32)        # (7,) → left arm + gripper
    first_left = np.array(df["joint_states_2"].iloc[0], dtype=np.float32)  # (7,) → right arm + gripper

    print("First timestep – Left arm joint angles (including gripper):")
    print(first_left)
    print("\nFirst timestep – Right arm joint angles (including gripper):")
    print(first_right)

    # Clip gripper values to valid range [0.0, GRIPPER_MAX]
    gripper_left  = np.clip(first_left[6],  0.0, GRIPPER_MAX)
    gripper_right = np.clip(first_right[6], 0.0, GRIPPER_MAX)

    print("Moving robot to the initial position from Parquet dataset...")
    env.move_followers_home(
        home_arm_left=first_left[:6],
        home_arm_right=first_right[:6],
        gripper_open_left=gripper_left,
        gripper_open_right=gripper_right
    )
    print("Finished moving robot to initial position.")


# =========================
# PLAYBACK SINGLE EPISODE
# =========================
def playback_episode(parquet_path, env, control_freq):
    actions, df = load_actions_from_parquet(parquet_path)

    print("Moving robots to initial joint pose from Parquet...")
    reset_to_position_dataset(env, df)
    print("Robot initialized.")

    print(f"Replaying {len(actions)} steps at {control_freq} Hz")

    for t, delta_action in enumerate(actions):
        start = time.time()

        # Initialize current EE state from the first "state" entry
        state_0 = np.array(df["observation.state"].iloc[t], dtype=np.float32)

        # Left
        curr_l_pos = state_0[0:3].copy()
        curr_l_quat = state_0[3:7].copy()  # x, y, z, w
        # Right
        curr_r_pos = state_0[8:11].copy()
        curr_r_quat = state_0[11:15].copy()
        
        # Extract delta components
        l_pos_delta = delta_action[0:3]
        l_aa_delta = delta_action[3:6]
        l_grip_abs = delta_action[6]      # already absolute → no change
        r_pos_delta = delta_action[7:10]
        r_aa_delta = delta_action[10:13]
        r_grip_abs = delta_action[13]     # already absolute

        # Update left position
        curr_l_pos = curr_l_pos + l_pos_delta

        # Update left rotation
        l_rot_curr = R.from_quat(curr_l_quat).as_matrix()  # [x,y,z,w] → matrix
        l_rot_delta = R.from_rotvec(l_aa_delta).as_matrix()
        l_rot_new = l_rot_delta @ l_rot_curr
        curr_l_quat = R.from_matrix(l_rot_new).as_quat()  # returns [x,y,z,w]

        # Update right position
        curr_r_pos = curr_r_pos + r_pos_delta

        # Update right rotation
        r_rot_curr = R.from_quat(curr_r_quat).as_matrix()
        r_rot_delta = R.from_rotvec(r_aa_delta).as_matrix()
        r_rot_new = r_rot_delta @ r_rot_curr
        curr_r_quat = R.from_matrix(r_rot_new).as_quat()

        # Assemble absolute action: [pos (3), quat (4), grip (1)] per arm → but env expects angle-axis!
        # So convert quat → angle-axis for the action vector
        l_aa_abs = R.from_quat(curr_l_quat).as_rotvec()
        r_aa_abs = R.from_quat(curr_r_quat).as_rotvec()

        # Build absolute action in format expected by env.step(): (14,) = [l_pos(3), l_aa(3), l_grip, r_pos(3), r_aa(3), r_grip]
        abs_action = np.concatenate([
            curr_l_pos,
            l_aa_abs,
            [l_grip_abs],
            curr_r_pos,
            r_aa_abs,
            [r_grip_abs]
        ])

        # Unnormalize grippers: assuming normalized [0,1] → physical [0, GRIPPER_MAX]
        # Original logic: closed = 1 → physical = 0; open = 0 → physical = GRIPPER_MAX
        abs_action[6]  = GRIPPER_MAX * (1 - abs_action[6])   # left gripper
        abs_action[13] = GRIPPER_MAX * (1 - abs_action[13])  # right gripper

        env.step(abs_action)

        elapsed = time.time() - start
        sleep_time = max(0.0, env.control_dt - elapsed)
        time.sleep(sleep_time)

        if (t + 1) % 100 == 0:
            print(f"Step {t+1}/{len(actions)}")
# =========================
# MAIN
# =========================
def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--data-dir', type=str, default=DEFAULT_DATA_DIR,
                        help="Directory containing Parquet files")
    parser.add_argument('--follower-left-ip', type=str, default='192.168.1.5')
    parser.add_argument('--follower-right-ip', type=str, default='192.168.1.3')
    parser.add_argument('--control-freq', type=float, default=20.0)
    parser.add_argument('--model', type=str, default='wxai_v0')
    parser.add_argument('--skip-action-playback', action='store_true', default=False)
    args = parser.parse_args()

    if args.skip_action_playback:
        print("Skipping robot playback.")
        return

    data_dir = Path(args.data_dir)
    if not data_dir.exists():
        raise FileNotFoundError(f"Data directory does not exist: {data_dir}")

    parquet_files = sorted(data_dir.glob("*.parquet"))
    if not parquet_files:
        raise ValueError(f"No .parquet files found in {data_dir}")

    print(f"Found {len(parquet_files)} Parquet files. Starting playback...")

    # Initialize environment once (assumes same hardware setup for all episodes)
    env = Trossen_env(
        follower_left_ip=args.follower_left_ip,
        follower_right_ip=args.follower_right_ip,
        control_dt=1.0 / args.control_freq,
        model=args.model,
        max_episode_steps=10000  # Will be overridden per episode; safe upper bound
    )

    try:
        for i, parquet_path in enumerate(parquet_files):
            print(f"\n=== Playing episode {i+1}/{len(parquet_files)}: {parquet_path.name} ===")
            env.reset()
            print("playing episode : ", parquet_path)
            playback_episode(parquet_path, env, args.control_freq)
    finally:
        env.stop_background_capture()


if __name__ == "__main__":
    main()