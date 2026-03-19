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


def load_actions_from_parquet_from_state(parquet_path):
    print(f"Loading Parquet file: {parquet_path}")
    print("loading the action from the state")
    df = pd.read_parquet(parquet_path)

    # Stored shapes (by construction):
    # robot0_eef_pos        -> (T, 6)  [lx,ly,lz, rx,ry,rz]
    # robot0_eef_quat       -> (T, 8)  [lqx,lqy,lqz,lqw, rqx,rqy,rqz,rqw]
    # robot0_gripper_qpos   -> (T, 2)  [lgrip, rgrip]
    state = np.asarray(df["observation.state"].tolist(), dtype=np.float32)

    # Split left / right (flattened)
    left_pos  = state[:, 0:3]
    right_pos = state[:, 8:11]

    left_quat  = state[:, 3:7]   # (x,y,z,w)
    right_quat = state[:, 11:15]

    left_grip  = state[:, 7:8]
    right_grip = state[:, 15:16]

    # Quaternion → angle-axis (rotvec)
    # EXACT inverse of how data was created
    left_aa  = R.from_quat(left_quat).as_rotvec()
    right_aa = R.from_quat(right_quat).as_rotvec()

    # Final action vector
    action = np.concatenate(
        [
            left_pos,
            left_aa,
            left_grip,
            right_pos,
            right_aa,
            right_grip,
        ],
        axis=1,
    ).astype(np.float32)

    assert action.shape[1] == 14
    print(f"Loaded {action.shape[0]} steps, action.shape = {action.shape}")

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
    actions, df = load_actions_from_parquet_from_state(parquet_path)

    print("Moving robots to initial joint pose from Parquet...")
    reset_to_position_dataset(env, df)
    print("Robot initialized.")

    print(f"Replaying {len(actions)} steps at {control_freq} Hz")

    for t, action in enumerate(actions):
        start = time.time()

        # Unnormalize grippers: assuming normalized [0,1] → physical [0, GRIPPER_MAX]
        # Original logic: closed = 1 → physical = 0; open = 0 → physical = GRIPPER_MAX
        action = action.copy()
        action[6]  = GRIPPER_MAX * (1 - action[6])   # left gripper
        action[13] = GRIPPER_MAX * (1 - action[13])  # right gripper

        env.step(action)

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