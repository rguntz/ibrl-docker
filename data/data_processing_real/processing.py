import h5py
import numpy as np
import os
from pathlib import Path
import json
from scipy.spatial.transform import Rotation as R

import h5py
import torch
import h5py
import cv2
import numpy as np
from pathlib import Path
import matplotlib.pyplot as plt
import numpy as np

GRIPPER_MIN = 0.0           
GRIPPER_MAX = 0.04
# For 16D actions: [pos_L_3d, quat_L_4d, grip_L_1d, pos_R_3d, quat_R_4d, grip_R_1d]
GRIPPER_INDICES_16D = [7, 15]
# For 14D delta actions: [pos_delta_L_3d, rot_delta_L_3d, grip_delta_L_1d, pos_delta_R_3d, rot_delta_R_3d, grip_delta_R_1d]
GRIPPER_INDICES_14D = [6, 13]

def filter_demo_states_with_reward(qpos_full, rewards, threshold):
    ## The new implementation doesnt treshold when the reward has been reached. 

    """
    Filter states based on qpos changes, but always keep states where reward == 1.

    Args:
        qpos_full: array of shape (T, D) — joint positions
        rewards: array of shape (T,) — scalar reward per step
        threshold: minimum norm change in qpos to keep a state (ignored if reward == 1)
    Returns:
        kept_idx: list of indices to keep
    """
    states = qpos_full[:, :16]  # Assume first 16 dims are joint states
    T = states.shape[0]
    if T == 0:
        return []

    kept_idx = [0]
    last_state = states[0].copy()

    for t in range(1, T):
        # Always keep if reward == 1
        if rewards[t] == 1:
            kept_idx.append(t)
            last_state = states[t].copy()  # Update last_state to avoid re-adding same state
        # Otherwise, apply threshold
        elif np.linalg.norm(states[t] - last_state) > threshold:
            kept_idx.append(t)
            last_state = states[t].copy()

    return kept_idx


def process_dataset(input_file, output_file, threshold):
    if not os.path.exists(input_file):
        raise FileNotFoundError(f"Input file not found: {input_file}")

    with h5py.File(input_file, "r") as fin, h5py.File(output_file, "w") as fout:

        # 🔹 Copy root-level attributes
        for k, v in fin.attrs.items():
            fout.attrs[k] = v

        # 🔹 Create 'data' group and preserve its attributes
        f_data_in = fin["data"]
        f_data_out = fout.create_group("data")
        for k, v in f_data_in.attrs.items():  # PRESERVE 'env_args'
            f_data_out.attrs[k] = v

        demo_names = list(f_data_in.keys())
        print("Found demos:", len(demo_names))

        for demo_name in demo_names:
            grp_in = f_data_in[demo_name]
            print(f"\nProcessing demo: {demo_name}")

            obs_in = grp_in["obs"]
            qpos_in = obs_in["qpos"][:]
            T, _ = qpos_in.shape

            # Must have rewards to apply constraint
            if "rewards" not in grp_in:
                print(f"  Warning: 'rewards' not found in {demo_name}. Skipping.")
                continue

            rewards_in = grp_in["rewards"][:]

            # Apply new filtering that preserves reward==1 steps
            kept_idx = filter_demo_states_with_reward(qpos_in, rewards_in, threshold)
            if len(kept_idx) == 0:
                print(f"  Warning: no indices kept for {demo_name}. Skipping.")
                continue

            grp_out = f_data_out.create_group(demo_name)

            # ----- Rewards -----
            rewards_out = rewards_in[kept_idx]
            grp_out.create_dataset("rewards", data=rewards_out, compression="gzip")

            # ----- Observations -----
            obs_out_grp = grp_out.create_group("obs")
            for key in obs_in.keys():
                data = obs_in[key]
                if isinstance(data, h5py.Dataset):
                    obs_out_grp.create_dataset(key, data=data[kept_idx], compression="gzip")
                else:
                    grp = obs_out_grp.create_group(key)
                    for attr_k, attr_v in obs_in[key].attrs.items():
                        grp.attrs[attr_k] = attr_v

            # ----- Actions -----
            if "actions" in grp_in:
                actions_in = grp_in["actions"][:]
                actions_out = actions_in[kept_idx]
                grp_out.create_dataset("actions", data=actions_out, compression="gzip")

            # ----- Copy other non-obs keys -----
            for key in grp_in.keys():
                if key not in ("obs", "actions", "rewards"):
                    try:
                        grp_in.copy(key, grp_out)
                    except Exception:
                        new_grp = grp_out.create_group(key)
                        for k, v in grp_in[key].attrs.items():
                            new_grp.attrs[k] = v

    print("\nDone. Filtered dataset written to:", output_file)


def trim_first_n_steps(input_file, output_file, n_steps=15):
    if not os.path.exists(input_file):
        raise FileNotFoundError(f"Input file not found: {input_file}")

    with h5py.File(input_file, "r") as fin, h5py.File(output_file, "w") as fout:

        # 🔹 Copy root-level attributes
        for k, v in fin.attrs.items():
            fout.attrs[k] = v

        # 🔹 Create 'data' group and preserve its attributes
        f_data_in = fin["data"]
        f_data_out = fout.create_group("data")
        for k, v in f_data_in.attrs.items():
            f_data_out.attrs[k] = v

        demo_names = list(f_data_in.keys())
        print("Found demos:", len(demo_names))

        for demo_name in demo_names:
            grp_in = f_data_in[demo_name]
            print(f"\nProcessing demo: {demo_name}")

            # Determine total number of timesteps from a canonical field (e.g., qpos)
            if "obs" not in grp_in or "qpos" not in grp_in["obs"]:
                print(f"  Warning: 'obs/qpos' not found in {demo_name}. Skipping.")
                continue

            T = grp_in["obs"]["qpos"].shape[0]
            if T <= n_steps:
                print(f"  Warning: demo {demo_name} has {T} steps <= {n_steps}. Skipping.")
                continue

            kept_idx = np.arange(n_steps, T)  # Keep everything after first n_steps

            grp_out = f_data_out.create_group(demo_name)

            # ----- Rewards -----
            if "rewards" in grp_in:
                rewards_in = grp_in["rewards"][:]
                rewards_out = rewards_in[kept_idx]
                grp_out.create_dataset("rewards", data=rewards_out, compression="gzip")

            # ----- Observations -----
            obs_in = grp_in["obs"]
            obs_out_grp = grp_out.create_group("obs")
            for key in obs_in.keys():
                data = obs_in[key]
                if isinstance(data, h5py.Dataset):
                    obs_out_grp.create_dataset(key, data=data[kept_idx], compression="gzip")
                else:
                    # Handle nested groups (e.g., images with attrs)
                    subgrp = obs_out_grp.create_group(key)
                    for attr_k, attr_v in data.attrs.items():
                        subgrp.attrs[attr_k] = attr_v

            # ----- Actions -----
            if "actions" in grp_in:
                actions_in = grp_in["actions"][:]
                actions_out = actions_in[kept_idx]
                grp_out.create_dataset("actions", data=actions_out, compression="gzip")

            # ----- Copy other non-obs keys (e.g., 'durations', 'valid', etc.) -----
            for key in grp_in.keys():
                if key not in ("obs", "actions", "rewards"):
                    try:
                        # Attempt direct copy (works for datasets or groups without shape)
                        grp_in.copy(key, grp_out)
                    except (TypeError, AttributeError, RuntimeError):
                        # Fallback: recreate group and copy attributes only
                        new_grp = grp_out.create_group(key)
                        for k, v in grp_in[key].attrs.items():
                            new_grp.attrs[k] = v

    print("\nDone. Trimmed dataset written to:", output_file)

def modify_rewards_and_create_dones(input_path, output_path):
    """
    Modify rewards and create dones key:
    - Rewards: Convert from range [0, 1, 2, 3, 4] to binary [0, 1]
      (1 only when original reward == 4, else 0)
    - Dones: Create a new key with same values as modified rewards (binary)
    
    Args:
        input_path: Path to input HDF5 file
        output_path: Path to output HDF5 file
    """
    input_path = Path(input_path)
    output_path = Path(output_path)
    output_path.parent.mkdir(parents=True, exist_ok=True)

    with h5py.File(input_path, "r") as fin, h5py.File(output_path, "w") as fout:
        
        # Copy file-level and 'data' attributes
        for k, v in fin.attrs.items():
            fout.attrs[k] = v
        
        fout.create_group("data")
        for k, v in fin["data"].attrs.items():
            fout["data"].attrs[k] = v

        data_in = fin["data"]
        data_out = fout["data"]

        for demo_name in data_in.keys():
            print(f"Processing {demo_name}...")
            
            demo_in = data_in[demo_name]
            demo_out = data_out.create_group(demo_name)

            # --- Actions (unchanged) ---
            demo_out.create_dataset("actions", data=demo_in["actions"][:], compression="gzip")

            # --- Rewards (modified) ---
            rewards_original = demo_in["rewards"][:]
            # Convert to binary: 1 if reward == 4, else 0
            rewards_binary = (rewards_original == 1).astype(np.float32)
            demo_out.create_dataset("rewards", data=rewards_binary, compression="gzip")

            # --- Dones (new key, same as modified rewards) ---
            dones = rewards_binary.copy()
            demo_out.create_dataset("dones", data=dones, compression="gzip")

            # --- Observations (unchanged) ---
            obs_in = demo_in["obs"]
            obs_out = demo_out.create_group("obs")

            for obs_key in obs_in.keys():
                obs_out.create_dataset(obs_key, data=obs_in[obs_key][:], compression="gzip")

        print(f"\n🎉 Finished! Saved dataset with modified rewards and new dones key to:\n{output_path}")
        print(f"Rewards converted to binary (1 only when original reward == 4)")
        print(f"Dones key created with same values as modified rewards")
        return output_path
    

    
def inspect_right_arm(file):
    with h5py.File(file, "r") as f:
        print("Top-level keys:", list(f.keys()))

        f_data = f["data"]
        print("f_data keys:", list(f_data.keys()))
        print("The number of demos is:", len(f_data))

        # Take first demo (change to "demo_0" if needed; you use "demo_1")
        demo_key = list(f_data.keys())[0]  # safer than hardcoding "demo_1"
        f_demo_0 = f_data[demo_key]
        print(f"Processing demo: {demo_key}")
        print("f_demo_0 keys:", list(f_demo_0.keys()))

        # Actions
        action = f_demo_0["actions"][:]
        print("Action size:", action.shape)
        print("First action:", action[0])
        print("Last action:", action[-1])

        obs = f_demo_0["obs"]
        print("obs keys:", list(obs.keys()))

        # --- Extract right arm proprioception ---
        robot0_eef_pos = obs["robot0_eef_pos"][:, 3 :]       # shape (T, 3)
        robot0_eef_quat = obs["robot0_eef_quat"][:, 4 :]     # shape (T, 4)
        robot0_gripper_qpos = obs["robot0_gripper_qpos"][:, 1:]  # shape (T, 1) — assuming 2-DoF gripper, take one
        robot0_eef_aa = obs["robot0_eef_aa"][:, 3:]

        # Concatenate into full 8-D right arm observation
        right_arm_obs = np.concatenate([robot0_eef_pos, robot0_eef_aa, robot0_gripper_qpos], axis=1)  # (T, 8)

        # Right arm action: last 8 dimensions of action vector (assuming 16-D total)
        right_arm_action = action[:, 7:]  # last 8 = right arm

        T = right_arm_obs.shape[0]
        steps = range(T)

        # --- Plot all 8 dimensions: pos (3), quat (4), gripper (1) ---
        dim_names = ["EEF X", "EEF Y", "EEF Z", "RX", "RY", "RZ", "Gripper"]

        fig, axes = plt.subplots(7, 1, figsize=(12, 16), sharex=True)

        for i in range(7):
            axes[i].plot(steps, right_arm_obs[:, i], label=f'{dim_names[i]} (obs)', color='blue')
            axes[i].plot(steps, right_arm_action[:, i], label=f'{dim_names[i]} (action)', color='red', linestyle='--')
            axes[i].set_ylabel(dim_names[i])
            axes[i].legend(loc='upper right')
            axes[i].grid(True)

        axes[-1].set_xlabel("Step")
        plt.suptitle("Right Arm: Full State (Pos + Quat + Gripper) vs Action")
        plt.tight_layout(rect=[0, 0, 1, 0.97])
        plt.show()

        # --- Plot Rewards ---
        if "rewards" in f_demo_0:
            rewards = f_demo_0["rewards"][:]
            print(f"\nRewards shape: {rewards.shape}")
            print(f"First reward: {rewards[0]}")
            print(f"Last reward: {rewards[-1]}")
            print(f"Min reward: {np.min(rewards):.6f}, Max reward: {np.max(rewards):.6f}")
            print(f"Unique reward values: {np.unique(rewards)}")
            
            fig_rewards, ax_rewards = plt.subplots(1, 1, figsize=(12, 4))
            ax_rewards.plot(steps, rewards, label='Reward', color='green', linewidth=1.5, marker='o', markersize=3)
            ax_rewards.set_xlabel("Step")
            ax_rewards.set_ylabel("Reward Value")
            ax_rewards.legend()
            ax_rewards.grid(True)
            plt.suptitle("Rewards Over Time")
            plt.tight_layout()
            plt.show()

        # --- Plot Dones ---
        if "dones" in f_demo_0:
            dones = f_demo_0["dones"][:]
            print(f"\nDones shape: {dones.shape}")
            print(f"First done: {dones[0]}")
            print(f"Last done: {dones[-1]}")
            print(f"Min done: {np.min(dones):.6f}, Max done: {np.max(dones):.6f}")
            print(f"Unique done values: {np.unique(dones)}")
            print(f"Number of done flags: {np.sum(dones)}")
            
            fig_dones, ax_dones = plt.subplots(1, 1, figsize=(12, 4))
            ax_dones.plot(steps, dones, label='Done', color='orange', linewidth=1.5, marker='s', markersize=3)
            ax_dones.set_xlabel("Step")
            ax_dones.set_ylabel("Done Flag")
            ax_dones.set_ylim([-0.1, 1.1])
            ax_dones.legend()
            ax_dones.grid(True)
            plt.suptitle("Done Flags Over Time")
            plt.tight_layout()
            plt.show()


def inspect_keys(file) : 
    with h5py.File(file, "r") as f:
        print(list(f.keys()))

        f_data = f["data"]
        print("f_data keys : ", f_data.keys())
        print("the number of demos is : ", len(f_data))

        f_demo_0 = f_data["demo_1"]
        print("f_demo_0 keys : ", f_demo_0.keys())

        action = f_demo_0["actions"]
        print("action size : ", action.shape)
        print("first action : ", action[0])
        print("last action : ", action[-1])

        action_min = action[:].min()  # Convert to array for min/max
        action_max = action[:].max()
        print("Minimum action value : ", action_min)
        print("Maximum action value : ", action_max)

        obs = f_demo_0["obs"]
        print("obs keys : ", obs.keys())

        robot0_eef_pos = obs["robot0_eef_pos"]
        print("prop keys : ", robot0_eef_pos.shape)
        print("proprio first episode : ", robot0_eef_pos[0])

        robot0_eef_quat = obs["robot0_eef_quat"]
        print("prop keys : ", robot0_eef_quat.shape)
        print("proprio first episode : ", robot0_eef_quat[0])

        robot0_gripper_qpos = obs["robot0_gripper_qpos"]
        print("prop keys : ", robot0_gripper_qpos.shape)
        print("proprio first episode : ", robot0_gripper_qpos[0])

        # --- NEW: Check min/max for proprioception values ---
        print("\n=== Proprioception Value Ranges ===")

        # robot0_eef_pos
        pos = robot0_eef_pos[:]
        print("robot0_eef_pos min:", pos.min(), "max:", pos.max())

        # robot0_eef_quat
        quat = robot0_eef_quat[:]
        print("robot0_eef_quat min:", quat.min(), "max:", quat.max())

        # robot0_gripper_qpos
        grip = robot0_gripper_qpos[:]
        print("robot0_gripper_qpos min:", grip.min(), "max:", grip.max())

        images = obs["cam_high_image"]
        print("the size of the images is : ", images.shape)

        print("image : ",np.min(images[0]), np.max(images[0]))


def shift_actions_with_clipping(input_path, output_path, k):
    if k < 0:
        raise ValueError("k must be non-negative")

    input_path = Path(input_path)
    output_path = Path(output_path)

    with h5py.File(input_path, "r") as fin, h5py.File(output_path, "w") as fout:
        for key, val in fin.attrs.items():
            fout.attrs[key] = val

        data_in = fin["data"]
        data_out = fout.create_group("data")
        for key, val in data_in.attrs.items():
            data_out.attrs[key] = val

        for demo_name in data_in.keys():
            demo_in = data_in[demo_name]
            T = demo_in["actions"].shape[0]
            demo_out = data_out.create_group(demo_name)

            # Copy everything except actions
            for key in demo_in.keys():
                if key == "actions":
                    continue
                if isinstance(demo_in[key], h5py.Dataset):
                    demo_out.create_dataset(key, data=demo_in[key][:], compression="gzip")
                else:
                    demo_in.copy(key, demo_out)

            # Modify actions: a[t] = a[min(t + k, T - 1)]
            actions_orig = demo_in["actions"][:]
            actions_new = np.empty_like(actions_orig)
            for t in range(T):
                future_t = min(t + k, T - 1)
                actions_new[t] = actions_orig[future_t]

            demo_out.create_dataset("actions", data=actions_new, compression="gzip")

        print(f"✅ Actions shifted with k={k}, clipped to last action. Saved to: {output_path}")


def compute_delta_action_for_arm_new(eef_pos, eef_quat, gripper_qpos, arm_action):
    """
    Compute delta action for one arm, where arm_action uses angle-axis for rotation.
    
    Args:
        eef_pos: (3,) — current position
        eef_quat: (4,) — current orientation in [w, x, y, z]
        gripper_qpos: (1,) — unused
        arm_action: (7,) — [pos_target(3), rot_target_aa(3), grip_target(1)]

    Returns:
        delta_action: (7,) — [pos_delta(3), rot_delta_aa(3), grip_absolute(1)]
    """
    pos_target = arm_action[0:3]
    rot_target_aa = arm_action[3:6]
    grip_target = arm_action[6:7]

    # Position delta
    pos_delta = pos_target - eef_pos  # (3,)

    # Convert current quat to Rotation
    q_cur_scipy = np.array([eef_quat[1], eef_quat[2], eef_quat[3], eef_quat[0]])  # [x,y,z,w]
    R_cur = R.from_quat(q_cur_scipy)

    # Convert target angle-axis to Rotation
    R_tar = R.from_rotvec(rot_target_aa)

    # Compute relative rotation: R_rel = R_tar * R_cur^{-1}
    R_rel = R_tar * R_cur.inv()

    # Get delta as rotation vector (axis-angle)
    rot_delta_aa = R_rel.as_rotvec()  # (3,)

    # Gripper stays absolute
    grip_absolute = grip_target

    return np.concatenate([pos_delta, rot_delta_aa, grip_absolute])  # (7,)



def convert_actions_to_delta(input_path, output_path):
    """
    Convert action representation from absolute poses to delta actions.
    
    Converts 16D actions (absolute poses for both arms) to 14D actions (position/rotation deltas + absolute gripper):
    - Position delta: target_pos - current_pos
    - Rotation delta: axis-angle representation of quaternion delta
    - Gripper absolute: target_grip (unchanged from leader robot, no delta computation)
    
    This effectively transforms from:
      [pos_L, quat_L, grip_L, pos_R, quat_R, grip_R] (16D)
    to:
      [pos_delta_L, rot_delta_L, grip_absolute_L, pos_delta_R, rot_delta_R, grip_absolute_R] (14D)
    
    Args:
        input_path (str or Path): Path to input HDF5 file (with absolute action poses)
        output_path (str or Path): Path to output HDF5 file (with delta position/rotation and absolute gripper)
    """
    input_path = Path(input_path)
    output_path = Path(output_path)
    output_path.parent.mkdir(parents=True, exist_ok=True)

    with h5py.File(input_path, "r") as fin, h5py.File(output_path, "w") as fout:
        # Copy root-level attributes
        for k, v in fin.attrs.items():
            fout.attrs[k] = v

        # Process 'data' group
        data_in = fin["data"]
        data_out = fout.create_group("data")
        # Copy 'data' group attributes (e.g., env_args)
        for k, v in data_in.attrs.items():
            data_out.attrs[k] = v

        # Process each demo
        for demo_name in data_in.keys():
            demo_in = data_in[demo_name]
            demo_out = data_out.create_group(demo_name)

            print(f"Converting actions to delta for {demo_name}...")

            # Get observations
            obs_in = demo_in["obs"]
            robot0_eef_pos = obs_in["robot0_eef_pos"][:]  # (T, 6)
            robot0_eef_quat = obs_in["robot0_eef_quat"][:]  # (T, 8)
            robot0_gripper_qpos = obs_in["robot0_gripper_qpos"][:]  # (T, 2)

            # Get actions
            actions_abs = demo_in["actions"][:]  # (T, 16)
            T = actions_abs.shape[0]

            # Initialize delta actions array (T, 14)
            actions_delta = np.zeros((T, 14), dtype=np.float32)

            i = 0

            # Process each timestep
            for t in range(T):
                # Extract current state
                eef_pos_left = robot0_eef_pos[t, 0:3]
                eef_pos_right = robot0_eef_pos[t, 3:6]
                eef_quat_left = robot0_eef_quat[t, 0:4]
                eef_quat_right = robot0_eef_quat[t, 4:8]
                grip_left = robot0_gripper_qpos[t, 0:1]
                grip_right = robot0_gripper_qpos[t, 1:2]

                # Now input actions are 14D: [pos_L(3), aa_L(3), grip_L(1), pos_R(3), aa_R(3), grip_R(1)]
                action_t = actions_abs[t]  # (14,)

                # Left arm: indices 0–6
                delta_left = compute_delta_action_for_arm_new(
                    eef_pos_left, eef_quat_left, grip_left, action_t[0:7]
                )

                # Right arm: indices 7–13
                delta_right = compute_delta_action_for_arm_new(
                    eef_pos_right, eef_quat_right, grip_right, action_t[7:14]
                )

                # Concatenate: [delta_L_7d, delta_R_7d]
                actions_delta[t] = np.concatenate([delta_left, delta_right])  # (14,)


            # Save delta actions and backup of original 16D actions
            demo_out.create_dataset("actions", data=actions_delta, compression="gzip")
            demo_out.create_dataset("action_quat", data=actions_abs, compression="gzip")

            # Copy all other data unchanged
            for key in demo_in.keys():
                if key == "actions":
                    continue
                if isinstance(demo_in[key], h5py.Dataset):
                    demo_out.create_dataset(key, data=demo_in[key][:], compression="gzip")
                else:
                    # e.g., 'obs' group
                    demo_in.copy(key, demo_out)

    print(f"✅ Delta action conversion complete!")
    print(f"Input:  {input_path} (16D absolute actions)")
    print(f"Output: {output_path} (14D delta actions)")


def normalize_gripper_in_file(input_path, output_path):
    """
    Normalize only the gripper dimensions of the 'actions' dataset 
    in every demo from physical range [0, 0.044] to [-1, 1].
    
    This function works with 14D actions with absolute gripper values:
    [pos_delta_L_3d, rot_delta_L_3d, grip_absolute_L_1d, pos_delta_R_3d, rot_delta_R_3d, grip_absolute_R_1d]
    
    Gripper indices: 6 (left) and 13 (right)
    
    Steps:
      1. Clamp gripper values to [0, 0.044] (handles noise/outliers)
      2. Apply linear map: x ↦ 2*(x / 0.044) - 1  →  [-1, 1]
    All other data (obs, rewards, non-gripper actions, etc.) are copied unchanged.

    Args:
        input_path (str or Path): Path to input HDF5 dataset with 14D actions (absolute gripper)
        output_path (str or Path): Path to save normalized dataset
    """

    input_path = Path(input_path)
    output_path = Path(output_path)

    with h5py.File(input_path, "r") as fin, h5py.File(output_path, "w") as fout:
        # Copy root-level attributes
        for k, v in fin.attrs.items():
            fout.attrs[k] = v

        # Process 'data' group
        data_in = fin["data"]
        data_out = fout.create_group("data")
        # Copy 'data' group attributes (e.g., env_args)
        for k, v in data_in.attrs.items():
            data_out.attrs[k] = v

        # Process each demo
        for demo_name in data_in.keys():
            demo_in = data_in[demo_name]
            demo_out = data_out.create_group(demo_name)

            # Copy all datasets/groups except 'actions'
            for key in demo_in.keys():
                if key == "actions":
                    continue
                if isinstance(demo_in[key], h5py.Dataset):
                    demo_out.create_dataset(key, data=demo_in[key][:], compression="gzip")
                else:
                    # e.g., 'obs' group
                    demo_in.copy(key, demo_out)

            # Process 'actions': normalize grippers only
            actions = demo_in["actions"][:]  # Shape: (T, 14) for delta actions
            actions_norm = actions.copy()    # Do not modify original

            # Extract gripper dims (indices 6 and 13 for 14D delta actions)
            grippers = actions_norm[:, GRIPPER_INDICES_14D]

            # Clamp to physical limits
            grippers = np.clip(grippers, GRIPPER_MIN, GRIPPER_MAX)

            # Normalize to [-1, 1]
            grippers_norm = 2 * (grippers - GRIPPER_MIN) / (GRIPPER_MAX - GRIPPER_MIN) - 1

            # Write back
            actions_norm[:, GRIPPER_INDICES_14D] = grippers_norm

            # Save normalized actions
            demo_out.create_dataset("actions", data=actions_norm, compression="gzip")

    print(f"✅ Gripper normalization complete!")
    print(f"Input:  {input_path}")
    print(f"Output: {output_path}")


def normalize_delta_actions_in_file(input_path, output_path, delta_min_max_stats):
    """
    Normalize delta position and rotation components to [-1, 1] range.
    Gripper values remain unchanged.
    
    Uses the normalization formula: a_norm = 2 * (a_raw - min) / (max - min) - 1
    
    14D action structure: [pos_delta_L_3d, rot_delta_L_3d, grip_L_1d, pos_delta_R_3d, rot_delta_R_3d, grip_R_1d]
    Normalized indices: 0-5, 7-12 (skip gripper at 6, 13)
    """
    input_path = Path(input_path)
    output_path = Path(output_path)
    output_path.parent.mkdir(parents=True, exist_ok=True)

    # Load min/max values from stats dict
    pos_x_min, pos_x_max = delta_min_max_stats["Pos X Delta"]["min"], delta_min_max_stats["Pos X Delta"]["max"]
    pos_y_min, pos_y_max = delta_min_max_stats["Pos Y Delta"]["min"], delta_min_max_stats["Pos Y Delta"]["max"]
    pos_z_min, pos_z_max = delta_min_max_stats["Pos Z Delta"]["min"], delta_min_max_stats["Pos Z Delta"]["max"]
    
    rot_x_min, rot_x_max = delta_min_max_stats["Rot X Delta"]["min"], delta_min_max_stats["Rot X Delta"]["max"]
    rot_y_min, rot_y_max = delta_min_max_stats["Rot Y Delta"]["min"], delta_min_max_stats["Rot Y Delta"]["max"]
    rot_z_min, rot_z_max = delta_min_max_stats["Rot Z Delta"]["min"], delta_min_max_stats["Rot Z Delta"]["max"]

    with h5py.File(input_path, "r") as fin, h5py.File(output_path, "w") as fout:
        # Copy root-level attributes
        for k, v in fin.attrs.items():
            fout.attrs[k] = v

        # Create 'data' group and copy its attributes
        data_in = fin["data"]
        data_out = fout.create_group("data")
        for k, v in data_in.attrs.items():
            data_out.attrs[k] = v

        # Process each demo
        for demo_name in data_in.keys():
            demo_in = data_in[demo_name]
            demo_out = data_out.create_group(demo_name)

            # Copy all datasets/groups except 'actions'
            for key in demo_in.keys():
                if key == "actions":
                    continue
                if isinstance(demo_in[key], h5py.Dataset):
                    demo_out.create_dataset(key, data=demo_in[key][:], compression="gzip")
                else:
                    demo_in.copy(key, demo_out)

            # Process actions: normalize position and rotation deltas
            actions = demo_in["actions"][:]  # (T, 14)
            actions_norm = actions.copy()

            # === LEFT ARM ===
            # Position deltas (indices 0, 1, 2)
            actions_norm[:, 0] = 2.0 * (actions[:, 0] - pos_x_min) / (pos_x_max - pos_x_min) - 1.0
            actions_norm[:, 1] = 2.0 * (actions[:, 1] - pos_y_min) / (pos_y_max - pos_y_min) - 1.0
            actions_norm[:, 2] = 2.0 * (actions[:, 2] - pos_z_min) / (pos_z_max - pos_z_min) - 1.0
            
            # Rotation deltas (indices 3, 4, 5)
            actions_norm[:, 3] = 2.0 * (actions[:, 3] - rot_x_min) / (rot_x_max - rot_x_min) - 1.0
            actions_norm[:, 4] = 2.0 * (actions[:, 4] - rot_y_min) / (rot_y_max - rot_y_min) - 1.0
            actions_norm[:, 5] = 2.0 * (actions[:, 5] - rot_z_min) / (rot_z_max - rot_z_min) - 1.0
            # Gripper (index 6) - unchanged

            # === RIGHT ARM ===
            # Position deltas (indices 7, 8, 9)
            actions_norm[:, 7] = 2.0 * (actions[:, 7] - pos_x_min) / (pos_x_max - pos_x_min) - 1.0
            actions_norm[:, 8] = 2.0 * (actions[:, 8] - pos_y_min) / (pos_y_max - pos_y_min) - 1.0
            actions_norm[:, 9] = 2.0 * (actions[:, 9] - pos_z_min) / (pos_z_max - pos_z_min) - 1.0
            
            # Rotation deltas (indices 10, 11, 12)
            actions_norm[:, 10] = 2.0 * (actions[:, 10] - rot_x_min) / (rot_x_max - rot_x_min) - 1.0
            actions_norm[:, 11] = 2.0 * (actions[:, 11] - rot_y_min) / (rot_y_max - rot_y_min) - 1.0
            actions_norm[:, 12] = 2.0 * (actions[:, 12] - rot_z_min) / (rot_z_max - rot_z_min) - 1.0
            # Gripper (index 13) - unchanged

            # Save normalized actions
            demo_out.create_dataset("actions", data=actions_norm, compression="gzip")

    print(f"✅ Delta action normalization complete!")
    print(f"Input:  {input_path}")
    print(f"Output: {output_path}")


def normalize_delta_actions_from_json(input_path, output_path, json_stats_path):
    """
    Convenience wrapper for normalize_delta_actions_in_file() that loads stats from JSON.
    
    Args:
        input_path (str or Path): Path to input HDF5 file with delta actions
        output_path (str or Path): Path to output HDF5 file with normalized delta actions
        json_stats_path (str or Path): Path to JSON file containing min/max statistics
                                      (generated by plot_right_arm_delta_actions())
    """
    json_stats_path = Path(json_stats_path)
    
    # Load stats from JSON
    with open(json_stats_path, 'r') as f:
        delta_min_max_stats = json.load(f)
    
    # Call the normalization function
    normalize_delta_actions_in_file(input_path, output_path, delta_min_max_stats)



def truncate_demos_at_k_dones(input_path, output_path, k):
    """
    Truncate each demonstration at the time step where the k-th 'done' (value == 1) occurs.
    
    For each demo:
      - Find indices where dones == 1.
      - If there are >= k such indices, keep data up to and including the k-th one.
      - If < k, keep the full trajectory (no truncation).
    
    Args:
        input_path (str/Path): Input HDF5 file.
        output_path (str/Path): Output HDF5 file.
        k (int): Number of 'done=1' signals to wait for before truncating.
    """
    if k <= 0:
        raise ValueError("k must be a positive integer.")

    input_path = Path(input_path)
    output_path = Path(output_path)
    output_path.parent.mkdir(parents=True, exist_ok=True)

    with h5py.File(input_path, "r") as fin, h5py.File(output_path, "w") as fout:
        # Copy root-level attributes
        for key, val in fin.attrs.items():
            fout.attrs[key] = val

        # Create 'data' group and copy its attributes
        data_in = fin["data"]
        data_out = fout.create_group("data")
        for key, val in data_in.attrs.items():
            data_out.attrs[key] = val

        for demo_name in data_in.keys():
            demo_in = data_in[demo_name]
            print(f"Truncating {demo_name} at k={k} done(s)...")

            # Must have 'dones'
            if "dones" not in demo_in:
                raise KeyError(f"'dones' key missing in {demo_name}")

            dones = demo_in["dones"][:]
            T = len(dones)

            # Find indices where done == 1
            done_indices = np.where(dones == 1)[0]

            if len(done_indices) >= k:
                # Keep up to and including the k-th done (0-based: index = done_indices[k-1])
                end_index = done_indices[k - 1] + 1  # +1 because slicing is exclusive
            else:
                # Not enough done flags; keep full trajectory
                end_index = T

            # Create output demo group
            demo_out = data_out.create_group(demo_name)

            # Copy and truncate all top-level datasets/groups
            for key in demo_in.keys():
                item = demo_in[key]
                if isinstance(item, h5py.Dataset):
                    # Truncate along first dimension (time)
                    truncated_data = item[:end_index]
                    demo_out.create_dataset(key, data=truncated_data, compression="gzip")
                else:
                    # It's a group (e.g., 'obs')
                    new_group = demo_out.create_group(key)
                    # Copy group attributes
                    for attr_k, attr_v in item.attrs.items():
                        new_group.attrs[attr_k] = attr_v
                    # Recursively truncate datasets inside the group
                    for subkey in item.keys():
                        subitem = item[subkey]
                        if isinstance(subitem, h5py.Dataset):
                            truncated_subdata = subitem[:end_index]
                            new_group.create_dataset(subkey, data=truncated_subdata, compression="gzip")
                        else:
                            # Nested groups — unlikely in your structure, but safe to skip or warn
                            print(f"⚠️ Warning: Nested group {key}/{subkey} not truncated (unsupported).")
                            item.copy(subkey, new_group)

            print(f"  Kept {end_index} steps (original: {T})")

    print(f"\n✅ Truncation complete! Saved to: {output_path}")
    return output_path

def plot_right_arm_delta_actions(file, output_json="delta_action_stats_not_use.json"):
    """
    Plot and analyze right arm delta actions (14D format), along with rewards and dones.

    14D action structure: [pos_delta_L_3d, rot_delta_L_3d, grip_delta_L_1d, pos_delta_R_3d, rot_delta_R_3d, grip_delta_R_1d]
    Right arm is indices 7–13 (last 7 dimensions)

    Plots:
    - Current position vs position delta (3 dims)
    - Current gripper vs gripper delta
    - Rewards over time
    - Dones over time
    """
    with h5py.File(file, "r") as f:
        f_data = f["data"]
        demo_keys = sorted(f_data.keys())

        # --- Plot only the first demo ---
        first_demo_key = demo_keys[0]
        f_demo_0 = f_data[first_demo_key]

        actions_delta = f_demo_0["actions"][:]  # (T, 14)
        obs = f_demo_0["obs"]
        rewards = f_demo_0["rewards"][:]        # (T,)
        dones = f_demo_0["dones"][:]            # (T,)

        # Extract right arm observations (current state)
        robot0_eef_pos = obs["robot0_eef_pos"][:, 3:]      # (T, 3) - right arm pos
        robot0_gripper_qpos = obs["robot0_gripper_qpos"][:, 1:]  # (T, 1) - right arm gripper

        # Extract right arm delta actions (indices 7–13)
        right_arm_delta = actions_delta[:, 7:14]  # (T, 7)
        # Structure: [pos_delta_3d, rot_delta_3d, grip_delta_1d]

        T = right_arm_delta.shape[0]
        steps = range(T)

        # --- Plot: Position, Gripper, Rewards, Dones ---
        fig, axes = plt.subplots(6, 1, figsize=(12, 14), sharex=True)

        # Position plots (indices 0–2)
        pos_dim_names = ["Pos X", "Pos Y", "Pos Z"]
        for i in range(3):
            axes[i].plot(steps, robot0_eef_pos[:, i], label=f'{pos_dim_names[i]} (current)', color='blue', linewidth=1.5)
            axes[i].plot(steps, right_arm_delta[:, i], label=f'{pos_dim_names[i]} (delta)', color='red', linestyle='--', linewidth=1.5)
            axes[i].axhline(0, color='black', linewidth=0.5, linestyle=':')
            axes[i].set_ylabel(pos_dim_names[i])
            axes[i].legend(loc='upper right')
            axes[i].grid(True, alpha=0.3)

        # Gripper plot (index 6 of delta)
        axes[3].plot(steps, robot0_gripper_qpos[:, 0], label='Grip (current)', color='blue', linewidth=1.5)
        axes[3].plot(steps, right_arm_delta[:, 6], label='Grip (delta)', color='red', linestyle='--', linewidth=1.5)
        axes[3].axhline(0, color='black', linewidth=0.5, linestyle=':')
        axes[3].set_ylabel("Gripper")
        axes[3].legend(loc='upper right')
        axes[3].grid(True, alpha=0.3)

        # Rewards plot
        axes[4].plot(steps, rewards, label='Reward', color='green', linewidth=1.5, marker='o', markersize=2)
        axes[4].set_ylabel("Reward")
        axes[4].set_ylim(-0.1, 1.1)
        axes[4].legend(loc='upper right')
        axes[4].grid(True, alpha=0.3)

        # Dones plot
        axes[5].plot(steps, dones, label='Done', color='orange', linewidth=1.5, marker='s', markersize=2)
        axes[5].set_ylabel("Done")
        axes[5].set_xlabel("Step")
        axes[5].set_ylim(-0.1, 1.1)
        axes[5].legend(loc='upper right')
        axes[5].grid(True, alpha=0.3)

        plt.suptitle("Right Arm: State vs Delta Actions + Rewards & Dones (First Demo Only)")
        plt.tight_layout(rect=[0, 0, 1, 0.97])
        plt.show()

        # --- Compute global min/max across ALL demos (only delta actions) ---
        print("\nComputing global min and max of delta actions across all demos...")
        global_min = np.full(7, np.inf)
        global_max = np.full(7, -np.inf)

        for demo_key in demo_keys:
            demo = f_data[demo_key]
            actions_delta_full = demo["actions"][:]  # (T, 14)
            right_arm_delta_full = actions_delta_full[:, 7:14]  # (T, 7)
            global_min = np.minimum(global_min, np.min(right_arm_delta_full, axis=0))
            global_max = np.maximum(global_max, np.max(right_arm_delta_full, axis=0))

        # Prepare dictionary for JSON
        delta_dim_names = ["Pos X Delta", "Pos Y Delta", "Pos Z Delta", "Rot X Delta", "Rot Y Delta", "Rot Z Delta", "Grip"]
        stats = {}
        for i, name in enumerate(delta_dim_names):
            stats[name] = {
                "min": float(global_min[i]),
                "max": float(global_max[i])
            }

        # Save to JSON
        with open(output_json, 'w') as f_out:
            json.dump(stats, f_out, indent=4)

        print(f"\nGlobal min/max saved to: {os.path.abspath(output_json)}")
        print("\nGlobal Min and Max of Delta Actions (Right Arm) across ALL demos:")
        print("-" * 70)
        for name in delta_dim_names:
            print(f"{name:15}: min = {stats[name]['min']: .6f}, max = {stats[name]['max']: .6f}")


def extract_first_demo(input_path, output_path="dataset_final_demo0.hdf5"):
    """
    Extract only the first demo from a dataset and save it to a new HDF5 file.
    
    Args:
        input_path (str or Path): Path to input HDF5 file with multiple demos
        output_path (str or Path): Path to output HDF5 file with only the first demo
                                  (default: 'dataset_final_demo0.hdf5')
    """
    input_path = Path(input_path)
    output_path = Path(output_path)
    output_path.parent.mkdir(parents=True, exist_ok=True)

    with h5py.File(input_path, "r") as fin, h5py.File(output_path, "w") as fout:
        # Copy root-level attributes
        for k, v in fin.attrs.items():
            fout.attrs[k] = v

        # Create 'data' group and copy its attributes
        data_in = fin["data"]
        data_out = fout.create_group("data")
        for k, v in data_in.attrs.items():
            data_out.attrs[k] = v

        # Get first demo key
        demo_keys = sorted(data_in.keys())
        if not demo_keys:
            raise ValueError("Input dataset has no demos!")
        
        first_demo_key = demo_keys[0]
        print(f"Extracting demo: {first_demo_key}")

        # Copy first demo to output
        demo_in = data_in[first_demo_key]
        demo_out = data_out.create_group(first_demo_key)

        # Copy all datasets and groups from first demo
        for key in demo_in.keys():
            if isinstance(demo_in[key], h5py.Dataset):
                demo_out.create_dataset(key, data=demo_in[key][:], compression="gzip")
            elif isinstance(demo_in[key], h5py.Group):
                # Copy group structure (e.g., 'obs' group)
                group_in = demo_in[key]
                group_out = demo_out.create_group(key)
                for sub_key in group_in.keys():
                    group_out.create_dataset(sub_key, data=group_in[sub_key][:], compression="gzip")

    print(f"✅ First demo extracted successfully!")
    print(f"Input:  {input_path} (contains {len(demo_keys)} demos)")
    print(f"Output: {output_path} (contains only '{first_demo_key}')")
    return output_path

def check_demos_with_reward_one(file_path):
    """
    Check every demo in the dataset to ensure it contains at least one step where reward == 1.
    If a demo lacks such a step, print its name.

    Args:
        file_path (str or Path): Path to the HDF5 dataset file.
    """
    file_path = Path(file_path)
    if not file_path.exists():
        raise FileNotFoundError(f"Dataset file not found: {file_path}")

    with h5py.File(file_path, "r") as f:
        data_group = f["data"]
        demo_names = list(data_group.keys())
        print(f"Checking {len(demo_names)} demos for presence of reward == 1...")

        demos_without_success = []

        for demo_name in demo_names:
            demo = data_group[demo_name]
            if "rewards" not in demo:
                print(f"⚠️ Warning: 'rewards' key missing in {demo_name}")
                demos_without_success.append(demo_name)
                continue

            rewards = demo["rewards"][:]
            if not np.any(rewards == 1):
                print(f"❌ Demo '{demo_name}' has no step with reward == 1.")
                demos_without_success.append(demo_name)

        if not demos_without_success:
            print("✅ All demos contain at least one step with reward == 1.")
        else:
            print(f"\nFound {len(demos_without_success)} demo(s) without any reward == 1.")

    return demos_without_success

def print_action_min_max_per_dimension(file_path):
    """
    Print the global min and max for each dimension of the 'actions' array across all demos.
    
    Args:
        file_path (str or Path): Path to the HDF5 dataset file containing demos under 'data'.
    """
    file_path = Path(file_path)
    if not file_path.exists():
        raise FileNotFoundError(f"Dataset file not found: {file_path}")
    
    with h5py.File(file_path, "r") as f:
        data_group = f["data"]
        demo_keys = list(data_group.keys())
        
        if not demo_keys:
            raise ValueError("No demos found in the dataset.")
        
        # Initialize min/max trackers using the first demo
        first_demo = data_group[demo_keys[0]]
        if "actions" not in first_demo:
            raise KeyError(f"'actions' key not found in demo {demo_keys[0]}")
        
        actions = first_demo["actions"][:]
        if actions.size == 0:
            raise ValueError("Actions array is empty.")
        
        action_dim = actions.shape[1]
        global_min = np.full(action_dim, np.inf)
        global_max = np.full(action_dim, -np.inf)
        
        # Iterate over all demos to compute global min/max per dimension
        for demo_name in demo_keys:
            demo = data_group[demo_name]
            if "actions" not in demo:
                print(f"⚠️ Warning: 'actions' missing in {demo_name}, skipping.")
                continue
            acts = demo["actions"][:]
            if acts.shape[1] != action_dim:
                raise ValueError(f"Inconsistent action dimension in {demo_name}: expected {action_dim}, got {acts.shape[1]}")
            global_min = np.minimum(global_min, np.min(acts, axis=0))
            global_max = np.maximum(global_max, np.max(acts, axis=0))
        
        # Print results
        print(f"Action dimension: {action_dim}")
        print("-" * 50)
        for i in range(action_dim):
            print(f"Action[{i:2d}]: min = {global_min[i]: .6f}, max = {global_max[i]: .6f}")

# #####################################################################################################################################################################################
file_original = "lifting.hdf5"
inspect_right_arm(file_original)
trim_first_n_steps(input_file=file_original, output_file="cut_first_steps_dataset.hdf5", n_steps=5)
inspect_right_arm(file="cut_first_steps_dataset.hdf5")
process_dataset(input_file="cut_first_steps_dataset.hdf5", output_file="dataset_tresholded.hdf5", threshold=0.01)
inspect_right_arm(file="dataset_tresholded.hdf5")
modify_rewards_and_create_dones(input_path="dataset_tresholded.hdf5", output_path="dataset_tresholded_wr.hdf5")
inspect_right_arm(file="dataset_tresholded_wr.hdf5")

shift_actions_with_clipping(input_path="dataset_tresholded_wr.hdf5", output_path="dataset_tresholded_wr_shifted.hdf5", k=3)
inspect_right_arm(file="dataset_tresholded_wr_shifted.hdf5")

convert_actions_to_delta(input_path="dataset_tresholded_wr_shifted.hdf5", output_path="dataset_tresholded_wr_shifted_delta.hdf5")
plot_right_arm_delta_actions(file= "dataset_tresholded_wr_shifted_delta.hdf5")
normalize_gripper_in_file(input_path="dataset_tresholded_wr_shifted_delta.hdf5", output_path="dataset_tresholded_wr_shifted_delta_gripper_normed.hdf5")
plot_right_arm_delta_actions(file="dataset_tresholded_wr_shifted_delta_gripper_normed.hdf5")

truncate_demos_at_k_dones(input_path="dataset_tresholded_wr_shifted_delta_gripper_normed.hdf5", output_path="dataset_tresholded_wr_shifted_delta_gripper_normed_cut_end.hdf5", k=1)
plot_right_arm_delta_actions(file="dataset_tresholded_wr_shifted_delta_gripper_normed_cut_end.hdf5", output_json="delta_action_stats.json")

normalize_delta_actions_from_json(input_path="dataset_tresholded_wr_shifted_delta_gripper_normed_cut_end.hdf5", output_path="dataset_tresholded_wr_shifted_delta_gripper_normed_cut_end_normalized.hdf5", json_stats_path="delta_action_stats.json")
plot_right_arm_delta_actions(file="dataset_tresholded_wr_shifted_delta_gripper_normed_cut_end_normalized.hdf5")
print_action_min_max_per_dimension("dataset_tresholded_wr_shifted_delta_gripper_normed_cut_end_normalized.hdf5")

check_demos_with_reward_one(file_path="dataset_tresholded_wr_shifted_delta_gripper_normed_cut_end_normalized.hdf5")

inspect_right_arm("dataset_tresholded_wr_shifted_delta_gripper_normed_cut_end_normalized.hdf5")

# # GET THE FIRST DEMO ONLY : 
# extract_first_demo(input_path="dataset_tresholded_wr_shifted_delta_gripper_normed_cut_end_normalized.hdf5", output_path="dataset_final_demo_0.hdf5")
# plot_right_arm_delta_actions(file="dataset_final_demo_0.hdf5")

"""
IN THE FOLLOWING CODE WE COMPUTE THE NORMALIZATION CONSTANTS FOR THE TELEOPERATION SETTINGS 
(BEFORE ANY PROCESSING OF THE FILE). IN THAT WAY THE NORMALIZATION CONSTANTS ARE MORE ACCURATE
FOR THE INFERENCE ON THE REAL ROBOT. 
"""

modify_rewards_and_create_dones(input_path="cut_first_steps_dataset.hdf5", output_path="raw_cut_first_steps_dataset.hdf5")
convert_actions_to_delta(input_path="raw_cut_first_steps_dataset.hdf5", output_path="raw_cut_first_steps_dataset_delta.hdf5")
normalize_gripper_in_file(input_path="raw_cut_first_steps_dataset_delta.hdf5", output_path="raw_cut_first_steps_dataset_delta_gripper_normed.hdf5")
plot_right_arm_delta_actions(file="raw_cut_first_steps_dataset_delta_gripper_normed.hdf5", output_json="delta_action_stats_teleop.json")