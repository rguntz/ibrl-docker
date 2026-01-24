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


def filter_initial_static_states(input_file, output_file, threshold):
    """
    Filter each demonstration by removing leading timesteps where the joint positions (qpos)
    are nearly identical to the first timestep, up to the first timestep that exceeds `threshold`.
    
    Only the prefix of static states (compared to step 0) is removed. The rest of the trajectory
    is kept intact—even if it later becomes static again.
    
    Args:
        input_file (str): Path to input HDF5 file.
        output_file (str): Path to output HDF5 file.
        threshold (float): Minimum L2 norm of qpos change (vs. step 0) to consider a state non-static.
    """
    if not os.path.exists(input_file):
        raise FileNotFoundError(f"Input file not found: {input_file}")

    with h5py.File(input_file, "r") as fin, h5py.File(output_file, "w") as fout:
        # Copy root-level attributes
        for k, v in fin.attrs.items():
            fout.attrs[k] = v

        # Create 'data' group and copy its attributes
        f_data_in = fin["data"]
        f_data_out = fout.create_group("data")
        for k, v in f_data_in.attrs.items():
            f_data_out.attrs[k] = v

        demo_names = list(f_data_in.keys())
        print("Found demos:", len(demo_names))

        for demo_name in demo_names:
            grp_in = f_data_in[demo_name]
            print(f"\nProcessing demo: {demo_name}")

            obs_in = grp_in["obs"]
            qpos_in = obs_in["qpos"][:]
            T = qpos_in.shape[0]

            if T == 0:
                print(f"  Warning: empty demo {demo_name}. Skipping.")
                continue

            # Find the first index where ||qpos[t] - qpos[0]|| > threshold
            start_idx = 0
            for t in range(1, T):
                if np.linalg.norm(qpos_in[t, :16] - qpos_in[0, :16]) > threshold:
                    start_idx = t
                    break
            # If no such t is found, start_idx remains 0 (keep full demo)

            grp_out = f_data_out.create_group(demo_name)

            # ----- Copy all top-level datasets, sliced from start_idx -----
            for key in grp_in.keys():
                if key == "obs":
                    continue  # handle obs separately below

                data = grp_in[key]
                if isinstance(data, h5py.Dataset):
                    grp_out.create_dataset(key, data=data[start_idx:], compression="gzip")
                else:
                    # Should not typically occur, but safe fallback
                    new_grp = grp_out.create_group(key)
                    for attr_k, attr_v in data.attrs.items():
                        new_grp.attrs[attr_k] = attr_v

            # ----- Observations -----
            obs_out_grp = grp_out.create_group("obs")
            for obs_key in obs_in.keys():
                data = obs_in[obs_key]
                if isinstance(data, h5py.Dataset):
                    obs_out_grp.create_dataset(obs_key, data=data[start_idx:], compression="gzip")
                else:
                    subgrp = obs_out_grp.create_group(obs_key)
                    for attr_k, attr_v in data.attrs.items():
                        subgrp.attrs[attr_k] = attr_v

    print("\n✅ Initial static states filtered. Dataset saved to:", output_file)


def transpose_image_dimensions(input_path, output_path):
    """
    Transpose image observations from (T, 3, 128, 128) to (T, 128, 128, 3).
    All other data in the HDF5 file remains unchanged.
    
    Args:
        input_path (str or Path): Path to input HDF5 file
        output_path (str or Path): Path to save transposed dataset
    """
    input_path = Path(input_path)
    output_path = Path(output_path)
    output_path.parent.mkdir(parents=True, exist_ok=True)

    with h5py.File(input_path, "r") as fin, h5py.File(output_path, "w") as fout:
        # Copy root-level attributes
        for k, v in fin.attrs.items():
            fout.attrs[k] = v

        # Create 'data' group and copy attributes
        data_in = fin["data"]
        data_out = fout.create_group("data")
        for k, v in data_in.attrs.items():
            data_out.attrs[k] = v

        # Process each demo
        for demo_name in data_in.keys():
            print(f"Processing {demo_name}...")
            
            demo_in = data_in[demo_name]
            demo_out = data_out.create_group(demo_name)

            # Process all keys in the demo
            for key in demo_in.keys():
                if key == "obs":
                    # Handle observations group specially
                    obs_in = demo_in["obs"]
                    obs_out = demo_out.create_group("obs")
                    
                    # Copy observation group attributes
                    for attr_k, attr_v in obs_in.attrs.items():
                        obs_out.attrs[attr_k] = attr_v
                    
                    # Process each observation key
                    for obs_key in obs_in.keys():
                        obs_data = obs_in[obs_key][:]
                        
                        # Check if this is an image dataset (3D or 4D with channel dimension)
                        # Images have shape (T, 3, H, W) where channels=3 is first
                        if len(obs_data.shape) == 4 and obs_data.shape[1] == 3:
                            print(f"  Transposing {obs_key}: {obs_data.shape} -> ", end="")
                            # Transpose from (T, 3, H, W) to (T, H, W, 3)
                            obs_data_transposed = np.transpose(obs_data, (0, 2, 3, 1))
                            print(f"{obs_data_transposed.shape}")
                            obs_out.create_dataset(obs_key, data=obs_data_transposed, compression="gzip")
                        else:
                            # Not an image, copy as-is
                            obs_out.create_dataset(obs_key, data=obs_data, compression="gzip")
                
                elif isinstance(demo_in[key], h5py.Dataset):
                    # Copy non-obs datasets unchanged
                    demo_out.create_dataset(key, data=demo_in[key][:], compression="gzip")
                else:
                    # Copy other groups unchanged
                    demo_in.copy(key, demo_out)

    print(f"\n✅ Image dimension transposition complete!")
    print(f"Images converted from (T, 3, 128, 128) to (T, 128, 128, 3)")
    print(f"Input:  {input_path}")
    print(f"Output: {output_path}")


def verify_image_dimensions(file_path):
    """
    Verify the image dimensions in the HDF5 file.
    
    Args:
        file_path (str or Path): Path to HDF5 file to verify
    """
    with h5py.File(file_path, "r") as f:
        f_data = f["data"]
        demo_key = list(f_data.keys())[0]
        obs = f_data[demo_key]["obs"]
        
        print(f"\nVerifying image dimensions in {file_path}:")
        print("-" * 60)
        
        for key in obs.keys():
            data = obs[key]
            if isinstance(data, h5py.Dataset):
                shape = data.shape
                print(f"{key:25s}: {shape}")
                
                # Check if it's an image dataset
                if len(shape) == 4:
                    if shape[-1] == 3:
                        print(f"  ✓ Correct format: (T, H, W, C)")
                    elif shape[1] == 3:
                        print(f"  ✗ Needs transposition: (T, C, H, W)")


#####################################################################################################################################################################################
file_original = "dataset.hdf5"
inspect_right_arm(file_original)
trim_first_n_steps(input_file=file_original, output_file="cut_first_steps_dataset.hdf5", n_steps=5)
inspect_right_arm(file="cut_first_steps_dataset.hdf5")
filter_initial_static_states(input_file="cut_first_steps_dataset.hdf5", output_file="dataset_tresholded.hdf5", threshold=0.01)
inspect_right_arm(file="dataset_tresholded.hdf5")
normalize_gripper_in_file(input_path="dataset_tresholded.hdf5", output_path="dataset_tresholded_gripper_normed.hdf5")
inspect_right_arm(file="dataset_tresholded_gripper_normed.hdf5")
modify_rewards_and_create_dones(input_path="dataset_tresholded_gripper_normed.hdf5", output_path="dataset_tresholded_gripper_normed_reward_dones.hdf5")
inspect_right_arm(file="dataset_tresholded_gripper_normed_reward_dones.hdf5")
truncate_demos_at_k_dones(input_path="dataset_tresholded_gripper_normed_reward_dones.hdf5", output_path="dataset_tresholded_gripper_normed_reward_dones_cut_end.hdf5", k=1)
inspect_right_arm(file="dataset_tresholded_gripper_normed_reward_dones_cut_end.hdf5")
transpose_image_dimensions(input_path="dataset_tresholded_gripper_normed_reward_dones_cut_end.hdf5", output_path="dataset_tresholded_gripper_normed_reward_dones_cut_end_image_transpose.hdf5")
inspect_right_arm(file="dataset_tresholded_gripper_normed_reward_dones_cut_end_image_transpose.hdf5")
verify_image_dimensions(file_path="dataset_tresholded_gripper_normed_reward_dones_cut_end_image_transpose.hdf5")
check_demos_with_reward_one(file_path="dataset_tresholded_gripper_normed_reward_dones_cut_end_image_transpose.hdf5")


