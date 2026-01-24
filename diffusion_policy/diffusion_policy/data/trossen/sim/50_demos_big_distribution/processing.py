import h5py
import numpy as np
import os
from pathlib import Path
import json

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
GRIPPER_INDICES = [7, 15]


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

        # Concatenate into full 8-D right arm observation
        right_arm_obs = np.concatenate([robot0_eef_pos, robot0_eef_quat, robot0_gripper_qpos], axis=1)  # (T, 8)

        # Right arm action: last 8 dimensions of action vector (assuming 16-D total)
        if action.shape[1] == 8:
            # For backward compatibility: if only 8-D, assume it's already right arm only
            right_arm_action = action
        elif action.shape[1] == 16:
            right_arm_action = action[:, 8:]  # last 8 = right arm
        else:
            raise ValueError(f"Unexpected action dimension: {action.shape[1]}. Expected 8 or 16.")

        T = right_arm_obs.shape[0]
        steps = range(T)

        # --- Plot all 8 dimensions: pos (3), quat (4), gripper (1) ---
        dim_names = ["EEF X", "EEF Y", "EEF Z", "QX", "QY", "QZ", "QW", "Gripper"]

        fig, axes = plt.subplots(8, 1, figsize=(12, 16), sharex=True)

        for i in range(8):
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



def normalize_gripper_in_file(input_path, output_path):
    """
    Normalize only the gripper dimensions (indices 3 and 7) of the 'actions' dataset 
    in every demo from physical range [0, 0.044] to [-1, 1].
    
    Steps:
      1. Clamp gripper values to [0, 0.044] (handles noise/outliers)
      2. Apply linear map: x ↦ 2*(x / 0.044) - 1  →  [-1, 1]
    All other data (obs, rewards, non-gripper actions, etc.) are copied unchanged.

    Args:
        input_path (str or Path): Path to input HDF5 dataset
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
            actions = demo_in["actions"][:]  # Shape: (T, 8)
            actions_norm = actions.copy()    # Do not modify original

            # Extract gripper dims
            grippers = actions_norm[:, GRIPPER_INDICES]

            # Clamp to physical limits
            grippers = np.clip(grippers, GRIPPER_MIN, GRIPPER_MAX)

            # Normalize to [-1, 1]
            grippers_norm = 2 * (grippers - GRIPPER_MIN) / (GRIPPER_MAX - GRIPPER_MIN) - 1

            # Write back
            actions_norm[:, GRIPPER_INDICES] = grippers_norm

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

import h5py
import numpy as np
import matplotlib.pyplot as plt
import json
import os

def plot_right_arm_obs_action_difference(file, output_json="obs_action_diff_stats.json"):
    with h5py.File(file, "r") as f:
        f_data = f["data"]
        demo_keys = sorted(f_data.keys())  # e.g., ['demo_0', 'demo_1', ..., 'demo_49']

        # --- Plot only the first demo ---
        first_demo_key = demo_keys[0]
        f_demo_0 = f_data[first_demo_key]

        action = f_demo_0["actions"][:]
        obs = f_demo_0["obs"]

        robot0_eef_pos = obs["robot0_eef_pos"][:, 3:]
        robot0_eef_quat = obs["robot0_eef_quat"][:, 4:]
        robot0_gripper_qpos = obs["robot0_gripper_qpos"][:, 1:]

        right_arm_obs = np.concatenate([robot0_eef_pos, robot0_eef_quat, robot0_gripper_qpos], axis=1)

        if action.shape[1] == 8:
            right_arm_action = action
        elif action.shape[1] == 16:
            right_arm_action = action[:, 8:]
        else:
            raise ValueError(f"Unexpected action dimension: {action.shape[1]}. Expected 8 or 16.")

        diff_plot = right_arm_obs - right_arm_action

        # Plot
        T = diff_plot.shape[0]
        steps = range(T)
        dim_names = ["EEF X", "EEF Y", "EEF Z", "QX", "QY", "QZ", "QW", "Gripper"]

        fig, axes = plt.subplots(8, 1, figsize=(12, 16), sharex=True)
        for i in range(8):
            axes[i].plot(steps, diff_plot[:, i], label=f'{dim_names[i]} (obs - action)', color='purple')
            axes[i].axhline(0, color='black', linewidth=0.5, linestyle='--')
            axes[i].set_ylabel(dim_names[i])
            axes[i].legend(loc='upper right')
            axes[i].grid(True)

        axes[-1].set_xlabel("Step")
        plt.suptitle("Right Arm: Observation Minus Action (First Demo Only)")
        plt.tight_layout(rect=[0, 0, 1, 0.97])
        plt.show()

        # --- Compute global min/max across ALL demos ---
        print("\nComputing global min and max of (Observation - Action) across all demos...")
        global_min = np.full(8, np.inf)
        global_max = np.full(8, -np.inf)

        for demo_key in demo_keys:
            demo = f_data[demo_key]
            action = demo["actions"][:]
            obs = demo["obs"]

            # Extract right arm observation
            robot0_eef_pos = obs["robot0_eef_pos"][:, 3:]
            robot0_eef_quat = obs["robot0_eef_quat"][:, 4:]
            robot0_gripper_qpos = obs["robot0_gripper_qpos"][:, 1:]

            right_arm_obs = np.concatenate([robot0_eef_pos, robot0_eef_quat, robot0_gripper_qpos], axis=1)

            # Extract right arm action
            if action.shape[1] == 8:
                right_arm_action = action
            elif action.shape[1] == 16:
                right_arm_action = action[:, 8:]
            else:
                raise ValueError(f"Unexpected action dimension in {demo_key}: {action.shape[1]}")

            diff = right_arm_obs - right_arm_action  # (T, 8)

            # Update global min/max per dimension
            global_min = np.minimum(global_min, np.min(diff, axis=0))
            global_max = np.maximum(global_max, np.max(diff, axis=0))

        # Prepare dictionary for JSON
        stats = {}
        for i, name in enumerate(dim_names):
            stats[name] = {
                "min": float(global_min[i]),
                "max": float(global_max[i])
            }

        # Save to JSON
        with open(output_json, 'w') as f_out:
            json.dump(stats, f_out, indent=4)

        print(f"\nGlobal min/max saved to: {os.path.abspath(output_json)}")

        # Also print to console
        print("\nGlobal Min and Max of (Observation - Action) across ALL demos:")
        print("-" * 65)
        for name in dim_names:
            print(f"{name:8}: min = {stats[name]['min']: .6f}, max = {stats[name]['max']: .6f}")

import h5py
import numpy as np
from pathlib import Path
from scipy.spatial.transform import Rotation as R
import matplotlib.pyplot as plt


def convert_quat_to_angle_axis_in_dataset(input_path, output_path):
    """
    Convert quaternion representations in actions from [w, x, y, z] to angle-axis [rx, ry, rz].
    
    Input actions format: [pos_left(3), quat_left(4), gripper_left(1), pos_right(3), quat_right(4), gripper_right(1)] = 16D
    Output actions format: [pos_left(3), aa_left(3), gripper_left(1), pos_right(3), aa_right(3), gripper_right(1)] = 14D
    
    Quaternion format is [w, x, y, z] as used in the recording code.
    
    Args:
        input_path (str or Path): Path to input HDF5 file with quaternion actions
        output_path (str or Path): Path to save converted dataset
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
            print(f"Converting {demo_name} from quaternion to angle-axis...")
            
            demo_in = data_in[demo_name]
            demo_out = data_out.create_group(demo_name)

            # Process actions: convert quaternions to angle-axis
            actions_quat = demo_in["actions"][:]  # Shape: (T, 16)
            T = actions_quat.shape[0]
            
            # Initialize output actions (T, 14)
            actions_aa = np.zeros((T, 14), dtype=np.float32)
            
            for t in range(T):
                # Left arm: pos (0:3), quat (3:7), gripper (7)
                pos_left = actions_quat[t, 0:3]
                quat_left = actions_quat[t, 3:7]  # [w, x, y, z]
                gripper_left = actions_quat[t, 7]
                
                # Right arm: pos (8:11), quat (11:15), gripper (15)
                pos_right = actions_quat[t, 8:11]
                quat_right = actions_quat[t, 11:15]  # [w, x, y, z]
                gripper_right = actions_quat[t, 15]
                
                # Convert quaternions to angle-axis
                # scipy expects [x, y, z, w] format, so we need to reorder
                quat_left_scipy = [quat_left[1], quat_left[2], quat_left[3], quat_left[0]]
                quat_right_scipy = [quat_right[1], quat_right[2], quat_right[3], quat_right[0]]
                
                rot_left = R.from_quat(quat_left_scipy)
                rot_right = R.from_quat(quat_right_scipy)
                
                aa_left = rot_left.as_rotvec()  # (3,) angle-axis
                aa_right = rot_right.as_rotvec()  # (3,) angle-axis
                
                # Assemble 14D action: [pos_left(3), aa_left(3), gripper_left(1), pos_right(3), aa_right(3), gripper_right(1)]
                actions_aa[t, 0:3] = pos_left
                actions_aa[t, 3:6] = aa_left
                actions_aa[t, 6] = gripper_left
                actions_aa[t, 7:10] = pos_right
                actions_aa[t, 10:13] = aa_right
                actions_aa[t, 13] = gripper_right
            
            # Save converted actions
            demo_out.create_dataset("actions", data=actions_aa, compression="gzip")
            
            # Copy all other datasets and groups unchanged
            for key in demo_in.keys():
                if key == "actions":
                    continue
                if isinstance(demo_in[key], h5py.Dataset):
                    demo_out.create_dataset(key, data=demo_in[key][:], compression="gzip")
                else:
                    # Copy group (e.g., 'obs')
                    demo_in.copy(key, demo_out)

    print(f"✅ Quaternion to angle-axis conversion complete!")
    print(f"Actions converted from 16D (quat) to 14D (angle-axis)")
    print(f"Input:  {input_path}")
    print(f"Output: {output_path}")


def inspect_right_arm_angle_axis(file):
    """
    Inspect right arm data when actions are in angle-axis format (14D).
    
    Action format: [pos_left(3), aa_left(3), gripper_left(1), pos_right(3), aa_right(3), gripper_right(1)]
    Observation format remains unchanged with quaternions.
    
    Plots:
    1. Position (3D) - obs vs action
    2. Angle-axis (3D) - converted obs vs action
    3. Gripper (1D) - obs vs action
    """
    with h5py.File(file, "r") as f:
        print("Top-level keys:", list(f.keys()))

        f_data = f["data"]
        print("f_data keys:", list(f_data.keys()))
        print("The number of demos is:", len(f_data))

        # Take first demo
        demo_key = list(f_data.keys())[0]
        f_demo_0 = f_data[demo_key]
        print(f"Processing demo: {demo_key}")
        print("f_demo_0 keys:", list(f_demo_0.keys()))

        # Actions (14D: angle-axis format)
        action = f_demo_0["actions"][:]
        print("Action size:", action.shape)
        print("First action:", action[0])
        print("Last action:", action[-1])

        obs = f_demo_0["obs"]
        print("obs keys:", list(obs.keys()))

        # Extract right arm observation (still in quaternion format)
        robot0_eef_pos = obs["robot0_eef_pos"][:, 3:]       # shape (T, 3)
        robot0_eef_quat = obs["robot0_eef_quat"][:, 4:]     # shape (T, 4) [w, x, y, z]
        robot0_gripper_qpos = obs["robot0_gripper_qpos"][:, 1:]  # shape (T, 1)

        # Convert observation quaternions to angle-axis for comparison
        T = robot0_eef_quat.shape[0]
        robot0_eef_aa = np.zeros((T, 3), dtype=np.float32)
        
        for t in range(T):
            quat = robot0_eef_quat[t]  # [w, x, y, z]
            # Convert to scipy format [x, y, z, w]
            quat_scipy = [quat[1], quat[2], quat[3], quat[0]]
            rot = R.from_quat(quat_scipy)
            robot0_eef_aa[t] = rot.as_rotvec()

        # Extract right arm action (14D format)
        if action.shape[1] == 14:
            # Right arm: pos (7:10), aa (10:13), gripper (13)
            right_arm_action_pos = action[:, 7:10]
            right_arm_action_aa = action[:, 10:13]
            right_arm_action_gripper = action[:, 13:14]
        else:
            raise ValueError(f"Unexpected action dimension: {action.shape[1]}. Expected 14 for angle-axis format.")

        steps = range(T)

        # --- Plot 1: Position (3D) ---
        pos_names = ["EEF X", "EEF Y", "EEF Z"]
        fig_pos, axes_pos = plt.subplots(3, 1, figsize=(12, 8), sharex=True)

        for i in range(3):
            axes_pos[i].plot(steps, robot0_eef_pos[:, i], label=f'{pos_names[i]} (obs)', color='blue')
            axes_pos[i].plot(steps, right_arm_action_pos[:, i], label=f'{pos_names[i]} (action)', color='red', linestyle='--')
            axes_pos[i].set_ylabel(pos_names[i])
            axes_pos[i].legend(loc='upper right')
            axes_pos[i].grid(True)

        axes_pos[-1].set_xlabel("Step")
        plt.suptitle("Right Arm: Position (Obs vs Action)")
        plt.tight_layout(rect=[0, 0, 1, 0.97])
        plt.show()

        # --- Plot 2: Angle-Axis (3D) ---
        aa_names = ["Rotation X", "Rotation Y", "Rotation Z"]
        fig_aa, axes_aa = plt.subplots(3, 1, figsize=(12, 8), sharex=True)

        for i in range(3):
            axes_aa[i].plot(steps, robot0_eef_aa[:, i], label=f'{aa_names[i]} (obs)', color='green')
            axes_aa[i].plot(steps, right_arm_action_aa[:, i], label=f'{aa_names[i]} (action)', color='orange', linestyle='--')
            axes_aa[i].set_ylabel(aa_names[i])
            axes_aa[i].legend(loc='upper right')
            axes_aa[i].grid(True)

        axes_aa[-1].set_xlabel("Step")
        plt.suptitle("Right Arm: Angle-Axis Rotation (Obs vs Action)")
        plt.tight_layout(rect=[0, 0, 1, 0.97])
        plt.show()

        # --- Plot 3: Gripper (1D) ---
        fig_grip, ax_grip = plt.subplots(1, 1, figsize=(12, 4))
        ax_grip.plot(steps, robot0_gripper_qpos[:, 0], label='Gripper (obs)', color='purple', linewidth=1.5)
        ax_grip.plot(steps, right_arm_action_gripper[:, 0], label='Gripper (action)', color='brown', linestyle='--', linewidth=1.5)
        ax_grip.set_xlabel("Step")
        ax_grip.set_ylabel("Gripper Position")
        ax_grip.legend(loc='upper right')
        ax_grip.grid(True)
        plt.suptitle("Right Arm: Gripper (Obs vs Action)")
        plt.tight_layout()
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

def save_camera_videos_from_demo(
    hdf5_path,
    demo_name="demo_0",  # or any specific demo
    output_dir="videos",
    fps=30,
    camera_keys=None
):
    """
    Extract and save videos for each camera view in a single demonstration.
    
    Args:
        hdf5_path (str/Path): Path to processed HDF5 file (with images in (T, H, W, 3) format)
        demo_name (str): Name of demo group (e.g., 'demo_0')
        output_dir (str): Directory to save .mp4 videos
        fps (int): Frames per second for output video
        camera_keys (list): List of image observation keys. If None, auto-detect RGB image datasets.
    """
    hdf5_path = Path(hdf5_path)
    output_dir = Path(output_dir)
    output_dir.mkdir(parents=True, exist_ok=True)

    with h5py.File(hdf5_path, "r") as f:
        demo = f["data"][demo_name]
        obs = demo["obs"]

        # Auto-detect image keys if not provided
        if camera_keys is None:
            camera_keys = []
            for key in obs.keys():
                data = obs[key]
                if isinstance(data, h5py.Dataset) and len(data.shape) == 4 and data.shape[-1] == 3:
                    camera_keys.append(key)
            print(f"Auto-detected camera keys: {camera_keys}")

        for cam_key in camera_keys:
            print(f"Processing {cam_key}...")
            images = obs[cam_key][:]  # Shape: (T, H, W, 3), uint8 or float?

            # Ensure dtype is uint8 in [0, 255]
            if images.dtype != np.uint8:
                if images.max() <= 1.0:
                    images = (images * 255).astype(np.uint8)
                else:
                    images = images.astype(np.uint8)

            T, H, W, C = images.shape
            assert C == 3, f"Expected 3 channels, got {C}"

            # Define video writer
            video_path = output_dir / f"{demo_name}_{cam_key}.mp4"
            fourcc = cv2.VideoWriter_fourcc(*'mp4v')
            out = cv2.VideoWriter(str(video_path), fourcc, fps, (W, H))

            for t in range(T):
                frame = images[t]
                # OpenCV expects BGR, but your data is likely RGB → convert
                frame_bgr = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
                out.write(frame_bgr)

            out.release()
            print(f"✅ Saved video: {video_path}")

def function_make_video(input_file = "dataset_shifted_norm_gripper_tresholded_wr_end_cut_angle_axis_image_transposed.hdf5") : 
    
    # Optional: specify exact camera keys if known
    CAMERA_KEYS = [
        "cam_high_image",
        "cam_low_image",
        "cam_left_wrist_image",
        "cam_right_wrist_image"
    ]

    # Save videos for all demos (or just one)
    with h5py.File(input_file, "r") as f:
        demo_names = list(f["data"].keys())

    for demo in demo_names[1:2]:  # Change to `demo_names` to process all
        save_camera_videos_from_demo(
            hdf5_path=input_file,
            demo_name=demo,
            output_dir="trajectory_videos",
            fps=30,
            camera_keys=CAMERA_KEYS  # or None to auto-detect
        )

#####################################################################################################################################################################################
file_original = "dataset_1.hdf5"
inspect_right_arm(file_original)
filter_initial_static_states(input_file="dataset_1.hdf5", output_file="dataset_tresholded.hdf5", threshold=0.01)
inspect_right_arm(file="dataset_tresholded.hdf5")
normalize_gripper_in_file(input_path="dataset_tresholded.hdf5", output_path="dataset_norm_gripper_tresholded.hdf5")
inspect_right_arm(file="dataset_norm_gripper_tresholded.hdf5")
modify_rewards_and_create_dones(input_path="dataset_norm_gripper_tresholded.hdf5", output_path="dataset_norm_gripper_tresholded_wr.hdf5")
inspect_right_arm(file="dataset_norm_gripper_tresholded_wr.hdf5")
truncate_demos_at_k_dones(input_path= "dataset_norm_gripper_tresholded_wr.hdf5", output_path = "dataset_norm_gripper_tresholded_wr_end_cut.hdf5", k = 1)
inspect_right_arm(file="dataset_norm_gripper_tresholded_wr_end_cut.hdf5")
convert_quat_to_angle_axis_in_dataset(input_path="dataset_norm_gripper_tresholded_wr_end_cut.hdf5", output_path="dataset_norm_gripper_tresholded_wr_end_cut_angle_axis.hdf5")
inspect_right_arm_angle_axis(file="dataset_norm_gripper_tresholded_wr_end_cut_angle_axis.hdf5")
transpose_image_dimensions(input_path="dataset_norm_gripper_tresholded_wr_end_cut_angle_axis.hdf5", output_path="dataset_norm_gripper_tresholded_wr_end_cut_angle_axis_image_transposed.hdf5")
inspect_right_arm_angle_axis(file="dataset_norm_gripper_tresholded_wr_end_cut_angle_axis_image_transposed.hdf5")
verify_image_dimensions(file_path="dataset_norm_gripper_tresholded_wr_end_cut_angle_axis_image_transposed.hdf5")

function_make_video(input_file="dataset_norm_gripper_tresholded_wr_end_cut_angle_axis_image_transposed.hdf5")