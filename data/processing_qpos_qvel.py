#!/usr/bin/env python3
"""
filter_dataset_by_state_change.py

Outputs a new HDF5 file with preserved data metadata (including env_args).
"""

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


def filter_demo_states(qpos_full, threshold):
    """
    Filter states based on qpos (joint positions) changes.
    Args:
        qpos_full: array of shape (T, 16) - joint positions
        threshold: minimum norm change to keep a state
    """
    states = qpos_full[:, :16]
    T = states.shape[0]
    if T == 0:
        return []

    kept_idx = [0]
    last_state = states[0].copy()

    for t in range(1, T):
        if np.linalg.norm(states[t] - last_state) > threshold:
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
        for k, v in f_data_in.attrs.items():  # PRESERVE 'env_args' here
            f_data_out.attrs[k] = v

        demo_names = list(f_data_in.keys())
        print("Found demos:", len(demo_names))

        for demo_name in demo_names:
            grp_in = f_data_in[demo_name]
            print(f"\nProcessing demo: {demo_name}")

            obs_in = grp_in["obs"]
            qpos_in = obs_in["qpos"][:]
            T, _ = qpos_in.shape

            kept_idx = filter_demo_states(qpos_in, threshold)
            if len(kept_idx) == 0:
                print(f"  Warning: no indices kept for {demo_name}. Skipping.")
                continue

            grp_out = f_data_out.create_group(demo_name)

            # ----- Rewards -----
            if "rewards" in grp_in:
                rewards_in = grp_in["rewards"][:]
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

            # ----- Actions based on state deltas -----
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


def shift_actions_by_k(input_path, output_path, k=1):
    """
    Replace the action at time t by the action at time t+k.
    For the last k steps where t+k exceeds the total length, 
    the last valid action is repeated.
    """
    
    input_path = Path(input_path)
    output_path = Path(output_path)

    with h5py.File(input_path, "r") as fin, h5py.File(output_path, "w") as fout:

        # Copy attributes from root data group
        fout.create_group("data")
        for key, val in fin["data"].attrs.items():
            fout["data"].attrs[key] = val

        data_in = fin["data"]
        data_out = fout["data"]

        for demo_name in data_in.keys():
            print(f"Processing {demo_name}...")

            demo_in = data_in[demo_name]
            demo_out = data_out.create_group(demo_name)

            # Get the original actions
            original_actions = demo_in["actions"][:]
            T = original_actions.shape[0]

            # Create shifted actions
            shifted_actions = np.zeros_like(original_actions)
            
            for t in range(T):
                if t + k < T:
                    # Use action at t+k
                    shifted_actions[t] = original_actions[t + k]
                else:
                    shifted_actions[t] = original_actions[T - 1]  # Use the very last action

            # --- Actions (shifted) ---
            demo_out.create_dataset("actions", data=shifted_actions, compression="gzip")

            # --- Rewards (unchanged) ---
            rewards_ds = demo_in["rewards"][:]
            demo_out.create_dataset("rewards", data=rewards_ds, compression="gzip")

            # --- Dones (unchanged) ---
            if "dones" in demo_in:
                dones_ds = demo_in["dones"][:]
                demo_out.create_dataset("dones", data=dones_ds, compression="gzip")

            # --- Observations (unchanged) ---
            obs_out = demo_out.create_group("obs")
            obs_in = demo_in["obs"]

            for obs_key in obs_in.keys():
                arr = obs_in[obs_key][:]   # copy all observations without modification
                obs_out.create_dataset(obs_key, data=arr, compression="gzip")

        print(f"\n🎉 Finished! Saved action-shifted dataset to:\n{output_path}")
        print(f"Actions shifted by k={k} steps (action at time t now contains action from time t+k)")
        return output_path
    



def normalize_actions_jointwise(input_path, output_path):
    joint_mins = np.array([-np.pi, 0, 0, -np.pi/2, -np.pi/2, -np.pi,
                           0, 0, -np.pi, 0, 0, -np.pi/2, -np.pi/2, -np.pi,
                           0, 0])
    joint_maxs = np.array([np.pi, np.pi, 2.36, np.pi/2, np.pi/2, np.pi,
                           0.04, 0.04, np.pi, np.pi, 2.36, np.pi/2, np.pi/2, np.pi,
                           0.04, 0.04])

    with h5py.File(input_path, "r") as fin, h5py.File(output_path, "w") as fout:
        
        # 🔹 PRESERVE file-level and 'data' attributes, including env_args
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

            actions = demo_in["actions"][:]
            actions_norm = 2 * (actions - joint_mins) / (joint_maxs - joint_mins) - 1
            demo_out.create_dataset("actions", data=actions_norm, compression="gzip")

            if "rewards" in demo_in:
                demo_out.create_dataset("rewards", data=demo_in["rewards"][:], compression="gzip")

            if "dones" in demo_in:
                demo_out.create_dataset("dones", data=demo_in["dones"][:], compression="gzip")

            obs_in = demo_in["obs"]
            obs_out = demo_out.create_group("obs")
            for obs_key in obs_in.keys():
                obs_out.create_dataset(obs_key, data=obs_in[obs_key][:], compression="gzip")

        print(f"\n🎉 Finished! Saved normalized dataset to:\n{output_path}")
        return output_path


def cut_first_steps(input_path, output_path, cut_first_n=15):
    """
    Remove the first `cut_first_n` steps from each demo in the HDF5 dataset.
    """

    input_path = Path(input_path)
    output_path = Path(output_path)
    output_path.parent.mkdir(parents=True, exist_ok=True)

    with h5py.File(input_path, "r") as fin, h5py.File(output_path, "w") as fout:

        # Copy data group and its attributes
        fout.create_group("data")
        for key, val in fin["data"].attrs.items():
            fout["data"].attrs[key] = val

        data_in = fin["data"]
        data_out = fout["data"]

        for demo_name in data_in.keys():
            print(f"Processing {demo_name}...")

            demo_in = data_in[demo_name]
            demo_out = data_out.create_group(demo_name)

            T = demo_in["actions"].shape[0]

            # Compute start index after cutting
            start_idx = min(cut_first_n, T - 1)
            idx = range(start_idx, T)

            # --- Actions ---
            demo_out.create_dataset("actions", data=demo_in["actions"][idx], compression="gzip")

            # --- Rewards ---
            demo_out.create_dataset("rewards", data=demo_in["rewards"][idx], compression="gzip")

            # --- Dones ---
            if "dones" in demo_in:
                demo_out.create_dataset("dones", data=demo_in["dones"][idx], compression="gzip")

            # --- Observations ---
            obs_out = demo_out.create_group("obs")
            obs_in = demo_in["obs"]
            for obs_key in obs_in.keys():
                obs_out.create_dataset(obs_key, data=obs_in[obs_key][idx], compression="gzip")

        print(f"\n🎉 Finished! Saved dataset with first {cut_first_n} steps removed to:\n{output_path}")
        return output_path
    

def clip_action_indices(input_path, output_path):
    """
    Clips the actions in the HDF5 file to the joint limits specified in normalize_actions_jointwise.
    Only modifies the 'actions' dataset; other data is copied unchanged.
    """
    joint_mins = np.array([
        -np.pi, 0, 0, -np.pi/2, -np.pi/2, -np.pi,
        0, 0, -np.pi, 0, 0, -np.pi/2, -np.pi/2, -np.pi,
        0, 0
    ])
    joint_maxs = np.array([
        np.pi, np.pi, 2.36, np.pi/2, np.pi/2, np.pi,
        0.04, 0.04, np.pi, np.pi, 2.36, np.pi/2, np.pi/2, np.pi,
        0.04, 0.04
    ])

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

            actions = demo_in["actions"][:]
            actions_clipped = np.clip(actions, joint_mins, joint_maxs)
            demo_out.create_dataset("actions", data=actions_clipped, compression="gzip")

            if "rewards" in demo_in:
                demo_out.create_dataset("rewards", data=demo_in["rewards"][:], compression="gzip")

            if "dones" in demo_in:
                demo_out.create_dataset("dones", data=demo_in["dones"][:], compression="gzip")

            obs_in = demo_in["obs"]
            obs_out = demo_out.create_group("obs")
            for obs_key in obs_in.keys():
                obs_out.create_dataset(obs_key, data=obs_in[obs_key][:], compression="gzip")

        print(f"\n🎉 Finished! Saved clipped dataset to:\n{output_path}")
        return output_path
    
def inspect_right_arm(file):
    with h5py.File(file, "r") as f:
        print("Top-level keys:", list(f.keys()))

        f_data = f["data"]
        print("f_data keys:", list(f_data.keys()))
        print("The number of demos is:", len(f_data))

        # Take first demo
        f_demo_0 = f_data["demo_1"]
        print("f_demo_0 keys:", list(f_demo_0.keys()))

        # Actions
        action = f_demo_0["actions"]
        print("Action size:", action.shape)
        print("First action:", action[0])
        print("Last action:", action[-1])

        # Observations
        obs = f_demo_0["obs"]
        print("obs keys:", list(obs.keys()))

        qpos = obs["qpos"]
        print("Qpos shape:", qpos.shape)
        print("First qpos state:", qpos[0])

        # Select only the right arm joint positions (first 7 of the 16 positions)
        right_arm_state = qpos[:, 8:16]    # shape: (num_steps, 7)
        right_arm_action = action[:, -8:]  # shape: (num_steps, 7)

        num_joints = right_arm_state.shape[1]
        steps = range(right_arm_state.shape[0])

        # Plot each joint in a separate subplot
        fig, axes = plt.subplots(num_joints, 1, figsize=(12, 2*num_joints), sharex=True)

        for j in range(num_joints):
            print(f"Joint {j}: min={np.min(right_arm_state[:, j]):.6f}, max={np.max(right_arm_state[:, j]):.6f}")
            axes[j].plot(steps, right_arm_state[:, j], label=f'Joint {j+1} Position', color='blue')
            axes[j].plot(steps, right_arm_action[:, j], label=f'Joint {j+1} Action', linestyle='--', color='red')
            axes[j].set_ylabel("Value")
            axes[j].legend()
            axes[j].grid(True)

        axes[-1].set_xlabel("Step")
        plt.suptitle("Right Arm Joint Positions and Actions")
        plt.tight_layout(rect=[0, 0, 1, 0.96])
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


def inspect_right_arm_original_dataset(file):
    with h5py.File(file, "r") as f:
        print("Top-level keys:", list(f.keys()))

        f_data = f["data"]
        print("f_data keys:", list(f_data.keys()))
        print("The number of demos is:", len(f_data))

        # Take first demo
        f_demo_0 = f_data["demo_1"]
        print("f_demo_0 keys:", list(f_demo_0.keys()))

        # Actions
        action = f_demo_0["actions"]
        print("Action size:", action.shape)
        print("First action:", action[0])
        print("Last action:", action[-1])

        # Observations
        obs = f_demo_0["obs"]
        print("obs keys:", list(obs.keys()))

        qpos = obs["qpos"]
        print("Qpos shape:", qpos.shape)
        print("First qpos state:", qpos[0])

        # Select only the right arm joint positions (first 7 of the 16 positions)
        right_arm_state = qpos[:, 8:15]    # shape: (num_steps, 7)
        right_arm_action = action[:, 7:14]  # shape: (num_steps, 7)

        num_joints = right_arm_state.shape[1]
        steps = range(right_arm_state.shape[0])

        # Plot each joint in a separate subplot
        fig, axes = plt.subplots(num_joints, 1, figsize=(12, 2*num_joints), sharex=True)

        for j in range(num_joints):
            print(f"Joint {j}: min={np.min(right_arm_state[:, j]):.6f}, max={np.max(right_arm_state[:, j]):.6f}")
            axes[j].plot(steps, right_arm_state[:, j], label=f'Joint {j+1} Position', color='blue')
            axes[j].plot(steps, right_arm_action[:, j], label=f'Joint {j+1} Action', linestyle='--', color='red')
            axes[j].set_ylabel("Value")
            axes[j].legend()
            axes[j].grid(True)

        axes[-1].set_xlabel("Step")
        plt.suptitle("Right Arm Joint Positions and Actions")
        plt.tight_layout(rect=[0, 0, 1, 0.96])
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


def save_first_demo(input_path, output_path, keep_stride=None):
    """
    Copies ONLY the first demo from the HDF5 dataset.
    Optionally downsamples using keep_stride.
    """

    input_path = Path(input_path)
    output_path = Path(output_path)
    output_path.parent.mkdir(parents=True, exist_ok=True)

    with h5py.File(input_path, "r") as fin, h5py.File(output_path, "w") as fout:

        # create "data" group
        fout.create_group("data")

        # copy global attributes
        for k, v in fin["data"].attrs.items():
            fout["data"].attrs[k] = v

        data_in = fin["data"]
        data_out = fout["data"]

        # ---- GET FIRST DEMO NAME ----
        demo_names = list(data_in.keys())
        if len(demo_names) == 0:
            raise ValueError("No demos found in input dataset!")

        first_demo = demo_names[0]
        print(f"Saving ONLY the first demo: {first_demo}")

        demo_in = data_in[first_demo]
        demo_out = data_out.create_group(first_demo)

        # ---- ACTIONS ----
        actions = demo_in["actions"][:]
        actions_ds = actions[::keep_stride] if keep_stride else actions
        demo_out.create_dataset("actions", data=actions_ds, compression="gzip")

        # ---- REWARDS ----
        rewards = demo_in["rewards"][:]
        rewards_ds = rewards[::keep_stride] if keep_stride else rewards
        demo_out.create_dataset("rewards", data=rewards_ds, compression="gzip")

        # ---- DONES ----
        if "dones" in demo_in:
            dones = demo_in["dones"][:]
            dones_ds = dones[::keep_stride] if keep_stride else dones
            demo_out.create_dataset("dones", data=dones_ds, compression="gzip")

        # ---- OBS ----
        obs_in = demo_in["obs"]
        obs_out = demo_out.create_group("obs")

        for obs_key in obs_in.keys():
            arr = obs_in[obs_key][:]

            arr_ds = arr[::keep_stride] if keep_stride else arr

            obs_out.create_dataset(
                obs_key,
                data=arr_ds,
                compression="gzip"
            )

    print(f"\n🎉 Saved only the first demo to:\n{output_path}")
    return output_path



def resize_images(input_path, output_path, new_size=(96, 96)):
    """
    Resize all camera images in the HDF5 dataset from their current size to new_size.
    
    Args:
        input_path: Path to input HDF5 file
        output_path: Path to output HDF5 file
        new_size: Tuple of (height, width) for the new image size
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

        # List of camera keys that need resizing
        camera_keys = ['cam_high_image', 'cam_left_wrist_image', 
                       'cam_low_image', 'cam_right_wrist_image']

        for demo_name in data_in.keys():
            print(f"Processing {demo_name}...")
            
            demo_in = data_in[demo_name]
            demo_out = data_out.create_group(demo_name)

            # --- Actions (unchanged) ---
            demo_out.create_dataset("actions", data=demo_in["actions"][:], compression="gzip")

            # --- Rewards (unchanged) ---
            demo_out.create_dataset("rewards", data=demo_in["rewards"][:], compression="gzip")

            # --- Dones (unchanged) ---
            if "dones" in demo_in:
                demo_out.create_dataset("dones", data=demo_in["dones"][:], compression="gzip")

            # --- Observations ---
            obs_in = demo_in["obs"]
            obs_out = demo_out.create_group("obs")

            for obs_key in obs_in.keys():
                if obs_key in camera_keys:
                    # Resize camera images
                    images = obs_in[obs_key][:]  # Shape: (T, C, H, W)
                    T, C, H, W = images.shape
                    
                    print(f"  Resizing {obs_key}: ({T}, {C}, {H}, {W}) -> ({T}, {C}, {new_size[0]}, {new_size[1]})")
                    
                    # Initialize resized array
                    resized_images = np.zeros((T, C, new_size[0], new_size[1]), dtype=images.dtype)
                    
                    for t in range(T):
                        # Convert from (C, H, W) to (H, W, C) for cv2
                        img = images[t].transpose(1, 2, 0)
                        
                        # Resize using cv2
                        img_resized = cv2.resize(img, (new_size[1], new_size[0]), 
                                                interpolation=cv2.INTER_LINEAR)
                        
                        # Convert back to (C, H, W)
                        resized_images[t] = img_resized.transpose(2, 0, 1)
                    
                    obs_out.create_dataset(obs_key, data=resized_images, compression="gzip")
                else:
                    # Copy non-image observations unchanged (like 'qpos', 'qvel')
                    obs_out.create_dataset(obs_key, data=obs_in[obs_key][:], compression="gzip")

        print(f"\n🎉 Finished! Saved resized dataset to:\n{output_path}")
        print(f"All camera images resized to {new_size[0]}x{new_size[1]}")
        return output_path
    

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
            rewards_binary = (rewards_original == 4).astype(np.float32)
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
    


def expand_gripper_actions(input_path, output_path):
    """
    Expands 14-dim actions to 16-dim by duplicating the gripper dimension
    for symmetric parts for a 2-arm setup.
    Compatible with new dataset format (qpos/qvel in obs, optional dones).
    """
    input_path = Path(input_path)
    output_path = Path(output_path)
    output_path.parent.mkdir(parents=True, exist_ok=True)

    with h5py.File(input_path, "r") as fin, h5py.File(output_path, "w") as fout:
        # Copy global attributes from "data"
        fout.create_group("data")
        for attr_key, attr_val in fin["data"].attrs.items():
            fout["data"].attrs[attr_key] = attr_val

        data_in = fin["data"]
        data_out = fout["data"]

        for demo_name in data_in.keys():
            print(f"Processing {demo_name}...")
            demo_in = data_in[demo_name]
            demo_out = data_out.create_group(demo_name)

            # Expand actions from 14 → 16
            actions = demo_in["actions"][:]
            actions_expanded = np.zeros((actions.shape[0], 16), dtype=actions.dtype)

            # Left arm
            actions_expanded[:, :6] = actions[:, :6]  # joints
            actions_expanded[:, 6] = actions[:, 6]   # gripper1
            actions_expanded[:, 7] = actions[:, 6]   # gripper2 (duplicated)

            # Right arm
            actions_expanded[:, 8:14] = actions[:, 7:13]  # joints
            actions_expanded[:, 14] = actions[:, 13]      # gripper1
            actions_expanded[:, 15] = actions[:, 13]      # gripper2

            demo_out.create_dataset("actions", data=actions_expanded, compression="gzip")

            # Copy rewards
            demo_out.create_dataset("rewards", data=demo_in["rewards"][:], compression="gzip")

            # Copy dones if present
            if "dones" in demo_in:
                demo_out.create_dataset("dones", data=demo_in["dones"][:], compression="gzip")

            # Copy observations (qpos, qvel, images, etc.)
            obs_in = demo_in["obs"]
            obs_out = demo_out.create_group("obs")
            for obs_key in obs_in.keys():
                obs_out.create_dataset(obs_key, data=obs_in[obs_key][:], compression="gzip")

        print(f"\n🎉 Finished! Saved expanded dataset to:\n{output_path}")
        return output_path

# ...existing code...


if __name__ == "__main__":

    input_file =                f"cube_picking_and_placing/shifting_5/dataset.hdf5"
    output_extend_gripper =     f"cube_picking_and_placing/shifting_5/extended_gripper_dataset.hdf5"
    treshold = 0.05
    output_processed_file =     f"cube_picking_and_placing/shifting_5/extended_gripper_dataset_threshold.hdf5"
    output_with_reward =        F"cube_picking_and_placing/shifting_5/extended_gripper_dataset_wr_threshold.hdf5"
    output_shifted =            f"cube_picking_and_placing/shifting_5/extended_gripper_dataset_wr_threshold_rewards_dones_shifted.hdf5"
    output_gripper_clipped =    f"cube_picking_and_placing/shifting_5/extended_gripper_dataset_wr_threshold_rewards_dones_shifted_gripper_clipped.hdf5"
    output_normed_file =        f"cube_picking_and_placing/shifting_5/extended_gripper_dataset_wr_threshold_rewards_dones_shifted_gripper_clipped_clipped_norm_min_max.hdf5"
    output_cut_file =           f"cube_picking_and_placing/shifting_5/extended_gripper_dataset_wr_threshold_rewards_dones_shifted_gripper_clipped_clipped_norm_min_max_cut.hdf5"

    # inspect_right_arm_original_dataset(input_file)
    # expand_gripper_actions(input_file, output_extend_gripper)
    # inspect_right_arm(output_extend_gripper)
    # process_dataset(output_extend_gripper, output_processed_file, treshold)
    # inspect_right_arm(output_processed_file)
    # modify_rewards_and_create_dones(output_processed_file, output_with_reward)
    # inspect_right_arm(output_with_reward)
    # shift_actions_by_k(output_with_reward, output_shifted, k = 5)
    # inspect_right_arm(output_shifted)
    # clip_action_indices(output_shifted, output_gripper_clipped)
    # inspect_right_arm(output_gripper_clipped)
    # normalize_actions_jointwise(output_gripper_clipped, output_normed_file)
    # inspect_right_arm(output_normed_file)
    # cut_first_steps(output_normed_file, output_cut_file)
    inspect_right_arm(output_cut_file)


