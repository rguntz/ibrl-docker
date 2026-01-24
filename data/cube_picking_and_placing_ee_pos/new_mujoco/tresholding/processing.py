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
GRIPPER_INDICES = [3, 7]

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

        # --- Extract right arm proprioception (no quaternion) ---
        robot0_eef_pos = obs["robot0_eef_pos"][:, 3:]          # right arm EEF (3D)
        robot0_gripper_qpos = obs["robot0_gripper_qpos"][:, 1:] # right gripper (1D)

        # Concatenate obs
        right_arm_obs = np.concatenate([robot0_eef_pos, robot0_gripper_qpos], axis=1)  # shape (T, 4)

        # Right arm action = last 4 of 8 total
        right_arm_action = action[:, 4:]  # shape (T, 4)

        num_dims = right_arm_obs.shape[1]  # should be 4
        steps = range(right_arm_obs.shape[0])

        # --- Plot right arm EE position and gripper vs. action ---
        fig, axes = plt.subplots(num_dims, 1, figsize=(12, 2 * num_dims), sharex=True)

        dim_names = [
            "EEF X", "EEF Y", "EEF Z",
            "Gripper"
        ]

        for i in range(num_dims):
            axes[i].plot(steps, right_arm_obs[:, i], label=f'{dim_names[i]} (obs)', color='blue')
            axes[i].plot(steps, right_arm_action[:, i], label=f'{dim_names[i]} (action)', color='red', linestyle='--')
            axes[i].set_ylabel("Value")
            axes[i].legend()
            axes[i].grid(True)

        axes[-1].set_xlabel("Step")
        plt.suptitle("Right Arm End-Effector Position and Gripper (Obs vs Action)")
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




#######################################################################################################################################################################################
file_original = "/home/qtf5422/Desktop/AIRE/ibrl-docker/data/cube_picking_and_placing_ee_pos/new_mujoco/dataset_2.hdf5"
inspect_right_arm(file_original)
process_dataset(file_original, output_file="/home/qtf5422/Desktop/AIRE/ibrl-docker/data/cube_picking_and_placing_ee_pos/new_mujoco/tresholding/dataset_tresholded.hdf5", threshold=0.05)
inspect_right_arm(file="/home/qtf5422/Desktop/AIRE/ibrl-docker/data/cube_picking_and_placing_ee_pos/new_mujoco/tresholding/dataset_tresholded.hdf5")
modify_rewards_and_create_dones(input_path="/home/qtf5422/Desktop/AIRE/ibrl-docker/data/cube_picking_and_placing_ee_pos/new_mujoco/tresholding/dataset_tresholded.hdf5", output_path="/home/qtf5422/Desktop/AIRE/ibrl-docker/data/cube_picking_and_placing_ee_pos/new_mujoco/tresholding/dataset_tresholded_wr.hdf5")
inspect_keys(file="/home/qtf5422/Desktop/AIRE/ibrl-docker/data/cube_picking_and_placing_ee_pos/new_mujoco/tresholding/dataset_tresholded_wr.hdf5")
inspect_right_arm(file="/home/qtf5422/Desktop/AIRE/ibrl-docker/data/cube_picking_and_placing_ee_pos/new_mujoco/tresholding/dataset_tresholded_wr.hdf5")
shift_actions_with_clipping(input_path="/home/qtf5422/Desktop/AIRE/ibrl-docker/data/cube_picking_and_placing_ee_pos/new_mujoco/tresholding/dataset_tresholded_wr.hdf5", output_path="/home/qtf5422/Desktop/AIRE/ibrl-docker/data/cube_picking_and_placing_ee_pos/new_mujoco/tresholding/dataset_tresholded_wr_shifted.hdf5", k=3)
inspect_right_arm(file="/home/qtf5422/Desktop/AIRE/ibrl-docker/data/cube_picking_and_placing_ee_pos/new_mujoco/tresholding/dataset_tresholded_wr_shifted.hdf5")
normalize_gripper_in_file(input_path="/home/qtf5422/Desktop/AIRE/ibrl-docker/data/cube_picking_and_placing_ee_pos/new_mujoco/tresholding/dataset_tresholded_wr_shifted.hdf5", output_path="/home/qtf5422/Desktop/AIRE/ibrl-docker/data/cube_picking_and_placing_ee_pos/new_mujoco/tresholding/dataset_tresholded_wr_shifted_norm_gripper.hdf5")
inspect_right_arm(file="/home/qtf5422/Desktop/AIRE/ibrl-docker/data/cube_picking_and_placing_ee_pos/new_mujoco/tresholding/dataset_tresholded_wr_shifted_norm_gripper.hdf5")
