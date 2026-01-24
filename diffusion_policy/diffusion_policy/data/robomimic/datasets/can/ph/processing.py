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


    
import h5py
import numpy as np
import matplotlib.pyplot as plt
from scipy.spatial.transform import Rotation as R

def inspect_right_arm(file):
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

        # Actions
        action = f_demo_0["actions"][:]
        print("length of the actions : ", action.shape)
        print("Action size:", action.shape)
        print("First action:", action[0])
        print("Last action:", action[-1])

        obs = f_demo_0["obs"]
        print("obs keys:", list(obs.keys()))

        # --- Extract right arm proprioception ---
        robot0_eef_pos = obs["robot0_eef_pos"][:]          # (T, 3)
        robot0_eef_quat = obs["robot0_eef_quat"][:]        # (T, 4) in [x, y, z, w] format
        robot0_gripper_qpos = obs["robot0_gripper_qpos"][:]  # (T, 1)
        print("robot0_gripper_qpos : ", robot0_gripper_qpos)

        # Convert quaternion to angle-axis
        # Note: scipy expects quaternions in [x, y, z, w] order, which matches your data
        rotations = R.from_quat(robot0_eef_quat)           # shape (T,)
        angle_axis = rotations.as_rotvec()                 # shape (T, 3)

        # Concatenate into full 7-D right arm observation: pos (3) + angle-axis (3) + gripper (1)
        right_arm_obs = np.concatenate([robot0_eef_pos, angle_axis, robot0_gripper_qpos], axis=1)  # (T, 7)

        # Assume action is already in pos (3) + angle-axis (3) + gripper (1) → 7-D
        right_arm_action = action  # should be (T, 7)

        print("Observation dim:", right_arm_obs.shape)
        print("Action dim:", right_arm_action.shape)

        T = right_arm_obs.shape[0]
        steps = range(T)

        # --- Plot all 7 dimensions: pos (3), angle-axis (3), gripper (1) ---
        dim_names = ["EEF X", "EEF Y", "EEF Z", "RX", "RY", "RZ", "Gripper"]

        fig, axes = plt.subplots(7, 1, figsize=(12, 14), sharex=True)

        for i in range(7):
            axes[i].plot(steps, right_arm_obs[:, i], label=f'{dim_names[i]} (obs)', color='blue')
            axes[i].plot(steps, right_arm_action[:, i], label=f'{dim_names[i]} (action)', color='red', linestyle='--')
            axes[i].set_ylabel(dim_names[i])
            axes[i].legend(loc='upper right')
            axes[i].grid(True)

        axes[-1].set_xlabel("Step")
        plt.suptitle("Right Arm: Pos + Angle-Axis + Gripper vs Action")
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


######################################################################################################################################################################################
file_original = "image_abs.hdf5"
inspect_right_arm(file_original)
