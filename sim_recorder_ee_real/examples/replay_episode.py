#!/usr/bin/env python3
"""
Playback recorded demonstration on real Trossen robots using Trossen_env.
1. Replays actions on real followers
2. Then replays saved camera images as a video
"""

import numpy as np
import h5py
import argparse
from pathlib import Path
import cv2
import time
import matplotlib.pyplot as plt

from env.trossen_env import Trossen_env
from scipy.spatial.transform import Rotation as R

USE_AXIS_ANGLE = False

# BE CAREFULL WITH CHOSING DELTA OR NOT
USE_DELTA = False
DATASET_PATH = "/home/qtf5422/Desktop/AIRE/ibrl-docker/sim_recorder_ee_real/server/data/dataset_cube.hdf5.hdf5"

# USE_DELTA = True
# DATASET_PATH = "/home/qtf5422/Desktop/AIRE/ibrl-docker/sim_recorder_ee_real/server/data/cube/dataset_tresholded_wr_shifted_delta.hdf5"

from trossen_arm_mujoco.utils import (
    plot_observation_images,
)


def load_episode_from_hdf5(hdf5_path: str, demo_id: int = 0):
    """Load actions from a specific demo in HDF5."""
    with h5py.File(hdf5_path, 'r') as f:
        demo_group = f[f'data/demo_{demo_id}']
        actions = np.array(demo_group['actions'])  # Shape: (T, action_dim)
        print(f"Loaded demo_{demo_id} with {len(actions)} steps")
        return actions


def load_demo_actions_and_obs(dataset_path, demo_name="demo_0"):
    """
    Loads actions and observations from the HDF5 dataset for a specific demo.
    
    Returns:
        actions: (T, 16) array of actions
        obs_dict: dict with keys 'cam_high', 'cam_low', 'cam_left_wrist', 'cam_right_wrist', 'prop'
                images are (T, 128, 128, 3) after transposing channels
    """
    with h5py.File(dataset_path, "r") as f:
        demo = f[f"data/{demo_name}"]
        actions = demo["actions"][:]

        quat_group = None

        if USE_AXIS_ANGLE : 
            obs_group = demo["obs"]
            obs_dict = {
                "cam_high": np.transpose(obs_group["cam_high_image"][:], (0, 2, 3, 1)),
                "cam_low": np.transpose(obs_group["cam_low_image"][:], (0, 2, 3, 1)),
                "cam_left_wrist": np.transpose(obs_group["cam_left_wrist_image"][:], (0, 2, 3, 1)),
                "cam_right_wrist": np.transpose(obs_group["cam_right_wrist_image"][:], (0, 2, 3, 1)),
                "qpos": obs_group["qpos"][:], 
                "qvel": obs_group["qvel"][:],
                "robot0_eef_pos" : obs_group["robot0_eef_pos"][:], 
                "robot0_eef_quat" : obs_group["robot0_eef_quat"][:], 
                "robot0_gripper_qpos" : obs_group["robot0_gripper_qpos"][:],
                "robot0_eef_aa" : obs_group["robot0_eef_aa"][:]
            }
        else : 
            obs_group = demo["obs"]
            obs_dict = {
                "cam_high": np.transpose(obs_group["cam_high_image"][:], (0, 2, 3, 1)),
                "cam_low": np.transpose(obs_group["cam_low_image"][:], (0, 2, 3, 1)),
                "cam_left_wrist": np.transpose(obs_group["cam_left_wrist_image"][:], (0, 2, 3, 1)),
                "cam_right_wrist": np.transpose(obs_group["cam_right_wrist_image"][:], (0, 2, 3, 1)),
                "qpos": obs_group["qpos"][:], 
                "qvel": obs_group["qvel"][:],
                "robot0_eef_pos" : obs_group["robot0_eef_pos"][:], 
                "robot0_eef_quat" : obs_group["robot0_eef_quat"][:], 
                "robot0_gripper_qpos" : obs_group["robot0_gripper_qpos"][:]
            }            
    return actions, obs_dict, quat_group

def load_images_from_hdf5(hdf5_path: str, demo_id: int = 0):
    """Load all camera images for a demo and return as dict of arrays (T, C, H, W)."""
    with h5py.File(hdf5_path, 'r') as f:
        demo_group = f[f'data/demo_{demo_id}']
        obs_group = demo_group['obs']

        images = {}
        for key in obs_group.keys():
            if key.endswith('_image'):
                # Shape: (T, C, H, W) → convert to (T, H, W, C)
                img_array = obs_group[key][:]
                if img_array.ndim == 4 and img_array.shape[1] in [1, 3]:
                    img_array = np.transpose(img_array, (0, 2, 3, 1))
                    # If grayscale, remove channel dim for display
                    if img_array.shape[-1] == 1:
                        img_array = img_array.squeeze(-1)
                images[key.replace('_image', '')] = img_array
        print(f"Loaded images for demo_{demo_id}: {list(images.keys())}")
        return images


def plotting_sim_teleop_with_dataset(
    demo_1_number,
    demo_2_number,
    dataset_path="/home/qtf5422/Desktop/AIRE/ibrl-docker/sim_recorder_ee_real/server/data/dataset.hdf5",
):

    cam_list = ["cam_high", "cam_low", "cam_left_wrist", "cam_right_wrist"]

    for demo_number in range(demo_1_number, demo_2_number):
        demo_name = f"demo_{demo_number}"
        print("the demo is :", demo_name)

        actions, dataset_obs, action_quat = load_demo_actions_and_obs(
            dataset_path, demo_name
        )

        plt.ion()

        # ---- CREATE FIGURE ONCE ----
        fig = plt.figure("Dataset Observations")
        fig.clf()  # important if reusing same window

        dataset_imgs = plot_observation_images(
            {'images': {cam: dataset_obs[cam][0] for cam in cam_list}},
            cam_list
        )

        # ---- BLITTING SETUP ----
        fig.canvas.draw()
        background = fig.canvas.copy_from_bbox(fig.bbox)

        for t in range(len(actions)):
            fig.canvas.restore_region(background)

            for img, cam, ax in zip(dataset_imgs, cam_list, fig.axes):
                img.set_data(dataset_obs[cam][t])
                ax.draw_artist(img)

            fig.canvas.blit(fig.bbox)
            fig.canvas.flush_events()



def delta_action_to_absolute(delta_action, current_left_pos, current_left_quat, 
                             current_right_pos, current_right_quat):
    """
    Convert 14D delta action to 14D absolute action (pos + axis-angle + gripper per arm).
    
    Args:
        delta_action: (14,) — [dpos_L(3), drot_L(3), grip_L(1), dpos_R(3), drot_R(3), grip_R(1)]
        current_left_pos: (3,) — current left EE position
        current_left_quat: (4,) — current left EE quat [w, x, y, z]
        current_right_pos: (3,) — current right EE position
        current_right_quat: (4,) — current right EE quat [w, x, y, z]
        
    Returns:
        absolute_action: (14,) — [pos_L(3), aa_L(3), grip_L(1), pos_R(3), aa_R(3), grip_R(1)]
    """
    # === Left arm ===
    dpos_L = delta_action[0:3]
    drot_L_aa = delta_action[3:6]
    grip_L = delta_action[6:7]

    # New absolute position
    pos_L_abs = current_left_pos + dpos_L

    # Convert current quat to Rotation
    qL_cur_scipy = [current_left_quat[1], current_left_quat[2], current_left_quat[3], current_left_quat[0]]
    R_L_cur = R.from_quat(qL_cur_scipy)

    # Delta rotation (from axis-angle)
    R_L_delta = R.from_rotvec(drot_L_aa)

    # Apply delta: R_new = R_delta * R_current
    R_L_new = R_L_delta * R_L_cur

    # Convert new orientation to axis-angle
    aa_L_abs = R_L_new.as_rotvec()

    # === Right arm ===
    dpos_R = delta_action[7:10]
    drot_R_aa = delta_action[10:13]
    grip_R = delta_action[13:14]

    pos_R_abs = current_right_pos + dpos_R

    qR_cur_scipy = [current_right_quat[1], current_right_quat[2], current_right_quat[3], current_right_quat[0]]
    R_R_cur = R.from_quat(qR_cur_scipy)
    R_R_delta = R.from_rotvec(drot_R_aa)
    R_R_new = R_R_delta * R_R_cur
    aa_R_abs = R_R_new.as_rotvec()

    # Concatenate into 14D absolute action
    absolute_action = np.concatenate([
        pos_L_abs, aa_L_abs, grip_L,
        pos_R_abs, aa_R_abs, grip_R
    ])

    return absolute_action.astype(np.float64)  # or float32 if your env expects it

def get_current_ee_state(env):
    """
    Returns current end-effector poses as:
        (left_pos, left_quat, right_pos, right_quat)
    All as np.ndarray of shape (3,) or (4,)
    Quaternions in [w, x, y, z] format.
    """
    obs = env._get_observation()
    eef_pos = obs['robot0_eef_pos']      # (6,)
    eef_quat = obs['robot0_eef_quat']    # (8,)
    
    left_pos = eef_pos[:3]
    right_pos = eef_pos[3:]
    left_quat = eef_quat[:4]
    right_quat = eef_quat[4:]
    
    return left_pos, left_quat, right_pos, right_quat

def constrain_position_to_sphere(action_14d, center=np.array([0.0, 0.0, 0.0]), diameter=1.3):
    """
    Clamp the 3D end-effector positions in a 14D absolute action to lie within a sphere.
    
    Args:
        action_14d: np.ndarray of shape (14,)
            Format: [pos_L(3), aa_L(3), grip_L(1), pos_R(3), aa_R(3), grip_R(1)]
        center: np.ndarray of shape (3,) — sphere center in Cartesian space
        diameter: float — diameter of the sphere (default: 0.75 m → radius = 0.375 m)
    
    Returns:
        clamped_action: np.ndarray of shape (14,) with positions constrained
    """
    radius = diameter / 2.0
    action = np.copy(action_14d)

    # Extract positions
    pos_L = action[0:3]
    pos_R = action[7:10]

    # Constrain left arm
    pos_L_rel = pos_L - center
    dist_L = np.linalg.norm(pos_L_rel)
    if dist_L > radius:
        pos_L = center + (pos_L_rel / dist_L) * radius

    # Constrain right arm
    pos_R_rel = pos_R - center
    dist_R = np.linalg.norm(pos_R_rel)
    if dist_R > radius:
        pos_R = center + (pos_R_rel / dist_R) * radius

    # Write back
    action[0:3] = pos_L
    action[7:10] = pos_R

    return action

def reset_to_position_dataset(env) : 
    file = "/home/qtf5422/Desktop/AIRE/ibrl-docker/sim_recorder_ee_real/server/data/dataset_tresholded_wr_shifted_delta_gripper_normed_cut_end_normalized.hdf5"
    with h5py.File(file, "r") as f:
        f_data = f["data"]

        sum = np.zeros(14, dtype=float)
        n = 0 

        for i in range(len(f_data)) : 
            f_demo_0 = f_data[f"demo_{i}"]
            obs = f_demo_0["obs"]

            for j in range(10) : 
                action_j = np.concatenate([obs["robot0_eef_pos"][j, :3], obs["robot0_eef_aa"][j, :3], np.array([obs["robot0_gripper_qpos"][j, 0]]), 
                                        obs["robot0_eef_pos"][j, -3:],obs["robot0_eef_aa"][j, -3:], np.array([obs["robot0_gripper_qpos"][j, 1]])])    
                sum += action_j 
                n += 1

        mean = sum/n 

    for i in range(10) : 
        print("First steps : ", i)
        env.step(mean)

def main():
    parser = argparse.ArgumentParser(description="Playback recorded episode on real robots + image replay")
    parser.add_argument('--dataset', type=str, required=False, help='Path to dataset.hdf5', default=DATASET_PATH)
    parser.add_argument('--demo-id', type=int, default=0, help='Demo index to playback (e.g., 0 for demo_0)')
    parser.add_argument('--follower-left-ip', type=str, default='192.168.1.5')
    parser.add_argument('--follower-right-ip', type=str, default='192.168.1.3')
    parser.add_argument('--control-freq', type=float, default=20.0, help='Control frequency (Hz)')
    parser.add_argument('--playback-fps', type=float, default=15.0, help='Image playback FPS')
    parser.add_argument('--model', type=str, default='wxai_v0')
    parser.add_argument('--skip-action-playback', action='store_true', help='Skip robot playback, only replay images', default=False)
    args = parser.parse_args()

    demo_1_number = 0
    demo_2_number = 1

    # # ---- PART 2: IMAGE REPLAY ----
    # print("\n" + "="*60)
    # print("Starting image replay...")

    # plotting_sim_teleop_with_dataset(demo_1_number=demo_1_number, demo_2_number=demo_2_number)
    
    # ---- PART 1: ACTION PLAYBACK ON REAL ROBOTS ----
    if not args.skip_action_playback:
        env = Trossen_env(
            follower_left_ip=args.follower_left_ip,
            follower_right_ip=args.follower_right_ip,
            control_dt= 1.0 / args.control_freq,
            model=args.model,
            max_episode_steps = 1000
        )

        for i in range(demo_1_number, demo_2_number) : 
            print("demo : ", i)

            env.reset()

            actions = load_episode_from_hdf5(args.dataset, demo_id=i)

            print(f"Replaying {len(actions)} steps at ~{args.control_freq} Hz (dt={1.0/args.control_freq*1000:.1f}ms)...")

            for i, delta_action in enumerate(actions):
                loop_start = time.time()

                if USE_DELTA : 

                    # Get real-time EE state from robot
                    left_pos, left_quat, right_pos, right_quat = get_current_ee_state(env)

                    print("delta action : ", delta_action)

                    # Convert delta → absolute (pos + axis-angle + gripper)
                    delta_action = delta_action_to_absolute(
                        delta_action, left_pos, left_quat, right_pos, right_quat
                    )

                delta_action = constrain_position_to_sphere(delta_action) # constrain to physical dimension of robot. 

                # Send absolute command to robot
                env.step(delta_action)

                # Timing stats
                elapsed = time.time() - loop_start
                if (i + 1) % 100 == 0:
                    print(f"  Step {i+1}/{len(actions)} - Loop time: {elapsed*1000:.1f}ms")
                    print(f"✓ Running at ~{1.0/elapsed:.0f} Hz")


        env.stop_background_capture()









if __name__ == '__main__':
    main()