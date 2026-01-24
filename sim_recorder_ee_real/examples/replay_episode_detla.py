#!/usr/bin/env python3
"""
Playback recorded demonstration on real Trossen robots using Trossen_env.
1. Replays actions on real followers
2. Then replays saved camera images as a video
"""
import torch

import numpy as np
import h5py
import argparse
from pathlib import Path
import cv2
import time
import matplotlib.pyplot as plt
from env.trossen_wrapper_real import PixelTrossen

from scipy.spatial.transform import Rotation as R

USE_AXIS_ANGLE = False

USE_DELTA = True
DATASET_PATH = "/home/qtf5422/Desktop/AIRE/ibrl-docker/sim_recorder_ee_real/server/data/cube/dataset_tresholded_wr_shifted_delta_gripper_normed_cut_end_normalized.hdf5"

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
        
        env = PixelTrossen(env_name="TransferCubeEETask", robots="real", episode_length = 2000, 
        denormalization_path_bc = "/home/qtf5422/Desktop/AIRE/ibrl-docker/sim_recorder_ee_real/server/data/cube/delta_action_stats.json", initial_position_file = DATASET_PATH) # initialize the trossen wrapper. 

        for i in range(demo_1_number, demo_2_number) : 
            print("demo : ", i)

            env.reset()

            actions = load_episode_from_hdf5(args.dataset, demo_id=i)

            print(f"Replaying {len(actions)} steps at ~{args.control_freq} Hz (dt={1.0/args.control_freq*1000:.1f}ms)...")

            for i, delta_action in enumerate(actions):
                loop_start = time.time()

                print("action : ", delta_action, "action dim : ", delta_action.shape)

                # delta_action = np.random.uniform(-1, 1, 14)

                # Send absolute command to robot (trossen env wrapper)
                # env.step(torch.from_numpy(delta_action).float())

                env.step(torch.from_numpy(delta_action).float())

                # Timing stats
                elapsed = time.time() - loop_start
                if (i + 1) % 100 == 0:
                    print(f"  Step {i+1}/{len(actions)} - Loop time: {elapsed*1000:.1f}ms")
                    print(f"✓ Running at ~{1.0/elapsed:.0f} Hz")


        env.env.stop_background_capture()









if __name__ == '__main__':
    main()