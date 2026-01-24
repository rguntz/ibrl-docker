import h5py
import torch
import h5py
import cv2
import numpy as np
from pathlib import Path

def count_episodes(hdf5_path):
    with h5py.File(hdf5_path, "r") as f:
        data_group = f["data"]
        num_episodes = len(data_group.keys())
    return num_episodes



def inspect_keys(file) : 
    with h5py.File(file, "r") as f:
        print(list(f.keys()))

        f_data = f["data"]
        print("Attributes of f_data:")
        for key, value in f_data.attrs.items():
            print(f"{key}: {value} (type: {type(value)})")

        print("f_data keys : ", f_data.keys())
        print("the number of demos is : ", len(f_data))

        f_demo_0 = f_data["demo_0"]
        print("f_demo_0 keys : ", f_demo_0.keys())

        action = f_demo_0["actions"]
        print("action size : ", action.shape)
        print("first action : ", action[5])
        print("last action : ", action[-1])

        action_min = action[:].min()  # Convert to array for min/max
        action_max = action[:].max()
        print("Minimum action value : ", action_min)
        print("Maximum action value : ", action_max)

        obs = f_demo_0["obs"]
        print("obs keys : ", obs.keys())

        qpos = obs["qpos"]
        print("qpos shape : ", qpos.shape)

        print("proprio first episode : ", qpos[0])

        qvel = obs["qvel"]
        print("qvel shape :", qvel.shape)
        print("proprio first episode : ", qvel[0])

        ###############################################
        #NEW 

        robot0_eef_pos = obs["robot0_eef_pos"]
        print("robot0_eef_pos shape :", robot0_eef_pos.shape)
        print("robot0_eef_pos first episode : ", robot0_eef_pos[0])

        robot0_eef_quat = obs["robot0_eef_quat"]
        print("robot0_eef_quat shape :", robot0_eef_quat.shape)
        print("robot0_eef_quat first episode : ", robot0_eef_quat[0])

        robot0_gripper_qpos = obs["robot0_gripper_qpos"]
        print("robot0_gripper_qpos shape :", robot0_gripper_qpos.shape)
        print("robot0_gripper_qpos first episode : ", robot0_gripper_qpos[0])

        images = obs["cam_high_image"]
        print("the size of the images is : ", images.shape)

        # rewards = f_demo_0["rewards"]
        # for i in range(len(rewards)): 
        #     print("reward i : ", rewards[i])


input_file = "dataset.hdf5"
print(f"Number of episodes in the dataset: {count_episodes(input_file)}")

inspect_keys(input_file)


