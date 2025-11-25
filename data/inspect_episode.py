import h5py
import torch

def count_episodes(hdf5_path):
    with h5py.File(hdf5_path, "r") as f:
        data_group = f["data"]
        num_episodes = len(data_group.keys())
    return num_episodes

input_file = "cube_picking_and_placing/dataset.hdf5"
print(f"Number of episodes in the dataset: {count_episodes(input_file)}")

import h5py
import cv2
import numpy as np
from pathlib import Path

def save_first_camera_video_and_image(hdf5_path, output_video_path, output_image_path, fps=30):
    hdf5_path = Path(hdf5_path)
    output_video_path = Path(output_video_path)
    output_image_path = Path(output_image_path)

    with h5py.File(hdf5_path, "r") as f:
        data_group = f["data"]
        first_demo = list(data_group.keys())[0]
        obs_group = data_group[first_demo]["obs"]
        first_camera = list(obs_group.keys())[0]

        images = obs_group[first_camera][:]   # (T, C, H, W) — encoded as RGB

        # Convert (T, C, H, W) → (T, H, W, C)
        images_hwc = np.transpose(images, (0, 2, 3, 1))   # now RGB

        # Convert RGB → BGR for OpenCV
        images_bgr = images_hwc[:, :, :, ::-1]

        # ------------------------
        # SAVE FIRST IMAGE
        # ------------------------
        cv2.imwrite(str(output_image_path), images_bgr[0])

        # ------------------------
        # SAVE VIDEO
        # ------------------------
        fourcc = cv2.VideoWriter_fourcc(*"mp4v")
        h, w = images_bgr[0].shape[:2]
        out = cv2.VideoWriter(str(output_video_path), fourcc, fps, (w, h))

        for frame in images_bgr:
            out.write(frame)

        out.release()


#input_file = "cube_picking_and_placing/dataset.hdf5"
input_file = "cube_picking_and_placing/dataset_actions16_shifted_5_normalized_minmax.hdf5"


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

        prop = obs["prop"]
        print("prop keys : ", prop.shape)

        print("proprio first episode : ", prop[0])


        images = obs["cam_high_image"]
        print("the size of the images is : ", images.shape)

        print("image : ",np.min(images[0]), np.max(images[0]))

print("arrives here")
inspect_keys(input_file)

input_file = "cube_picking_and_placing/dataset.hdf5"
inspect_keys(input_file)

"""
input_file = "cube_picking_and_placing/dataset_200steps_actions16_shifted_5_normalized_minmax.hdf5"

datafile = h5py.File(input_file) # maps global step index → (episode_id, timestep)
num_episode: int = len(list(datafile["data"].keys()))  # type: ignore
print(f"Raw Dataset size (#episode): {num_episode}")

all_actions = []  # for # logging purpose
for episode_id in range(num_episode): # everything that is below is inside this loop.
    episode_tag = f"demo_{episode_id}" 
    episode = datafile[f"data/{episode_tag}"]
    actions = np.array(episode["actions"]).astype(np.float32)  # type: ignore
    actions = torch.from_numpy(actions)
    all_actions.append(actions)

print("all actions : ", all_actions)

"""




