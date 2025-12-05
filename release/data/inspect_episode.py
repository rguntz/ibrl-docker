import h5py
import cv2
import numpy as np
from pathlib import Path

input_file = "/home/qtf5422/Desktop/AIRE/ibrl-docker/release/data/robomimic/can/processed_data96.hdf5"

def inspect_keys(file) : 
    with h5py.File(file, "r") as f:
        print(list(f.keys()))

        f_data = f["data"]
        print("f_data keys : ", f_data.keys())
        print("the number of demos is : ", len(f_data))

        f_demo_0 = f_data["demo_0"]
        print("f_demo_0 keys : ", f_demo_0.keys())

        action = f_demo_0["actions"]
        print("action size : ", action.shape)

        print("first action : ", action[0])
        print("first end effector : ", f_demo_0["obs"]["robot0_eef_pos"][0])
        print("last action : ", action[-1])

        obs = f_demo_0["obs"]
        print("obs keys : ", obs.keys())

        print("size of the image : ", obs["robot0_eye_in_hand_image"].shape)

        print("image :  ", np.min(obs["robot0_eye_in_hand_image"][0]), np.max(obs["robot0_eye_in_hand_image"][0]))

        states = f_demo_0["states"]
        print("states", states)

        # 🔹 NEW: Compute min and max action values across all demos
        all_actions = []
        for demo_key in f_data.keys():
            demo_actions = f_data[demo_key]["obs"]["robot0_eef_pos"][:]
            all_actions.append(demo_actions)

        all_actions = np.concatenate(all_actions, axis=0)
        print("\n--- Action Value Statistics Across Dataset ---")
        print("Min action values: ", np.min(all_actions, axis=0))
        print("Max action values: ", np.max(all_actions, axis=0))
        print("Overall min: ", np.min(all_actions))
        print("Overall max: ", np.max(all_actions))



        


inspect_keys(input_file)






