import h5py
import torch
import h5py
import cv2
import numpy as np
from pathlib import Path
import matplotlib.pyplot as plt
import numpy as np

def count_episodes(hdf5_path):
    with h5py.File(hdf5_path, "r") as f:
        data_group = f["data"]
        num_episodes = len(data_group.keys())
    return num_episodes


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
        print("robot0_eef_pos keys : ", robot0_eef_pos.shape)
        print("proprio first episode : ", robot0_eef_pos[0])

        robot0_eef_quat = obs["robot0_eef_quat"]
        print("robot0_eef_quat keys : ", robot0_eef_quat.shape)
        print("proprio first episode : ", robot0_eef_quat[0])

        robot0_gripper_qpos = obs["robot0_gripper_qpos"]
        print("robot0_gripper_qpos keys : ", robot0_gripper_qpos.shape)
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

def inspect_keys_original_file(file) : 
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


        reward = f_demo_0["rewards"]
        for i in range(len(reward)): 
            print("rewards : ", reward[i])
            print("type of reward : ", type(reward[i]))

        dones = f_demo_0["dones"]
        for i in range(len(dones)): 
            print("dones : ", dones[i])

        states = f_demo_0["states"]
        print("state 1 : ", states[0])

        print("obs prop : ", )



def inspect_true_dataset(file):
    with h5py.File(file, "r") as f:
        # Show top-level keys
        print("Top-level keys:", list(f.keys()))

        f_data = f["data"]
        print("f_data keys:", list(f_data.keys()))
        print("The number of demos is:", len(f_data))

        # Take first demo (note: you're using "demo_2" — double-check if intentional)
        f_demo_0 = f_data["demo_2"]
        print("f_demo_0 keys:", list(f_demo_0.keys()))

        obs = f_demo_0["obs"]
        print("obs keys:", list(obs.keys()))

        action = f_demo_0["actions"]
        print("First action:", action[0])

        # --- Position and action (first 3 dims) plot ---
        ee_pos = obs["robot0_eef_pos"]
        total_ee_pos = []
        total_action_pos = []

        for i in range(len(ee_pos)):
            total_ee_pos.append(ee_pos[i])
            total_action_pos.append(action[i][:3])

        ee_array = np.array(total_ee_pos)
        action_array_pos = np.array(total_action_pos)

        ee_min = ee_array.min(axis=0)
        ee_max = ee_array.max(axis=0)
        print("EEF position min:", ee_min)
        print("EEF position max:", ee_max)

        steps = range(len(ee_array))

        plt.figure(figsize=(12, 6))
        plt.plot(steps, ee_array[:, 0], label='EEF X Position', color='red')
        plt.plot(steps, ee_array[:, 1], label='EEF Y Position', color='green')
        plt.plot(steps, ee_array[:, 2], label='EEF Z Position', color='blue')
        plt.plot(steps, action_array_pos[:, 0], label='Action X', linestyle='--', color='red')
        plt.plot(steps, action_array_pos[:, 1], label='Action Y', linestyle='--', color='green')
        plt.plot(steps, action_array_pos[:, 2], label='Action Z', linestyle='--', color='blue')
        plt.xlabel("Step")
        plt.ylabel("Value")
        plt.title("End-Effector Position and Action (First 3 dims)")
        plt.legend()
        plt.grid(True)
        plt.show()

        # --- Quaternion plot (new part) ---
        ee_quat = obs["robot0_eef_quat"]
        total_ee_quat = []
        total_action_quat = []

        for i in range(len(ee_quat)):
            total_ee_quat.append(ee_quat[i])
            total_action_quat.append(action[i][3:])  # last 4 values

        quat_array = np.array(total_ee_quat)        # shape: (T, 4)
        action_quat_array = np.array(total_action_quat)  # shape: (T, 4)

        plt.figure(figsize=(12, 6))
        quat_labels = ['Q_w', 'Q_x', 'Q_y', 'Q_z']
        colors = ['purple', 'orange', 'brown', 'pink']

        for j in range(4):
            plt.plot(steps, quat_array[:, j], label=f'Observed {quat_labels[j]}', color=colors[j])
            plt.plot(steps, action_quat_array[:, j], label=f'Action {quat_labels[j]}', 
                     linestyle='--', color=colors[j])

        plt.xlabel("Step")
        plt.ylabel("Quaternion Component")
        plt.title("End-Effector Quaternion and Corresponding Action Quaternion")
        plt.legend()
        plt.grid(True)
        plt.show()



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

        prop = obs["prop"]
        print("Proprio shape:", prop.shape)
        print("First proprio state:", prop[0])

        # Select only the right arm joint positions (last 8 of the 16 positions)
        right_arm_state = prop[:, 8:16]    # shape: (num_steps, 8)
        right_arm_action = action[:, -8:]  # shape: (num_steps, 8)

        num_joints = right_arm_state.shape[1]
        steps = range(right_arm_state.shape[0])

        # Plot each joint in a separate subplot
        fig, axes = plt.subplots(num_joints, 1, figsize=(12, 2*num_joints), sharex=True)

        for j in range(num_joints):
            print("min and max values : ", np.min(right_arm_state[:, j]), np.max(right_arm_state[:, j]))
            axes[j].plot(steps, right_arm_state[:, j], label=f'Joint {j+1} Position', color='blue')
            axes[j].plot(steps, right_arm_action[:, j], label=f'Joint {j+1} Action', linestyle='--', color='red')
            axes[j].set_ylabel("Value")
            axes[j].legend()
            axes[j].grid(True)

        axes[-1].set_xlabel("Step")
        plt.suptitle("Right Arm Joint Positions and Actions")
        plt.tight_layout(rect=[0, 0, 1, 0.96])
        plt.show()




def verify_actions_norm(): 

    file = "cube_picking_and_placing/dataset_200steps_actions16.hdf5"
    with h5py.File(file, "r") as f:
        print("Top-level keys:", list(f.keys()))

        f_data = f["data"]
        print("f_data keys:", list(f_data.keys()))
        print("The number of demos is:", len(f_data))

        # Take first demo
        f_demo_0 = f_data["demo_1"]
        print("f_demo_0 keys:", list(f_demo_0.keys()))

        # Actions
        action_true = f_demo_0["actions"]

        print("arrives here")


        file = "cube_picking_and_placing/dataset_200steps_actions16_delta_action_normalized_minmax.hdf5"
        with h5py.File(file, "r") as f:
            print("Top-level keys:", list(f.keys()))

            f_data_2 = f["data"]
            print("f_data keys:", list(f_data_2.keys()))
            print("The number of demos is:", len(f_data_2))

            # Take first demo
            f_demo_0_2 = f_data_2["demo_1"]
            print("f_demo_0 keys:", list(f_demo_0_2.keys()))

            # Actions
            action_norm = f_demo_0_2["actions"]


            for i in range(len(action_norm)) : 

                joint_mins = np.array([-np.pi, 0, 0, -np.pi/2, -np.pi/2, -np.pi,
                                        0, 0, -np.pi, 0, 0, -np.pi/2, -np.pi/2, -np.pi,
                                        0, 0])
                joint_maxs = np.array([np.pi, np.pi, 2.36, np.pi/2, np.pi/2, np.pi,
                                    0.04, 0.04, np.pi, np.pi, 2.36, np.pi/2, np.pi/2, np.pi,
                                    0.04, 0.04])

                # 2. model output (joint delta)
                model_delta = ((action_norm[i]  + 1) / 2) * (joint_maxs - joint_mins) + joint_mins # we need to denormalize the action because the current one is between -1 and 1.
                model_delta = model_delta + f_demo_0_2["obs"]["prop"][i][:16]


                print("true action : ", action_true[i])
                print("vs norm action : ", model_delta)




def inspect_initial_position(file) : 
    with h5py.File(file, "r") as f:
        print(list(f.keys()))

        f_data = f["data"]
        print("f_data keys : ", f_data.keys())
        print("the number of demos is : ", len(f_data))

        sum = np.zeros(16, dtype=float)
        n = 0 

        for i in range(len(f_data)) : 
            f_demo_0 = f_data[f"demo_{i}"]
            prop = f_demo_0["obs"]["prop"]
            for j in range(10) : 
                sum += prop[j, :16]
                n += 1
        mean = sum/n
        print("the mean is : ", mean)   



input_file = "/home/qtf5422/Desktop/AIRE/ibrl-docker/data/cube_picking_and_placing_ee_pos/dataset.hdf5"
inspect_keys(input_file)
save_first_camera_video_and_image(input_file, output_video_path = "/home/qtf5422/Desktop/AIRE/ibrl-docker/data/cube_picking_and_placing_ee_pos/episode.mp4", output_image_path = "/home/qtf5422/Desktop/AIRE/ibrl-docker/data/cube_picking_and_placing_ee_pos/first_frame.png", fps=30)

print("#############################################################################################")
print("#############################################################################################")
print("#############################################################################################")
print("#############################################################################################")

inspect_true_dataset(file = "/home/qtf5422/Desktop/AIRE/ibrl-docker/release/data/robomimic/can/processed_data96.hdf5")


