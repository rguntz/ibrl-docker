import matplotlib.pyplot as plt
import numpy as np
import pandas as pd
from resfit.dexmg.environments.dexmg import RobosuiteGymWrapper
import time 

"""
    0-2   Right wrist Δpos Cartesian x / y / z offset (metres) for the EE site gripper0_right_grip_site.
    3-5   Right wrist Δrot Axis-angle components rx, ry, rz; ‖r‖ = rotation angle (rad).
    6-11  Right Inspire-hand joints (joint-position targets, rad)
    12-14 Left wrist Δpos Cartesian x / y / z offset for gripper0_left_grip_site.
    15-17 Left wrist Δrot Axis-angle components for left EE orientation.
    18-23 Left Inspire-hand joints (same ordering as right).
"""

def init_plot():
    fig, ax = plt.subplots()
    im = ax.imshow(np.zeros((224, 224, 3), dtype=np.uint8))  # placeholder
    ax.axis('off')
    plt.ion()
    plt.show()
    return fig, ax, im

def update_plot(im, frame):
    im.set_data(frame)
    plt.pause(0.001)

def main():
    # -----------------------------
    # Environment setup
    # -----------------------------
    eval_env = "TwoArmCoffee"
    video_key = "observation.images.frontview"

    # Directly instantiate the RobosuiteGymWrapper
    env = RobosuiteGymWrapper(
        env_name=eval_env,
        num_envs=1,          # wrapper still expects 1
        render_gpu_device_id=0,
        camera_size=84,
        render_size=224,
        env_id=0,
    )
    env.set_video_key(video_key)

    # -----------------------------
    # Load actions from Parquet
    # -----------------------------
    parquet_path = "/home/qtf5422/.cache/huggingface/lerobot/ankile/dexmg-two-arm-coffee/data/chunk-000/episode_000999.parquet"
    df = pd.read_parquet(parquet_path)

    actions = np.stack(df["action"].to_numpy()).astype(np.float32)  # shape: (num_steps, 24)
    state = np.stack(df["observation.state"].to_numpy()).astype(np.float32)
    state_left = state[:, 0:3]
    state_right = state[:, 18:21]

    # -----------------------------
    # Reset environment
    # -----------------------------
    obs, _ = env.reset()
    print("obs state shape : ", obs["observation.state"].shape)
    fig, ax, im = init_plot()
    raise RuntimeError("stop execution")

    # -----------------------------
    # Replay actions
    # -----------------------------
    for step_idx, action in enumerate(actions):
        batch_action = action[None, :]  # keep batch dim (1, action_dim)
        obs, reward, terminated, truncated, info = env.step(batch_action)

        # Direct access to sim
        sim = env.unwrapped.env.sim
        all_bodies = [sim.model.body_id2name(i) for i in range(sim.model.nbody)]
        print("Bodies in the simulation:", all_bodies)
        # print("Sim keys:", dir(sim.data))  # inspect available physics attributes

        # Render frame
        frame = env.render()
        update_plot(im, frame)

        # # Get the body ID for the cup base
        # cup_base_id = sim.model.body_name2id("coffee_pod_main")
        # coffee_pod_main = sim.data.body_xpos[cup_base_id]
        # print("z position coffee : ", coffee_pod_main[2])

        print("-----------------------------------------------")
        print("z state_left : ", state_left[step_idx, 2])
        print("z state_right : ", state_right[step_idx, 2])
        print("-----------------------------------------------")

        time.sleep(0.2)

        if terminated or truncated:
            print(f"Episode ended at step {step_idx}")
            break

    env.close()

if __name__ == "__main__":
    main()