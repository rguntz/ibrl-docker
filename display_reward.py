import numpy as np
import matplotlib.pyplot as plt
import time
import os

FILENAME = "/home/qtf5422/Desktop/AIRE/DATA/IBRL/EXPERIMENT_RL_EE_DELTA_MEDAL/rewards.bin"
POLL_INTERVAL = 0.01  # 10 ms
MAX_REWARD = 1.0      # adjust according to your reward range

plt.ion()
fig, ax = plt.subplots(figsize=(4, 6))

bar = ax.bar(0, 0.0, width=0.5, color='skyblue')
ax.set_ylim(0, MAX_REWARD)
ax.set_xlim(-0.5, 0.5)
ax.set_ylabel("Reward")
ax.set_xticks([])

last_value = None

while True:
    try:
        with open(FILENAME, "rb") as f:
            data = f.read(4)
            if len(data) == 4:
                reward = np.frombuffer(data, dtype=np.float32)[0]

                if reward != last_value:
                    bar[0].set_height(reward)
                    ax.set_title(f"Reward: {reward:.6f}")
                    fig.canvas.draw()
                    fig.canvas.flush_events()
                    last_value = reward

    except FileNotFoundError:
        pass  # file not created yet

    time.sleep(POLL_INTERVAL)

