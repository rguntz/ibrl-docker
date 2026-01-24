import numpy as np

path = "forward.npz"  # adjust path if needed

data = np.load(path)

for k in data.keys():
    print(k, data[k].shape)

import matplotlib.pyplot as plt

rewards = data["rewards"].squeeze()  # (6000,)

plt.figure()
plt.plot(rewards[:400])
plt.xlabel("Step")
plt.ylabel("Reward")
plt.title("Reward per step (first 1000 steps)")
plt.grid(True)
plt.show()


terminals = data["terminals"].squeeze()  # (6000,)

plt.figure()
plt.plot(terminals[:400])
plt.xlabel("Step")
plt.ylabel("terminals")
plt.title("terminals per step (first 1000 steps)")
plt.grid(True)
plt.show()

import cv2
import numpy as np

# Load observations
observations = data["observations"]  # (6000, 84, 84, 3)

# Parameters
num_frames = 1000
height, width, channels = observations.shape[1:]
output_path = "observations_first_200.mp4"
fps = 20  # frames per second

# Define video writer (MP4)
fourcc = cv2.VideoWriter_fourcc(*"mp4v")
video = cv2.VideoWriter(output_path, fourcc, fps, (width, height))

for i in range(num_frames):
    frame = observations[i]

    # Ensure uint8 format (important)
    if frame.dtype != np.uint8:
        frame = (frame * 255).astype(np.uint8)

    # OpenCV expects BGR, but your data is RGB
    frame = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)

    video.write(frame)

video.release()
print(f"Video saved to {output_path}")
