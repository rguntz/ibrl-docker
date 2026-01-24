import numpy as np
import matplotlib.pyplot as plt

# Load trajectory
trajectory_actions = np.load("saved_trajectory.npy", allow_pickle=True)  # positions in trajectory

# Sampling frequency
hz = 26
dt = 1 / hz  # time between steps

# Extract right arm positions (first 3 indices)
right_arm_positions = np.array([action[6:9] for action in trajectory_actions])

# Compute velocity: difference between consecutive positions divided by dt
# velocity shape will be (N-1, 3)
right_arm_velocities = (right_arm_positions[1:] - right_arm_positions[:-1]) / dt

# Compute speed norm
speed_norm = np.linalg.norm(right_arm_velocities, axis=1)

# Time array for plotting (use mid-point times between steps)
time_array = np.arange(len(speed_norm)) * dt

# Plot
plt.figure(figsize=(10,5))
plt.plot(time_array, speed_norm, color='blue', marker='o', markersize=3)
plt.title("Right Arm Speed Along Trajectory")
plt.xlabel("Time [s]")
plt.ylabel("Speed [units/s]")
plt.grid(True)
plt.show()
