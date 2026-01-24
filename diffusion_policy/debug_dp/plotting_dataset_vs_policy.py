import numpy as np
import matplotlib.pyplot as plt

# Files
dataset_file = "actions_dataset.npy"
policy_file = "actions_policy.npy"

# Load actions
dataset_actions = np.load(dataset_file, allow_pickle=True)
policy_actions = np.load(policy_file, allow_pickle=True)

print("dataset_actions : ", dataset_actions)
print("Dataset length:", len(dataset_actions))
print("Policy length:", len(policy_actions))

# Use the minimum length to stay safe
num_steps = min(len(dataset_actions), len(policy_actions))

# Indices for right arm position
idx_x, idx_y, idx_z = 8, 9, 10

# Extract trajectories
dataset_xyz = np.array([
    [dataset_actions[i][idx_x],
     dataset_actions[i][idx_y],
     dataset_actions[i][idx_z]]
    for i in range(num_steps)
])

print("dataset_xyz : ", dataset_xyz)

policy_xyz = np.array([
    [policy_actions[i][idx_x],
     policy_actions[i][idx_y],
     policy_actions[i][idx_z]]
    for i in range(num_steps)
])

steps = np.arange(num_steps)

# Plot
fig, axes = plt.subplots(3, 1, figsize=(10, 8), sharex=True)

labels = ["x", "y", "z"]

for i in range(3):
    axes[i].plot(steps, dataset_xyz[:, i], label="Dataset", linestyle="--")
    axes[i].plot(steps, policy_xyz[:, i], label="Policy")
    axes[i].set_ylabel(f"Right arm {labels[i]}")
    axes[i].legend()
    axes[i].grid(True)

axes[-1].set_xlabel("Step")

plt.suptitle("Right Arm Position: Dataset vs Policy")
plt.tight_layout()
plt.show()
