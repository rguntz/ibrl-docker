import h5py
import numpy as np
import matplotlib.pyplot as plt

input_file = "cube_picking_and_placing/dataset_200steps_actions16.hdf5"

# Parameters
qpos_indices = range(8)  # first 8 qpos
k_max = 20  # range of k to check, you can change this

# Load data
with h5py.File(input_file, "r") as f:
    f_data = f["data"]
    
    # Prepare lists for all demos
    qpos_all = []
    actions_all = []
    
    for demo_key in f_data.keys():
        demo = f_data[demo_key]
        prop = demo["obs"]["prop"][:]  # shape (200, 32)
        actions = demo["actions"][:]   # shape (200, 16)
        qpos_all.append(prop[:, :16])
        actions_all.append(actions)
        
    qpos_all = np.array(qpos_all)    # shape (num_demos, 200, 16)
    actions_all = np.array(actions_all)  # shape (num_demos, 200, 16)

# Compute delta_qpos for different k
delta_qpos_k = {k: [] for k in range(1, k_max+1)}

num_demos, num_steps, _ = qpos_all.shape

for k in range(1, k_max+1):
    for demo in range(num_demos):
        delta_qpos = qpos_all[demo, k:, :16] - qpos_all[demo, :-k, :16]
        delta_qpos_k[k].append(delta_qpos[:, :8])  # first 8 qpos
    # average over demos and time
    delta_qpos_k[k] = np.mean(np.concatenate(delta_qpos_k[k], axis=0), axis=0)

# Convert to array for plotting
delta_qpos_matrix = np.stack([delta_qpos_k[k] for k in range(1, k_max+1)], axis=0)  # shape (k_max, 8)

# Plot
plt.figure(figsize=(12, 6))
for i in range(8):
    plt.plot(
        range(1, k_max+1),
        delta_qpos_matrix[:, i],
        label=f'qpos_{i}',
        linewidth=3   # <--- thicker lines
    )
plt.xlabel('k (steps ahead)')
plt.ylabel('Average Δqpos')
plt.title('Average change in qpos over k steps')
plt.legend()
plt.grid(True)
plt.show()
