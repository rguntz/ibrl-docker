import numpy as np
import matplotlib.pyplot as plt

# Load saved actions
actions_file = "actions_policy.npy"
actions_list = np.load(actions_file, allow_pickle=True)  # this is a list of arrays

print("actions_list length: ", len(actions_list))
# Iterate over each step
for step_idx, action_matrix in enumerate(actions_list):
    # Make sure it's a proper numpy array
    print("action_matrix : ", action_matrix)

    if step_idx == 20 : 
        break
