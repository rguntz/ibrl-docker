#!/usr/bin/env python3
"""
filter_dataset_by_state_change.py

Outputs a new HDF5 file with preserved data metadata (including env_args).
"""

import h5py
import numpy as np
import os
from pathlib import Path
import json

INPUT_FILE = "cube_picking_and_placing/dataset.hdf5"
THRESHOLD = 0.05
OUTPUT_PROCESSED_FILE = f"cube_picking_and_placing/dataset_filtered_threshold.hdf5"
OUTPUT_SHIFTED = f"cube_picking_and_placing/dataset_filtered_threshold_shifted.hdf5"
OUTPUT_NORM_FILE = f"cube_picking_and_placing/dataset_filtered_threshold_shifted_norm_min_max.hdf5"


def filter_demo_states(prop_full, threshold):
    states = prop_full[:, :16]
    T = states.shape[0]
    if T == 0:
        return []

    kept_idx = [0]
    last_state = states[0].copy()

    for t in range(1, T):
        if np.linalg.norm(states[t] - last_state) > threshold:
            kept_idx.append(t)
            last_state = states[t].copy()

    return kept_idx


def process_dataset(input_file, output_file, threshold):
    if not os.path.exists(input_file):
        raise FileNotFoundError(f"Input file not found: {input_file}")

    with h5py.File(input_file, "r") as fin, h5py.File(output_file, "w") as fout:

        # 🔹 Copy root-level attributes
        for k, v in fin.attrs.items():
            fout.attrs[k] = v

        # 🔹 Create 'data' group and preserve its attributes (⚠ The missing part)
        f_data_in = fin["data"]
        f_data_out = fout.create_group("data")
        for k, v in f_data_in.attrs.items():  # PRESERVE 'env_args' here
            f_data_out.attrs[k] = v

        demo_names = list(f_data_in.keys())
        print("Found demos:", len(demo_names))

        for demo_name in demo_names:
            grp_in = f_data_in[demo_name]
            print(f"\nProcessing demo: {demo_name}")

            obs_in = grp_in["obs"]
            prop_in = obs_in["prop"][:]
            T, _ = prop_in.shape

            kept_idx = filter_demo_states(prop_in, threshold)
            if len(kept_idx) == 0:
                print(f"  Warning: no indices kept for {demo_name}. Skipping.")
                continue

            grp_out = f_data_out.create_group(demo_name)

            # ----- Rewards -----
            if "rewards" in grp_in:
                rewards_in = grp_in["rewards"][:]
                rewards_out = rewards_in[kept_idx]
                grp_out.create_dataset("rewards", data=rewards_out, compression="gzip")

            # ----- Observations -----
            obs_out_grp = grp_out.create_group("obs")
            for key in obs_in.keys():
                data = obs_in[key]
                if isinstance(data, h5py.Dataset):
                    obs_out_grp.create_dataset(key, data=data[kept_idx], compression="gzip")
                else:
                    grp = obs_out_grp.create_group(key)
                    for attr_k, attr_v in obs_in[key].attrs.items():
                        grp.attrs[attr_k] = attr_v

            # ----- Actions based on state deltas -----
            kept_states = prop_in[kept_idx, :16]
            N_kept = kept_states.shape[0]
            actions_out = np.zeros((N_kept, 16))
            for i in range(N_kept - 1):
                actions_out[i] = kept_states[i + 1]
            actions_out[-1] = kept_states[-1]
            grp_out.create_dataset("actions", data=actions_out, compression="gzip")

            # ----- Copy other non-obs keys -----
            for key in grp_in.keys():
                if key not in ("obs", "actions", "rewards"):
                    try:
                        grp_in.copy(key, grp_out)
                    except Exception:
                        new_grp = grp_out.create_group(key)
                        for k, v in grp_in[key].attrs.items():
                            new_grp.attrs[k] = v

            print(f"  Original length: {T}, Kept steps: {N_kept}")

    print("\nDone. Filtered dataset written to:", output_file)


def shift_actions_by_k(input_path, output_path, k=1):
    """
    Replace the action at time t by the action at time t+k.
    For the last k steps where t+k exceeds the total length, 
    the last valid action is repeated.
    """
    
    input_path = Path(input_path)
    output_path = Path(output_path)

    with h5py.File(input_path, "r") as fin, h5py.File(output_path, "w") as fout:

        # Copy attributes from root data group
        fout.create_group("data")
        for key, val in fin["data"].attrs.items():
            fout["data"].attrs[key] = val

        data_in = fin["data"]
        data_out = fout["data"]

        for demo_name in data_in.keys():
            print(f"Processing {demo_name}...")

            demo_in = data_in[demo_name]
            demo_out = data_out.create_group(demo_name)

            # Get the original actions
            original_actions = demo_in["actions"][:]
            T = original_actions.shape[0]

            # Create shifted actions
            shifted_actions = np.zeros_like(original_actions)
            
            for t in range(T):
                if t + k < T:
                    # Use action at t+k
                    shifted_actions[t] = original_actions[t + k]
                else:
                    shifted_actions[t] = original_actions[T - 1]  # Use the very last action

            # --- Actions (shifted) ---
            demo_out.create_dataset("actions", data=shifted_actions)

            # --- Rewards (unchanged) ---
            rewards_ds = demo_in["rewards"][:]
            demo_out.create_dataset("rewards", data=rewards_ds)

            # --- Observations (unchanged) ---
            obs_out = demo_out.create_group("obs")
            obs_in = demo_in["obs"]

            for obs_key in obs_in.keys():
                arr = obs_in[obs_key][:]   # copy all observations without modification
                obs_out.create_dataset(obs_key, data=arr, compression="gzip")

        print(f"\n🎉 Finished! Saved action-shifted dataset to:\n{output_path}")
        print(f"Actions shifted by k={k} steps (action at time t now contains action from time t+k)")
        return output_path
    



def normalize_actions_jointwise(input_path, output_path):
    joint_mins = np.array([-np.pi, 0, 0, -np.pi/2, -np.pi/2, -np.pi,
                           0, 0, -np.pi, 0, 0, -np.pi/2, -np.pi/2, -np.pi,
                           0, 0])
    joint_maxs = np.array([np.pi, np.pi, 2.36, np.pi/2, np.pi/2, np.pi,
                           0.04, 0.04, np.pi, np.pi, 2.36, np.pi/2, np.pi/2, np.pi,
                           0.04, 0.04])

    with h5py.File(input_path, "r") as fin, h5py.File(output_path, "w") as fout:
        
        # 🔹 PRESERVE file-level and 'data' attributes, including env_args
        for k, v in fin.attrs.items():
            fout.attrs[k] = v

        fout.create_group("data")
        for k, v in fin["data"].attrs.items():
            fout["data"].attrs[k] = v

        data_in = fin["data"]
        data_out = fout["data"]

        for demo_name in data_in.keys():
            print(f"Processing {demo_name}...")
            demo_in = data_in[demo_name]
            demo_out = data_out.create_group(demo_name)

            actions = demo_in["actions"][:]
            actions_norm = 2 * (actions - joint_mins) / (joint_maxs - joint_mins) - 1
            demo_out.create_dataset("actions", data=actions_norm)

            if "rewards" in demo_in:
                demo_out.create_dataset("rewards", data=demo_in["rewards"][:])

            obs_in = demo_in["obs"]
            obs_out = demo_out.create_group("obs")
            for obs_key in obs_in.keys():
                obs_out.create_dataset(obs_key, data=obs_in[obs_key][:], compression="gzip")

        print(f"\n🎉 Finished! Saved normalized dataset to:\n{output_path}")
        return output_path


if __name__ == "__main__":
    process_dataset(INPUT_FILE, OUTPUT_PROCESSED_FILE, THRESHOLD)
    shift_actions_by_k(OUTPUT_PROCESSED_FILE, OUTPUT_SHIFTED, k = 5)
    normalize_actions_jointwise(OUTPUT_SHIFTED, OUTPUT_NORM_FILE)
