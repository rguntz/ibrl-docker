import h5py
import numpy as np
from pathlib import Path


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


if __name__ == "__main__":
    input_file = "cube_picking_and_placing/dataset_200steps_actions16.hdf5"
    output_file = "cube_picking_and_placing/dataset_200steps_actions16_shifted_5.hdf5"
    shift_actions_by_k(input_file, output_file, k=5)