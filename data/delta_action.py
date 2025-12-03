import h5py
import numpy as np
from pathlib import Path

def subtract_state_from_actions(input_path, output_path):
    """
    Modify the dataset such that each action becomes:
        new_action[t] = action[t] - prop[t][:16]
    """
    
    input_path = Path(input_path)
    output_path = Path(output_path)

    with h5py.File(input_path, "r") as fin, h5py.File(output_path, "w") as fout:

        # Copy root-level attributes if any
        fout.create_group("data")
        for key, val in fin["data"].attrs.items():
            fout["data"].attrs[key] = val

        data_in = fin["data"]
        data_out = fout["data"]

        for demo_name in data_in.keys():
            print(f"Processing {demo_name}...")

            demo_in = data_in[demo_name]
            demo_out = data_out.create_group(demo_name)

            # --- Observations ---
            obs_in = demo_in["obs"]
            obs_out = demo_out.create_group("obs")

            # Copy all observations
            for obs_key in obs_in.keys():
                arr = obs_in[obs_key][:]
                obs_out.create_dataset(obs_key, data=arr, compression="gzip")

            # Get the actions and the first 16 values of prop
            actions = demo_in["actions"][:]
            prop_state = obs_in["prop"][:, :16]

            # Subtract state from action
            modified_actions = actions - prop_state

            # Save modified actions
            demo_out.create_dataset("actions", data=modified_actions)

            # --- Rewards (unchanged) ---
            rewards_ds = demo_in["rewards"][:]
            demo_out.create_dataset("rewards", data=rewards_ds)

        print(f"\n🎉 Finished! Saved dataset with actions - state to:\n{output_path}")
        return output_path


if __name__ == "__main__":
    input_file = "cube_picking_and_placing/dataset_200steps_actions16.hdf5"
    output_file = "cube_picking_and_placing/dataset_200steps_actions16_delta_action.hdf5"
    subtract_state_from_actions(input_file, output_file)
