import h5py
import numpy as np
from pathlib import Path


def downsample_to_fixed_length(input_path, output_path, target_steps=200):
    """
    Downsample each demo in the HDF5 dataset to exactly `target_steps` steps.
    The function creates uniform indices per demo so that the output length equals target_steps.
    """

    input_path = Path(input_path)
    output_path = Path(output_path)
    output_path.parent.mkdir(parents=True, exist_ok=True)

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

            # Determine episode length
            T = demo_in["actions"].shape[0]

            # If episode is shorter than required target steps → keep all
            if T <= target_steps:
                idx = np.arange(T)
            else:
                # Uniform sampling from 0 to T-1
                idx = np.linspace(0, T - 1, target_steps, dtype=int)

            # --- Actions ---
            actions_ds = demo_in["actions"][idx]
            demo_out.create_dataset("actions", data=actions_ds)

            # --- Rewards ---
            rewards_ds = demo_in["rewards"][idx]
            demo_out.create_dataset("rewards", data=rewards_ds)

            # --- Observations ---
            obs_out = demo_out.create_group("obs")
            obs_in = demo_in["obs"]

            for obs_key in obs_in.keys():
                arr = obs_in[obs_key][idx]   # maintains correct slicing for images and props   
                obs_out.create_dataset(obs_key, data=arr, compression="gzip")

        print(f"\n🎉 Finished! Saved fixed-length dataset to:\n{output_path}")
        return output_path


if __name__ == "__main__":
    input_file = "cube_picking_and_placing/dataset_actions16.hdf5"
    output_file = "cube_picking_and_placing/dataset_200steps_actions16.hdf5"
    downsample_to_fixed_length(input_file, output_file)
