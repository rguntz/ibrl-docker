import h5py
import numpy as np
from pathlib import Path

def normalize_actions(input_path, output_path):
    """
    Normalize actions in the HDF5 dataset to [-1, 1] by dividing by pi.
    Assumes all demos already have the desired fixed length (e.g., 200 steps).
    """

    input_path = Path(input_path)
    output_path = Path(output_path)

    with h5py.File(input_path, "r") as fin, h5py.File(output_path, "w") as fout:

        # Create root group and copy attributes
        fout.create_group("data")
        for key, val in fin["data"].attrs.items():
            fout["data"].attrs[key] = val

        data_in = fin["data"]
        data_out = fout["data"]

        for demo_name in data_in.keys():
            print(f"Processing {demo_name}...")

            demo_in = data_in[demo_name]
            demo_out = data_out.create_group(demo_name)

            # --- Actions: normalize by pi ---
            actions_ds = demo_in["actions"][:] / np.pi
            demo_out.create_dataset("actions", data=actions_ds)

            # --- Rewards ---
            rewards_ds = demo_in["rewards"][:]
            demo_out.create_dataset("rewards", data=rewards_ds)

            # --- Observations ---
            obs_out = demo_out.create_group("obs")
            obs_in = demo_in["obs"]

            for obs_key in obs_in.keys():
                arr = obs_in[obs_key][:]
                obs_out.create_dataset(obs_key, data=arr, compression="gzip")

        print(f"\n🎉 Finished! Saved normalized dataset to:\n{output_path}")
        return output_path
    


def normalize_actions_jointwise(input_path, output_path):
    """
    Normalize actions in the HDF5 dataset to [-1, 1] using joint-specific limits.
    Supports 16-dimensional actions for left+right arm + grippers.
    """

    # Joint limits
    joint_mins = np.array([-np.pi, 0, 0, -np.pi/2, -np.pi/2, -np.pi,
                           0, 0, -np.pi, 0, 0, -np.pi/2, -np.pi/2, -np.pi,
                           0, 0])
    joint_maxs = np.array([np.pi, np.pi, 2.36, np.pi/2, np.pi/2, np.pi,
                           0.04, 0.04, np.pi, np.pi, 2.36, np.pi/2, np.pi/2, np.pi,
                           0.04, 0.04])

    input_path = Path(input_path)
    output_path = Path(output_path)

    with h5py.File(input_path, "r") as fin, h5py.File(output_path, "w") as fout:

        # Create root group and copy attributes
        fout.create_group("data")
        for key, val in fin["data"].attrs.items():
            fout["data"].attrs[key] = val

        data_in = fin["data"]
        data_out = fout["data"]

        for demo_name in data_in.keys():
            print(f"Processing {demo_name}...")

            demo_in = data_in[demo_name]
            demo_out = data_out.create_group(demo_name)

            # --- Actions: joint-wise normalization ---
            actions = demo_in["actions"][:]
            # Normalize: (a - min) / (max - min) → [0,1], then scale to [-1,1]
            actions_norm = 2 * (actions - joint_mins) / (joint_maxs - joint_mins) - 1
            demo_out.create_dataset("actions", data=actions_norm)

            # --- Rewards ---
            rewards_ds = demo_in["rewards"][:]
            demo_out.create_dataset("rewards", data=rewards_ds)

            # --- Observations ---
            obs_out = demo_out.create_group("obs")
            obs_in = demo_in["obs"]

            for obs_key in obs_in.keys():
                arr = obs_in[obs_key][:]
                obs_out.create_dataset(obs_key, data=arr, compression="gzip")

        print(f"\n🎉 Finished! Saved normalized dataset to:\n{output_path}")
        return output_path



if __name__ == "__main__":

    """
    input_file = "cube_picking_and_placing/dataset_200steps_actions16_shifted_5.hdf5"
    output_file = "cube_picking_and_placing/dataset_200steps_actions16_shifted_5_norm.hdf5"
    normalize_actions(input_file, output_file)

    input_file = "cube_picking_and_placing/dataset_200steps_actions16_shifted_5.hdf5"
    output_file = "cube_picking_and_placing/dataset_200steps_actions16_shifted_5_normalized_minmax.hdf5"
    normalize_actions_jointwise(input_file, output_file)

    
    input_file = "cube_picking_and_placing/dataset_200steps_actions16.hdf5"
    output_file = "cube_picking_and_placing/dataset_200steps_actions16_norm.hdf5"
    normalize_actions(input_file, output_file)

    input_file = "cube_picking_and_placing/dataset_200steps_actions16.hdf5"
    output_file = "cube_picking_and_placing/dataset_200steps_actions16_normalized_minmax.hdf5"
    normalize_actions_jointwise(input_file, output_file)
    """

    input_file = "cube_picking_and_placing/dataset_200steps_actions16_shifted_5_cut_15.hdf5"
    output_file = "cube_picking_and_placing/dataset_200steps_actions16_shifted_5_cut_15_normalized_minmax.hdf5"
    normalize_actions_jointwise(input_file, output_file)