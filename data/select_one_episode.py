import h5py
import numpy as np
from pathlib import Path

def save_first_demo(input_path, output_path, keep_stride=None):
    """
    Copies ONLY the first demo from the HDF5 dataset.
    Optionally downsamples using keep_stride.
    """

    input_path = Path(input_path)
    output_path = Path(output_path)
    output_path.parent.mkdir(parents=True, exist_ok=True)

    with h5py.File(input_path, "r") as fin, h5py.File(output_path, "w") as fout:

        # create "data" group
        fout.create_group("data")

        # copy global attributes
        for k, v in fin["data"].attrs.items():
            fout["data"].attrs[k] = v

        data_in = fin["data"]
        data_out = fout["data"]

        # ---- GET FIRST DEMO NAME ----
        demo_names = list(data_in.keys())
        if len(demo_names) == 0:
            raise ValueError("No demos found in input dataset!")

        first_demo = demo_names[0]
        print(f"Saving ONLY the first demo: {first_demo}")

        demo_in = data_in[first_demo]
        demo_out = data_out.create_group(first_demo)

        # ---- ACTIONS ----
        actions = demo_in["actions"][:]
        actions_ds = actions[::keep_stride] if keep_stride else actions
        demo_out.create_dataset("actions", data=actions_ds)

        # ---- REWARDS ----
        rewards = demo_in["rewards"][:]
        rewards_ds = rewards[::keep_stride] if keep_stride else rewards
        demo_out.create_dataset("rewards", data=rewards_ds)

        # ---- OBS ----
        obs_in = demo_in["obs"]
        obs_out = demo_out.create_group("obs")

        for obs_key in obs_in.keys():
            arr = obs_in[obs_key][:]

            arr_ds = arr[::keep_stride] if keep_stride else arr

            obs_out.create_dataset(
                obs_key,
                data=arr_ds,
                compression="gzip"
            )

    print(f"\n🎉 Saved only the first demo to:\n{output_path}")
    return output_path

input_file = "cube_picking_and_placing/dataset_200steps_actions16_shifted_5_normalized_minmax.hdf5"
output_file = "cube_picking_and_placing/dataset_200steps_actions16_shifted_5_normalized_minmax_one_episode.hdf5"

save_first_demo(input_file, output_file)
