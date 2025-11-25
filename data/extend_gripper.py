import h5py
import numpy as np
from pathlib import Path


def expand_gripper_actions(input_path, output_path):
    """
    Expands 14-dim actions to 16-dim by duplicating the gripper dimension
    for symmetric parts for a 2-arm setup.
    """

    input_path = Path(input_path)
    output_path = Path(output_path)
    output_path.parent.mkdir(parents=True, exist_ok=True)

    with h5py.File(input_path, "r") as fin, h5py.File(output_path, "w") as fout:

        # Copy global attributes from "data"
        fout.create_group("data")
        for attr_key, attr_val in fin["data"].attrs.items():
            fout["data"].attrs[attr_key] = attr_val

        data_in = fin["data"]
        data_out = fout["data"]

        for demo_name in data_in.keys():
            print(f"Processing {demo_name}...")

            demo_in = data_in[demo_name]
            demo_out = data_out.create_group(demo_name)

            # Expand actions from 14 → 16
            actions = demo_in["actions"][:]
            actions_expanded = np.zeros((actions.shape[0], 16), dtype=actions.dtype)

            # Left arm
            actions_expanded[:, :6] = actions[:, :6]  # joints
            actions_expanded[:, 6] = actions[:, 6]   # gripper1
            actions_expanded[:, 7] = actions[:, 6]   # gripper2 (duplicated)

            # Right arm
            actions_expanded[:, 8:14] = actions[:, 7:13]  # joints
            actions_expanded[:, 14] = actions[:, 13]      # gripper1
            actions_expanded[:, 15] = actions[:, 13]      # gripper2

            demo_out.create_dataset("actions", data=actions_expanded)

            # Copy rewards
            demo_out.create_dataset("rewards", data=demo_in["rewards"][:])

            # Copy observations
            obs_in = demo_in["obs"]
            obs_out = demo_out.create_group("obs")
            for obs_key in obs_in.keys():
                obs_out.create_dataset(obs_key, data=obs_in[obs_key][:], compression="gzip")

        print(f"\n🎉 Finished! Saved expanded dataset to:\n{output_path}")
        return output_path


if __name__ == "__main__":
    input_file = "cube_picking_and_placing/dataset.hdf5"  # or original dataset
    output_file = "cube_picking_and_placing/dataset_actions16.hdf5"
    expand_gripper_actions(input_file, output_file)
