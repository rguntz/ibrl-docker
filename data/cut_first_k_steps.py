import h5py
from pathlib import Path

def cut_first_steps(input_path, output_path, cut_first_n=15):
    """
    Remove the first `cut_first_n` steps from each demo in the HDF5 dataset.
    """

    input_path = Path(input_path)
    output_path = Path(output_path)
    output_path.parent.mkdir(parents=True, exist_ok=True)

    with h5py.File(input_path, "r") as fin, h5py.File(output_path, "w") as fout:

        # Copy data group and its attributes
        fout.create_group("data")
        for key, val in fin["data"].attrs.items():
            fout["data"].attrs[key] = val

        data_in = fin["data"]
        data_out = fout["data"]

        for demo_name in data_in.keys():
            print(f"Processing {demo_name}...")

            demo_in = data_in[demo_name]
            demo_out = data_out.create_group(demo_name)

            T = demo_in["actions"].shape[0]

            # Compute start index after cutting
            start_idx = min(cut_first_n, T - 1)
            idx = range(start_idx, T)

            # --- Actions ---
            demo_out.create_dataset("actions", data=demo_in["actions"][idx])

            # --- Rewards ---
            demo_out.create_dataset("rewards", data=demo_in["rewards"][idx])

            # --- Observations ---
            obs_out = demo_out.create_group("obs")
            obs_in = demo_in["obs"]
            for obs_key in obs_in.keys():
                obs_out.create_dataset(obs_key, data=obs_in[obs_key][idx], compression="gzip")

        print(f"\n🎉 Finished! Saved dataset with first {cut_first_n} steps removed to:\n{output_path}")
        return output_path


if __name__ == "__main__":
    input_file = "cube_picking_and_placing/dataset_200steps_actions16_shifted_5.hdf5"
    output_file = "cube_picking_and_placing/dataset_200steps_actions16_shifted_5_cut_15.hdf5"
    cut_first_steps(input_file, output_file)
