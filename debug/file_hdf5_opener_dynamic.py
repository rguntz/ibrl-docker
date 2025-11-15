import h5py
import argparse

# Parse command-line argument
parser = argparse.ArgumentParser()
parser.add_argument("--demo", type=int, default=0, help="Demo number to load")
args = parser.parse_args()

demo_name = f"demo_{args.demo}"

# Open file
dataset_path = "/home/qtf5422/Desktop/AIRE/ibrl-docker/data/dataset.hdf5"
f = h5py.File(dataset_path, "r")

print("Top-level keys:", list(f.keys()))

data_group = f["data"]
print("Episodes:", list(data_group.keys()))

# Load specified demo
demo_group = data_group[demo_name]
print("Episode datasets:", list(demo_group.keys()))

obs_group = demo_group["obs"]
print("Observation keys:", list(obs_group.keys()))

# Access image data
cam_data = obs_group["cam_high_image"]
print("Shape of cam_high_image:", cam_data.shape)
