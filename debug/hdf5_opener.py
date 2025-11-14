import h5py

# Open the file in read-only mode
dataset_path = "/home/qtf5422/Desktop/AIRE/ibrl-docker/data/dataset.hdf5"
#dataset_path = "/home/qtf5422/Desktop/AIRE/ibrl-docker/release/data/robomimic/can/processed_data96.hdf5"

f = h5py.File(dataset_path, "r")

print("Top-level keys:", list(f.keys()))

data_group = f["data"]
print("Episodes:", list(data_group.keys()))

demo0 = data_group["demo_0"]
print("Episode datasets:", list(demo0.keys()))

obs_group = demo0["obs"]
print("Observation keys:", list(obs_group.keys()))

# Access the dataset for the camera
cam_data = obs_group["cam_high_image"]
#cam_data = obs_group["robot0_eye_in_hand_image"]

# Get the shape
print("Shape of cam_high_image:", cam_data.shape)

