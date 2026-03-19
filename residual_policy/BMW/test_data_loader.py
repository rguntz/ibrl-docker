import sys
sys.path.append("/home/qtf5422/Desktop/AIRE/ibrl/lerobot/src")
sys.path.append("/home/qtf5422/Desktop/AIRE/ibrl/lerobot-utils/src")
sys.path.append("/home/qtf5422/Desktop/AIRE/ibrl/storage-adapter/src")

import torch 
from lerobot.configs.types import FeatureType
from lerobot.datasets.lerobot_dataset import LeRobotDataset, LeRobotDatasetMetadata
from lerobot.datasets.utils import dataset_to_policy_features
import matplotlib.pyplot as plt
from lerobot.policies.diffusion.configuration_diffusion import DiffusionConfig

def replay_video():

    plt.ion()  # Turn on interactive mode
    fig, ax = plt.subplots()

    for i in range(0, 1001):
        img = dataset[i]['video'].permute(1, 2, 0)  # Convert from CxHxW to HxWxC
        ax.clear()  # Clear previous image
        ax.imshow(img)
        ax.set_title(f"Frame {i}")
        ax.axis('off')
        plt.pause(0.03)  # Pause in seconds (adjust for speed)
        
    plt.ioff()  # Turn off interactive mode
    plt.show()

def inspect_batch(batch):
    print("\n=== Batch inspection ===")
    for key, value in batch.items():
        print(f"\nKey: '{key}'")

        if isinstance(value, torch.Tensor):
            print(f"  Type      : torch.Tensor")
            print(f"  Shape     : {tuple(value.shape)}")
            print(f"  Dtype     : {value.dtype}")
            print(f"  Device    : {value.device}")

            # Heuristic: image-like tensors
            if value.ndim >= 3:
                # common cases:
                # (B, T, C, H, W) or (B, C, H, W)
                shape = value.shape
                channels = None

                if value.ndim == 5:  # (B, T, C, H, W)
                    channels = shape[2]
                elif value.ndim == 4:  # (B, C, H, W)
                    channels = shape[1]

                if channels is not None:
                    if channels == 3:
                        print("  Detected  : RGB image")
                    elif channels == 1:
                        print("  Detected  : Grayscale image")
                    else:
                        print(f"  Detected  : Image-like (channels={channels})")
                else:
                    print("  Detected  : High-dim tensor (possibly image sequence)")

            else:
                print("  Detected  : State / action / low-dim tensor")

        else:
            print(f"  Type      : {type(value)} (non-tensor)")


import pyarrow.parquet as pq

print("------------------------------------------------------------")
table = pq.read_table("/home/qtf5422/Desktop/AIRE/residual-offpolicy-rl/arm=trossen_dual/task=entire_ecu_assembly/data/chunk-000/episode_000001.parquet")
print(table.schema)
print(table.column_names)
print("------------------------------------------------------------")



device = torch.device("cuda")
dataset_metadata = LeRobotDatasetMetadata("/home/qtf5422/Desktop/AIRE/residual-offpolicy-rl/arm=trossen_dual/task=entire_ecu_assembly")
features = dataset_to_policy_features(dataset_metadata.features)
output_features = {key: ft for key, ft in features.items() if ft.type is FeatureType.ACTION}
input_features = {key: ft for key, ft in features.items() if key not in output_features}
print("input feature : ", input_features)
print("output_features : ", output_features)

dataset_stats=dataset_metadata.stats
print("dataset_stats : ", dataset_stats.keys())

# Policies are initialized with a configuration class, in this case `DiffusionConfig`. For this example,
# we'll just use the defaults and so no arguments other than input/output features need to be passed.
cfg = DiffusionConfig(input_features=input_features, output_features=output_features)

# Another policy-dataset interaction is with the delta_timestamps. Each policy expects a given number frames
# which can differ for inputs, outputs and rewards (if there are some).
delta_timestamps = {
    # Visual observations
    "observation.images.video": [i / dataset_metadata.fps for i in cfg.observation_delta_indices],
    "observation.images.wrist_video": [i / dataset_metadata.fps for i in cfg.observation_delta_indices],
    "observation.images.video_2": [i / dataset_metadata.fps for i in cfg.observation_delta_indices],
    "observation.images.wrist_video_2": [i / dataset_metadata.fps for i in cfg.observation_delta_indices],

    # State observations
    "observation.state": [i / dataset_metadata.fps for i in cfg.observation_delta_indices],
    # Actions
    "action": [i / dataset_metadata.fps for i in cfg.action_delta_indices],
}

# We can then instantiate the dataset with these delta_timestamps configuration.
dataset = LeRobotDataset("/home/qtf5422/Desktop/AIRE/residual-offpolicy-rl/arm=trossen_dual/task=entire_ecu_assembly", delta_timestamps=delta_timestamps)

dataloader = torch.utils.data.DataLoader(
    dataset,
    num_workers=4,
    batch_size=64,
    shuffle=True,
    pin_memory=device.type != "cpu",
    drop_last=True,
)

# Run training loop.
step = 0
done = False
for batch in dataloader:
    batch = {k: (v.to(device) if isinstance(v, torch.Tensor) else v) for k, v in batch.items()}
    inspect_batch(batch)
    break
