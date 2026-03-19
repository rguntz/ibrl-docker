#!/usr/bin/env python
"""Debug script to find corrupted frames in LeRobot dataset."""

import sys
import torch
from pathlib import Path
from tqdm import tqdm

from lerobot.common.datasets.lerobot_dataset import LeRobotDataset
from lerobot.common.datasets.transforms import ImageTransforms, ImageTransformsConfig

DATASET_PATH = "/home/qtf5422/Desktop/AIRE/residual-offpolicy-rl/arm=trossen_dual/task=entire_ecu_assembly"

print(f"🔍 Scanning dataset: {DATASET_PATH}")

# Match the exact initialization from your training script
image_transforms_config = ImageTransformsConfig(enable=True)
image_transforms = ImageTransforms(image_transforms_config)

dataset = LeRobotDataset(
    DATASET_PATH,
    delta_timestamps=None,  # We'll check raw data first
    download_videos=True,
    image_transforms=image_transforms,
)

print(f"✅ Dataset loaded. Total frames: {len(dataset)}")
print("🚀 Starting sequential scan (shuffling disabled)...")

# Wrap range with tqdm for progress monitoring
for idx in tqdm(range(len(dataset)), desc="Scanning frames", unit="frame"):
    try:
        item = dataset[idx]
        
        # Check every key in the returned dictionary for None
        for key, value in item.items():
            if value is None:
                print(f"\n❌ CRITICAL ERROR at Index {idx}:")
                print(f"   Key '{key}' is None!")
                sys.exit(1)
            
            # Check for NaN values in tensors
            if isinstance(value, torch.Tensor):
                if torch.isnan(value).any():
                    print(f"\n⚠️  WARNING at Index {idx}: Key '{key}' contains NaN values")
                    
    except Exception as e:
        print(f"\n💥 EXCEPTION at Index {idx}:")
        print(f"   {type(e).__name__}: {e}")
        import traceback
        traceback.print_exc()
        sys.exit(1)

print("\n✅ Dataset scan complete. No None values found.")