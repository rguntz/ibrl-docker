import h5py
import numpy as np
import os
from pathlib import Path
import json
from scipy.spatial.transform import Rotation as R
import h5py
import torch
import h5py
import cv2
import numpy as np
from pathlib import Path
import matplotlib.pyplot as plt
import numpy as np

def inspect_keys(file) : 
    with h5py.File(file, "r") as f:
        print(list(f.keys()))

        f_data = f["data"]
        print("f_data keys : ", f_data.keys())
        print("the number of demos is : ", len(f_data))

        f_demo_0 = f_data["demo_1"]
        print("f_demo_0 keys : ", f_demo_0.keys())

        action = f_demo_0["actions"]
        print("action size : ", action.shape)
        print("first action : ", action[0])
        print("last action : ", action[-1])

        action_min = action[:].min()  # Convert to array for min/max
        action_max = action[:].max()
        print("Minimum action value : ", action_min)
        print("Maximum action value : ", action_max)

        obs = f_demo_0["obs"]
        print("obs keys : ", obs.keys())

        robot0_eef_pos = obs["robot0_eef_pos"]
        print("prop keys : ", robot0_eef_pos.shape)
        print("proprio first episode : ", robot0_eef_pos[0])

        robot0_eef_quat = obs["robot0_eef_quat"]
        print("prop keys : ", robot0_eef_quat.shape)
        print("proprio first episode : ", robot0_eef_quat[0])

        robot0_gripper_qpos = obs["robot0_gripper_qpos"]
        print("prop keys : ", robot0_gripper_qpos.shape)
        print("proprio first episode : ", robot0_gripper_qpos[0])

        # --- NEW: Check min/max for proprioception values ---
        print("\n=== Proprioception Value Ranges ===")

        # robot0_eef_pos
        pos = robot0_eef_pos[:]
        print("robot0_eef_pos min:", pos.min(), "max:", pos.max())

        # robot0_eef_quat
        quat = robot0_eef_quat[:]
        print("robot0_eef_quat min:", quat.min(), "max:", quat.max())

        # robot0_gripper_qpos
        grip = robot0_gripper_qpos[:]
        print("robot0_gripper_qpos min:", grip.min(), "max:", grip.max())

        images = obs["cam_high_image"]
        print("the size of the images is : ", images.shape)

        print("image : ",np.min(images[0]), np.max(images[0]))

def modify_hdf5_file(input_file, output_file):
    """
    Modify HDF5 file to:
    1. Remove 'action_quat' key from each demo
    2. Rename 'dones' to 'terminals'
    3. Rename 'obs' to 'observations'
    """
    with h5py.File(input_file, "r") as f_in:
        with h5py.File(output_file, "w") as f_out:
            # Create the data group
            data_out = f_out.create_group("data")
            
            # Get all demo keys
            demo_keys = list(f_in["data"].keys())
            print(f"Processing {len(demo_keys)} demos...")
            
            for demo_key in demo_keys:
                print(f"Processing {demo_key}...")
                demo_in = f_in["data"][demo_key]
                demo_out = data_out.create_group(demo_key)
                
                # Copy all keys except action_quat, and rename as needed
                for key in demo_in.keys():
                    if key == "action_quat":
                        # Skip this key (don't copy it)
                        print(f"  Skipping {key}")
                        continue
                    elif key == "dones":
                        # Rename to terminals
                        print(f"  Renaming {key} -> terminals")
                        demo_out.create_dataset("terminals", data=demo_in[key][:])
                    elif key == "obs":
                        # Rename to observations and copy all sub-keys
                        print(f"  Renaming {key} -> observations")
                        obs_out = demo_out.create_group("observations")
                        for obs_key in demo_in[key].keys():
                            obs_out.create_dataset(obs_key, data=demo_in[key][obs_key][:])
                    else:
                        # Copy as-is
                        print(f"  Copying {key}")
                        demo_out.create_dataset(key, data=demo_in[key][:])
            
            print(f"\nSuccessfully created {output_file}")

def verify_modifications(file):
    """Verify the modifications were applied correctly"""
    print("\n=== Verifying modifications ===")
    with h5py.File(file, "r") as f:
        demo_0 = f["data"]["demo_0"]
        print(f"Keys in demo_0: {list(demo_0.keys())}")
        
        # Check that action_quat is gone
        assert "action_quat" not in demo_0.keys(), "ERROR: action_quat still exists!"
        print("✓ action_quat successfully removed")
        
        # Check that terminals exists
        assert "terminals" in demo_0.keys(), "ERROR: terminals not found!"
        print("✓ dones successfully renamed to terminals")
        
        # Check that observations exists
        assert "observations" in demo_0.keys(), "ERROR: observations not found!"
        print("✓ obs successfully renamed to observations")
        
        # Check that obs is gone
        assert "obs" not in demo_0.keys(), "ERROR: obs still exists!"
        print("✓ obs key successfully replaced")
        
        # Verify observations structure
        obs_keys = list(demo_0["observations"].keys())
        print(f"\nObservations keys: {obs_keys}")

def inspect_keys_vision(file) : 
    with h5py.File(file, "r") as f:
        print(list(f.keys()))

        f_data = f["data"]
        print("f_data keys : ", f_data.keys())
        print("the number of demos is : ", len(f_data))

        f_demo_0 = f_data["demo_1"]
        print("f_demo_0 keys : ", f_demo_0.keys())

        action = f_demo_0["actions"]
        print("action size : ", action.shape)
        print("first action : ", action[0])
        print("last action : ", action[-1])

        action_min = action[:].min()  # Convert to array for min/max
        action_max = action[:].max()
        print("Minimum action value : ", action_min)
        print("Maximum action value : ", action_max)

        obs = f_demo_0["observations"]
        print("obs keys : ", obs.keys())

        robot0_eef_pos = obs["robot0_eef_pos"]
        print("prop keys : ", robot0_eef_pos.shape)
        print("proprio first episode : ", robot0_eef_pos[0])

        robot0_eef_quat = obs["robot0_eef_quat"]
        print("prop keys : ", robot0_eef_quat.shape)
        print("proprio first episode : ", robot0_eef_quat[0])

        robot0_gripper_qpos = obs["robot0_gripper_qpos"]
        print("prop keys : ", robot0_gripper_qpos.shape)
        print("proprio first episode : ", robot0_gripper_qpos[0])

        # --- NEW: Check min/max for proprioception values ---
        print("\n=== Proprioception Value Ranges ===")

        # robot0_eef_pos
        pos = robot0_eef_pos[:]
        print("robot0_eef_pos min:", pos.min(), "max:", pos.max())

        # robot0_eef_quat
        quat = robot0_eef_quat[:]
        print("robot0_eef_quat min:", quat.min(), "max:", quat.max())

        # robot0_gripper_qpos
        grip = robot0_gripper_qpos[:]
        print("robot0_gripper_qpos min:", grip.min(), "max:", grip.max())

        images = obs["cam_high_image"]
        print("the size of the images is : ", images.shape)

        print("image : ",np.min(images[0]), np.max(images[0]))


import h5py
import numpy as np

def concatenate_all_demos(input_file, output_file):
    """
    Concatenate all demos into a single demo called 'demo_0'
    """
    with h5py.File(input_file, "r") as f_in:
        with h5py.File(output_file, "w") as f_out:
            # Create the data group
            data_out = f_out.create_group("data")
            demo_out = data_out.create_group("demo_0")
            
            # Get all demo keys and sort them
            demo_keys = sorted(f_in["data"].keys(), key=lambda x: int(x.split('_')[1]))
            print(f"Concatenating {len(demo_keys)} demos...")
            
            # Initialize lists to store concatenated data
            concatenated_data = {}
            obs_data = {}
            
            # First pass: collect all data
            for i, demo_key in enumerate(demo_keys):
                print(f"Processing {demo_key} ({i+1}/{len(demo_keys)})...")
                demo = f_in["data"][demo_key]
                
                # Handle top-level datasets (actions, terminals, rewards)
                for key in demo.keys():
                    if key == "observations":
                        # Handle observations separately
                        for obs_key in demo["observations"].keys():
                            if obs_key not in obs_data:
                                obs_data[obs_key] = []
                            obs_data[obs_key].append(demo["observations"][obs_key][:])
                    else:
                        # Regular datasets
                        if key not in concatenated_data:
                            concatenated_data[key] = []
                        concatenated_data[key].append(demo[key][:])
            
            print("\nConcatenating arrays...")
            
            # Write concatenated top-level datasets
            for key, data_list in concatenated_data.items():
                concatenated_array = np.concatenate(data_list, axis=0)
                print(f"  {key}: shape {concatenated_array.shape}")
                demo_out.create_dataset(key, data=concatenated_array)
            
            # Write concatenated observations
            obs_out = demo_out.create_group("observations")
            for obs_key, data_list in obs_data.items():
                concatenated_array = np.concatenate(data_list, axis=0)
                print(f"  observations/{obs_key}: shape {concatenated_array.shape}")
                obs_out.create_dataset(obs_key, data=concatenated_array)
            
            print(f"\nSuccessfully created {output_file} with single concatenated demo")

def verify_concatenation(original_file, concatenated_file):
    """Verify the concatenation worked correctly"""
    print("\n=== Verifying concatenation ===")
    
    with h5py.File(original_file, "r") as f_orig:
        with h5py.File(concatenated_file, "r") as f_concat:
            # Count original demos
            original_demos = list(f_orig["data"].keys())
            num_original_demos = len(original_demos)
            print(f"Original file had {num_original_demos} demos")
            
            # Check concatenated file has only one demo
            concat_demos = list(f_concat["data"].keys())
            assert len(concat_demos) == 1, f"Expected 1 demo, found {len(concat_demos)}"
            print(f"✓ Concatenated file has 1 demo: {concat_demos[0]}")
            
            # Calculate expected total length
            total_length = 0
            for demo_key in original_demos:
                demo_length = f_orig["data"][demo_key]["actions"].shape[0]
                total_length += demo_length
            
            print(f"\nExpected total length: {total_length}")
            
            # Check concatenated demo length
            concat_demo = f_concat["data"]["demo_0"]
            concat_length = concat_demo["actions"].shape[0]
            print(f"Concatenated demo length: {concat_length}")
            
            assert concat_length == total_length, "Length mismatch!"
            print("✓ Lengths match!")
            
            # Display structure
            print(f"\nConcatenated demo structure:")
            print(f"  Keys: {list(concat_demo.keys())}")
            print(f"  actions shape: {concat_demo['actions'].shape}")
            print(f"  terminals shape: {concat_demo['terminals'].shape}")
            print(f"  rewards shape: {concat_demo['rewards'].shape}")
            print(f"  observations keys: {list(concat_demo['observations'].keys())}")
            
            # Show a sample observation shape
            for obs_key in list(concat_demo['observations'].keys())[:3]:
                print(f"    {obs_key} shape: {concat_demo['observations'][obs_key].shape}")


import h5py
import numpy as np
import cv2

def convert_hdf5_to_npz(input_file, output_file, camera_key='cam_right_wrist_image', target_size=(84, 84)):
    """
    Convert concatenated HDF5 file to NPZ format with:
    - observations: resized images from cam_right_wrist_image (N, 84, 84, 3)
    - actions: (N, 4) - taking first 4 dimensions of actions
    - rewards: (N, 1)
    - terminals: (N, 1)
    - next_observations: (N, 84, 84, 3) - shifted observations
    """
    with h5py.File(input_file, "r") as f:
        demo = f["data"]["demo_0"]
        
        print("Loading data from HDF5...")
        
        # Get images from cam_right_wrist_image and resize them
        images = demo["observations"][camera_key][:]  # Shape: (N, 3, 128, 128)
        print(f"Original images shape: {images.shape}")
        
        N = images.shape[0]
        
        # Resize images to 84x84 and transpose to (N, 84, 84, 3)
        print(f"Resizing images to {target_size}...")
        observations = np.zeros((N, target_size[0], target_size[1], 3), dtype=np.uint8)
        
        for i in range(N):
            # Convert from (3, 128, 128) to (128, 128, 3)
            img = np.transpose(images[i], (1, 2, 0))
            # Resize to (84, 84, 3)
            resized = cv2.resize(img, target_size, interpolation=cv2.INTER_LINEAR)
            observations[i] = resized
            
            if (i + 1) % 1000 == 0:
                print(f"  Processed {i + 1}/{N} images")
        
        print(f"Resized observations shape: {observations.shape}")
        
        # Create next_observations (shift observations by 1)
        next_observations = np.zeros_like(observations)
        next_observations[:-1] = observations[1:]
        next_observations[-1] = observations[-1]  # Last one stays the same
        
        # Get actions - take first 4 dimensions
        actions_full = demo["actions"][:]
        print(f"Original actions shape: {actions_full.shape}")
        actions = actions_full[:, :4].astype(np.float32)
        print(f"Actions shape (first 4 dims): {actions.shape}")
        
        # Get rewards and reshape to (N, 1)
        rewards = demo["rewards"][:].reshape(-1, 1).astype(np.float32)
        print(f"Rewards shape: {rewards.shape}")
        
        # Get terminals and reshape to (N, 1)
        terminals = demo["terminals"][:].reshape(-1, 1).astype(np.float32)
        print(f"Terminals shape: {terminals.shape}")
        
        # Save to NPZ
        print(f"\nSaving to {output_file}...")
        np.savez(
            output_file,
            observations=observations,
            actions=actions,
            rewards=rewards,
            terminals=terminals,
            next_observations=next_observations,
            allow_pickle=np.array([])  # Empty array to match your expected output
        )
        
        print(f"Successfully created {output_file}")

def verify_npz(file):
    """Verify the NPZ file structure"""
    print("\n=== Verifying NPZ file ===")
    data = np.load(file)
    
    print("\nKeys and shapes:")
    for k in data.keys():
        print(f"  {k}: {data[k].shape}")
    
    print("\nData types:")
    for k in data.keys():
        if k != 'allow_pickle':
            print(f"  {k}: {data[k].dtype}")
    
    print("\nValue ranges:")
    print(f"  observations: min={data['observations'].min()}, max={data['observations'].max()}")
    print(f"  actions: min={data['actions'].min()}, max={data['actions'].max()}")
    print(f"  rewards: min={data['rewards'].min()}, max={data['rewards'].max()}")
    print(f"  terminals: min={data['terminals'].min()}, max={data['terminals'].max()}")
    
    # Check first and last observations are different
    print("\nSanity checks:")
    print(f"  First obs == Last obs: {np.array_equal(data['observations'][0], data['observations'][-1])}")
    print(f"  First next_obs == Second obs: {np.array_equal(data['next_observations'][0], data['observations'][1])}")



inspect_keys(file="dataset_1_tresholded_wr_shifted_delta_gripper_normed_cut_end_normalized.hdf5")

# Usage
input_file = "dataset_1_tresholded_wr_shifted_delta_gripper_normed_cut_end_normalized.hdf5"
output_file = "dataset_1_tresholded_wr_shifted_delta_gripper_normed_cut_end_normalized_modified.hdf5"

modify_hdf5_file(input_file, output_file)
verify_modifications(output_file)

inspect_keys_vision(file="dataset_1_tresholded_wr_shifted_delta_gripper_normed_cut_end_normalized_modified.hdf5")

# Usage
input_file = "dataset_1_tresholded_wr_shifted_delta_gripper_normed_cut_end_normalized_modified.hdf5"
output_file = "dataset_1_concatenated.hdf5"

concatenate_all_demos(input_file, output_file)
verify_concatenation(input_file, output_file)

# Usage
input_file = "dataset_1_concatenated.hdf5"
output_file = "forward.npz"

convert_hdf5_to_npz(input_file, output_file, target_size=(84, 84))
verify_npz(output_file)

