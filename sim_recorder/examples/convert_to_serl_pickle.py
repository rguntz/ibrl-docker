#!/usr/bin/env python3
"""
Convert integrated_teleop_recorder episodes to SERL pickle format

SERL expects pickle files with list of transition dicts:
[
    {
        'observations': {'image': ..., 'state': ...},
        'actions': array,
        'rewards': float,
        'dones': bool
    },
    ...
]
"""

import numpy as np
import pickle as pkl
import json
from pathlib import Path
import argparse
from typing import List, Dict


def load_episode(episode_dir: Path) -> Dict:
    """Load episode data from integrated recorder format"""
    # Load observations
    obs_npz = np.load(episode_dir / 'observations.npz')
    obs_dict = {k: obs_npz[k] for k in obs_npz.keys()}
    
    # Load actions
    actions = np.load(episode_dir / 'actions.npy')
    
    # Load metadata
    with open(episode_dir / 'meta.json') as f:
        meta = json.load(f)
    
    return {
        'observations': obs_dict,
        'actions': actions,
        'meta': meta
    }


def convert_to_serl_format(episode_data: Dict, 
                           primary_camera: str = 'cam_high',
                           include_state: bool = True) -> List[Dict]:
    """
    Convert episode to SERL pickle format
    
    Args:
        episode_data: Data from load_episode()
        primary_camera: Which camera image to use as 'image' observation
        include_state: If True, include qpos/qvel as 'state'
    
    Returns:
        List of transition dicts
    """
    obs_dict = episode_data['observations']
    actions = episode_data['actions']
    num_steps = len(actions)
    
    # Build transition list
    transitions = []
    
    for t in range(num_steps):
        # Build observation dict
        obs = {}
        
        # Add primary camera as 'image'
        if primary_camera in obs_dict:
            obs['image'] = obs_dict[primary_camera][t]
        
        # Add all cameras individually (if you want multi-camera)
        for cam_name in ['cam_high', 'cam_low', 'cam_left_wrist', 'cam_right_wrist']:
            if cam_name in obs_dict:
                obs[cam_name] = obs_dict[cam_name][t]
        
        # Add robot state
        if include_state:
            state = np.concatenate([
                obs_dict['qpos'][t],
                obs_dict['qvel'][t]
            ])
            obs['state'] = state
        
        # Create transition
        transition = {
            'observations': obs,
            'actions': actions[t],
            'rewards': 0.0,  # Dummy reward (set properly if you have rewards)
            'dones': (t == num_steps - 1)  # Last step is done
        }
        
        transitions.append(transition)
    
    return transitions


def convert_episode_to_pickle(episode_dir: Path,
                              output_path: Path,
                              primary_camera: str = 'cam_high',
                              include_state: bool = True):
    """Convert single episode to SERL pickle"""
    print(f"Loading: {episode_dir}")
    episode_data = load_episode(episode_dir)
    
    print(f"Converting: {episode_data['meta']['num_steps']} steps")
    transitions = convert_to_serl_format(
        episode_data,
        primary_camera=primary_camera,
        include_state=include_state
    )
    
    # Save pickle
    output_path.parent.mkdir(parents=True, exist_ok=True)
    with open(output_path, 'wb') as f:
        pkl.dump(transitions, f)
    
    print(f"✓ Saved: {output_path}")
    print(f"  Transitions: {len(transitions)}")
    print(f"  Observation keys: {list(transitions[0]['observations'].keys())}")
    
    # Print shapes
    for key, val in transitions[0]['observations'].items():
        print(f"    {key}: {val.shape if hasattr(val, 'shape') else type(val)}")


def convert_directory(input_dir: Path,
                      output_dir: Path,
                      primary_camera: str = 'cam_high',
                      include_state: bool = True):
    """Convert all episodes in directory"""
    episode_dirs = sorted([d for d in input_dir.iterdir() if d.is_dir()])
    
    if not episode_dirs:
        print(f"No episode directories found in {input_dir}")
        return
    
    print(f"Found {len(episode_dirs)} episodes")
    print(f"Output dir: {output_dir}")
    print()
    
    for i, episode_dir in enumerate(episode_dirs):
        output_path = output_dir / f"{episode_dir.name}.pkl"
        
        try:
            convert_episode_to_pickle(
                episode_dir,
                output_path,
                primary_camera=primary_camera,
                include_state=include_state
            )
            print()
        except Exception as e:
            print(f"✗ Failed to convert {episode_dir.name}: {e}\n")
    
    print(f"✓ Converted {len(episode_dirs)} episodes")
    print(f"  Saved to: {output_dir}")


def main():
    parser = argparse.ArgumentParser(description='Convert integrated recorder episodes to SERL pickle')
    parser.add_argument('input_dir', type=str, help='Directory with recorded episodes')
    parser.add_argument('--output-dir', type=str, default='serl_demos',
                       help='Output directory for pickle files')
    parser.add_argument('--primary-camera', default='cam_high',
                       choices=['cam_high', 'cam_low', 'cam_left_wrist', 'cam_right_wrist'],
                       help='Which camera to use as primary "image"')
    parser.add_argument('--no-state', action='store_true',
                       help='Do not include qpos/qvel state in observations')
    
    args = parser.parse_args()
    
    input_dir = Path(args.input_dir)
    output_dir = Path(args.output_dir)
    
    if not input_dir.exists():
        print(f"✗ Input directory not found: {input_dir}")
        return 1
    
    convert_directory(
        input_dir,
        output_dir,
        primary_camera=args.primary_camera,
        include_state=not args.no_state
    )
    
    return 0


if __name__ == '__main__':
    exit(main())
