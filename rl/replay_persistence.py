"""
Replay buffer persistence module for crash recovery.
Handles saving and loading replay buffer episodes to/from disk to ensure
no data is lost in case of training crashes.
"""

import os
import h5py
import torch
import numpy as np
from pathlib import Path
from typing import Optional
import pickle
import json


class ReplayPersistence:
    """
    Manages persistent storage of replay buffer episodes on disk.
    
    Directory structure:
    - replay_dir/
      - rl_episodes/: All RL training episodes
      - bc_episodes/: BC successful episodes (if using bc_replay)
      - warmup_episodes/: Warmup phase episodes
      - metadata.json: Replay buffer metadata for recovery
    """
    
    def __init__(self, replay_dir: str):
        """
        Initialize the persistence manager.
        
        Args:
            replay_dir: Root directory for storing all replay data
        """
        self.replay_dir = Path(replay_dir)
        self.rl_episodes_dir = self.replay_dir / "rl_episodes"
        self.bc_episodes_dir = self.replay_dir / "bc_episodes"
        self.warmup_episodes_dir = self.replay_dir / "warmup_episodes"
        self.metadata_file = self.replay_dir / "replay_metadata.json"
        
        # Create directories if they don't exist
        self.rl_episodes_dir.mkdir(parents=True, exist_ok=True)
        self.bc_episodes_dir.mkdir(parents=True, exist_ok=True)
        self.warmup_episodes_dir.mkdir(parents=True, exist_ok=True)
        
        # Load existing metadata or initialize new
        self.metadata = self._load_metadata()
    
    def _load_metadata(self) -> dict:
        """Load metadata from disk or create new one."""
        if self.metadata_file.exists():
            with open(self.metadata_file, 'r') as f:
                return json.load(f)
        return {
            'rl_episodes': 0,
            'bc_episodes': 0,
            'warmup_episodes': 0,
            'num_rl_success': 0,
            'num_bc_success': 0,
        }
    
    def _save_metadata(self):
        """Save metadata to disk."""
        with open(self.metadata_file, 'w') as f:
            json.dump(self.metadata, f, indent=2)
    
    def save_episode(
        self,
        episode_data: dict,
        episode_type: str = "rl",
        success: bool = False,
        episode_id: Optional[int] = None,
    ) -> str:
        """
        Save a single episode to disk.
        
        Args:
            episode_data: Dictionary containing episode information
                - obs: dict of observations
                - actions: tensor of actions
                - rewards: tensor of rewards
                - terminals: tensor of terminal flags
            episode_type: Type of episode ("rl", "bc", "warmup")
            success: Whether the episode was successful
            episode_id: Optional episode ID, if None auto-increments
        
        Returns:
            Filepath where episode was saved
        """
        # Select appropriate directory
        if episode_type == "warmup":
            episodes_dir = self.warmup_episodes_dir
            counter_key = 'warmup_episodes'
        elif episode_type == "bc":
            episodes_dir = self.bc_episodes_dir
            counter_key = 'bc_episodes'
            if success:
                self.metadata['num_bc_success'] = self.metadata.get('num_bc_success', 0) + 1
        else:  # "rl"
            episodes_dir = self.rl_episodes_dir
            counter_key = 'rl_episodes'
            if success:
                self.metadata['num_rl_success'] = self.metadata.get('num_rl_success', 0) + 1
        
        # Get episode ID
        if episode_id is None:
            episode_id = self.metadata[counter_key]
        
        # Create filename
        success_tag = "_success" if success else ""
        filename = episodes_dir / f"episode_{episode_id:06d}{success_tag}.h5"
        
        # Save episode as HDF5
        self._write_episode_h5(filename, episode_data)
        
        # Update metadata
        self.metadata[counter_key] += 1
        self._save_metadata()
        
        return str(filename)
    
    def _write_episode_h5(self, filename: Path, episode_data: dict):
        """Write episode data to HDF5 file."""
        with h5py.File(filename, 'w') as hf:
            # Store metadata
            hf.attrs['episode_type'] = episode_data.get('episode_type', 'rl')
            hf.attrs['success'] = episode_data.get('success', False)
            
            # Store observations
            obs_grp = hf.create_group('obs')
            for key, value in episode_data.get('obs', {}).items():
                if isinstance(value, torch.Tensor):
                    value = value.numpy()
                obs_grp.create_dataset(key, data=value)
            
            # Store actions
            actions = episode_data.get('actions', None)
            if actions is not None:
                if isinstance(actions, torch.Tensor):
                    actions = actions.numpy()
                hf.create_dataset('actions', data=actions)
            
            # Store rewards
            rewards = episode_data.get('rewards', None)
            if rewards is not None:
                if isinstance(rewards, torch.Tensor):
                    rewards = rewards.numpy()
                hf.create_dataset('rewards', data=rewards)
            
            # Store terminals
            terminals = episode_data.get('terminals', None)
            if terminals is not None:
                if isinstance(terminals, torch.Tensor):
                    terminals = terminals.numpy()
                hf.create_dataset('terminals', data=terminals)
    
    def get_recovery_episodes(self, episode_type: str = "rl") -> list[str]:
        """
        Get list of saved episodes for recovery.
        
        Args:
            episode_type: Type of episodes to retrieve ("rl", "bc", "warmup")
        
        Returns:
            List of filepaths to episode files
        """
        if episode_type == "warmup":
            episodes_dir = self.warmup_episodes_dir
        elif episode_type == "bc":
            episodes_dir = self.bc_episodes_dir
        else:
            episodes_dir = self.rl_episodes_dir
        
        episodes = sorted(episodes_dir.glob("episode_*.h5"))
        return [str(ep) for ep in episodes]
    
    def get_episode_count(self, episode_type: str = "rl") -> int:
        """Get number of saved episodes of a type."""
        if episode_type == "warmup":
            return self.metadata.get('warmup_episodes', 0)
        elif episode_type == "bc":
            return self.metadata.get('bc_episodes', 0)
        else:
            return self.metadata.get('rl_episodes', 0)
    
    def get_metadata(self) -> dict:
        """Get current replay persistence metadata."""
        return self.metadata.copy()


def save_replay_episode_from_trajectory(
    persistence: ReplayPersistence,
    obs_dict: dict,
    actions: torch.Tensor,
    rewards: torch.Tensor,
    terminals: torch.Tensor,
    episode_type: str = "rl",
    success: bool = False,
) -> str:
    """
    Convenience function to save a complete episode trajectory.
    
    Args:
        persistence: ReplayPersistence instance
        obs_dict: Dictionary of observations (each should be a list or tensor)
        actions: Tensor or list of actions
        rewards: Tensor or list of rewards
        terminals: Tensor or list of terminal flags
        episode_type: Type of episode ("rl", "bc", "warmup")
        success: Whether episode was successful
    
    Returns:
        Filepath where episode was saved
    """
    episode_data = {
        'obs': obs_dict,
        'actions': actions,
        'rewards': rewards,
        'terminals': terminals,
        'episode_type': episode_type,
        'success': success,
    }
    return persistence.save_episode(episode_data, episode_type, success)


def load_episode_from_file(filepath: str) -> dict:
    """
    Load a single episode from HDF5 file.
    
    Args:
        filepath: Path to episode HDF5 file
    
    Returns:
        Dictionary containing episode data
    """
    episode_data = {}
    with h5py.File(filepath, 'r') as hf:
        # Load metadata
        episode_data['episode_type'] = hf.attrs.get('episode_type', 'rl')
        episode_data['success'] = bool(hf.attrs.get('success', False))
        
        # Load observations
        obs = {}
        if 'obs' in hf:
            for key in hf['obs'].keys():
                obs[key] = torch.from_numpy(np.array(hf['obs'][key]))
        episode_data['obs'] = obs
        
        # Load actions
        if 'actions' in hf:
            episode_data['actions'] = torch.from_numpy(np.array(hf['actions']))
        
        # Load rewards
        if 'rewards' in hf:
            episode_data['rewards'] = torch.from_numpy(np.array(hf['rewards']))
        
        # Load terminals
        if 'terminals' in hf:
            episode_data['terminals'] = torch.from_numpy(np.array(hf['terminals']))
    
    return episode_data
