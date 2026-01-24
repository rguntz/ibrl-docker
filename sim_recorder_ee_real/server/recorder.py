"""
Recorder - Episode management only (list/delete).
Local recording is now handled by the teleop client.
"""

import numpy as np
import threading
import time
from pathlib import Path
import json
from typing import Optional, Dict, List
from datetime import datetime
import h5py
import logging

# Disable Flask's default request logging
log = logging.getLogger('werkzeug')
log.setLevel(logging.ERROR)


class Recorder:
    """Manages episode list/delete operations only. Data is recorded locally by teleop client."""
    
    def __init__(self, camera_manager, base_path='data/dataset_3.hdf5'):
        """
        Initialize recorder for episode management.
        
        Args:
            camera_manager: Not used (kept for backward compatibility)
            base_path: Path to HDF5 dataset file
        """
        self.base_path = Path(base_path)
        self.base_path.parent.mkdir(parents=True, exist_ok=True)
        
        # Recording state (used only to track if recording should be "active" for UI)
        self._recording = False
        self.current_episode_name = None
        self.fps = 15
        
    def is_recording(self) -> bool:
        """Return whether recording is active (for UI state)"""
        return self._recording
    
    def start_recording(self, episode_name: str, fps: float = 15) -> bool:
        """Mark recording as started (actual saving done by teleop client)"""
        if self._recording:
            print("⚠️  Already marked as recording!")
            return False
        
        self.current_episode_name = episode_name
        self.fps = fps
        self._recording = True
        print(f"🔴 RECORDING STATE SET: {episode_name} @ {fps} FPS (actual saving by teleop client)")
        return True
    
    def stop_recording(self) -> Optional[Path]:
        """Mark recording as stopped"""
        if not self._recording:
            print("⚠️  Not marked as recording!")
            return False
        
        self._recording = False
        episode_name = self.current_episode_name
        self.current_episode_name = None
        print(f"⏹️  RECORDING STATE STOPPED: {episode_name}")
        
        return True
    
    def list_episodes(self) -> List[Dict]:
        """List all recorded episodes from HDF5 dataset_3"""
        episodes = []
        dataset_path = Path(self.base_path)

        
        if not dataset_path.exists():
            return episodes
        
        try:
            with h5py.File(dataset_path, "r") as f: 
                if "data" not in f:
                    return episodes
                
                data_group = f["data"]
                for demo_name in sorted(data_group.keys()):
                    demo_group = data_group[demo_name]
                    
                    # Get number of steps from actions dataset
                    num_steps = len(demo_group.get('actions', [])) if 'actions' in demo_group else 0
                    
                    episodes.append({
                        'id': demo_name,
                        'name': demo_name,
                        'num_steps': num_steps,
                        'duration': num_steps / self.fps,
                        'path': str(dataset_path)
                    })
        except Exception as e:
            print(f"⚠️  Error reading episodes from HDF5: {e}")
        
        return episodes
    
    def delete_episode(self, episode_id: str) -> bool:
        """Delete an episode from HDF5 dataset_3"""
        dataset_path = Path(self.base_path)
        
        if not dataset_path.exists():
            print(f"⚠️  dataset file not found: {dataset_path}")
            return False
        
        try:
            with h5py.File(dataset_path, "r+") as f:
                if "data" not in f:
                    print(f"⚠️  No 'data' group in HDF5 file")
                    return False
                
                data_group = f["data"]
                
                if episode_id not in data_group:
                    print(f"⚠️  Episode '{episode_id}' not found in HDF5 file")
                    return False
                
                # Delete the demo group
                del data_group[episode_id]
                print(f"✗ Deleted episode: {episode_id} from HDF5")
                return True
                
        except Exception as e:
            print(f"❌ Error deleting episode '{episode_id}': {e}")
            return False
