"""
Recorder - Core recording engine with FPS-controlled sampling
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
log.setLevel(logging.ERROR)  # only show errors


class Recorder:
    """Records episodes with FPS-controlled sampling"""
    
    def __init__(self, camera_manager, base_path='data'):
        self.camera_manager = camera_manager
        self.base_path = Path(base_path)
        self.base_path.mkdir(parents=True, exist_ok=True)
        
        # Recording state
        self._recording = False
        self._recording_thread = None
        self.current_episode_name = None
        self.current_episode_data = None
        self.fps = 15
        # Latest externally pushed state (qpos, qvel, action)
        # Access protected by _state_lock
        self.latest_state = None
        self._state_lock = threading.Lock()
        
    def is_recording(self) -> bool:
        return self._recording
    
    def get_num_steps(self) -> int:
        if self.current_episode_data:
            return len(self.current_episode_data['observations'])
        return 0
    
    def start_recording(self, episode_name: str, fps: float = 15) -> bool:
        """Start recording new episode"""
        if self._recording:
            print("Already recording!")
            return False
        
        self.current_episode_name = episode_name
        self.fps = fps
        self.current_episode_data = {
            'name': episode_name,
            'start_time': time.time(),
            'observations': [],
            'actions': [],
            # Per-step full robot state storage (qpos, qvel)
            'qpos': [],
            'qvel': []
        }
        
        self._recording = True
        self._recording_thread = threading.Thread(target=self._recording_loop)
        self._recording_thread.start()
        
        print(f"🔴 RECORDING STARTED: {episode_name} @ {fps} FPS")
        return True

    def set_latest_state(self, qpos: np.ndarray, qvel: np.ndarray, action: np.ndarray):
        """Set the most recent state pushed from the client/server.

        Stored as numpy arrays under a lock so the recording thread can read them.
        """
        with self._state_lock:
            # Store copies to avoid shared-mutation issues
            self.latest_state = {
                'qpos': np.array(qpos, copy=True),
                'qvel': np.array(qvel, copy=True),
                'action': np.array(action, copy=True)
            }

    def get_latest_state(self) -> Optional[Dict[str, np.ndarray]]:
        """Return a copy of the latest state if available, else None."""
        with self._state_lock:
            if self.latest_state is None:
                return None
            return {k: np.array(v, copy=True) for k, v in self.latest_state.items()}
    
    def stop_recording(self) -> Optional[Path]:
        """Stop recording and save episode"""
        if not self._recording:
            print("Not recording!")
            return None
        
        self._recording = False
        if self._recording_thread:
            self._recording_thread.join()
        
        # Save episode
        episode_path = self._save_episode_hdf5()
        
        print(f"✅ RECORDING STOPPED: {len(self.current_episode_data['observations'])} steps")
        
        self.current_episode_data = None
        self.current_episode_name = None
        
        return episode_path
    
    def _recording_loop(self):
        """Background thread that samples at FPS"""
        dt = 1.0 / self.fps
        
        while self._recording:
            loop_start = time.time()
            
            # Capture current state
            frames = self.camera_manager.get_all_frames()
            
            # Get the latest externally pushed state if available, otherwise fall
            # back to a sensible default (zeros). We attempt to preserve the
            # expected dimensionality: prefer the pushed action length, else use
            # a 14-D zero action (dual robots: 7+7), which matches the
            # teleop client behaviour.
            latest = self.get_latest_state()
            if latest is not None:
                # If the external state provides qpos/qvel/action, use them.
                action = latest.get('action', None)
                qpos = latest.get('qpos', None)
                qvel = latest.get('qvel', None)
            else:
                action = None
                qpos = None
                qvel = None

            # Fallback defaults when state pieces are missing
            if action is None:
                action = np.zeros(14, dtype=float)
            if qpos is None:
                qpos = np.zeros(16, dtype=float)
            if qvel is None:
                qvel = np.zeros(16, dtype=float)

            # Note: if desired, recorder could also store qpos/qvel from latest
            # state; currently actions are saved alongside observations.
            
            # Store observation + state
            self.current_episode_data['observations'].append(frames)
            self.current_episode_data['actions'].append(action)
            self.current_episode_data['qpos'].append(qpos)
            self.current_episode_data['qvel'].append(qvel)
            
            # Sleep to maintain FPS
            elapsed = time.time() - loop_start
            
            
            sleep_time = dt - elapsed
            if sleep_time > 0:
                time.sleep(sleep_time)
    

    def _save_episode_hdf5(self, dataset_path="data/dataset.hdf5") -> Path:
        """
        Save the current episode in robomimic-compatible HDF5 format.
        Returns the Path to the dataset file.
        """

        #################################
        # Visualizer for the file : https://myhdf5.hdfgroup.org/view?url=blob%3Ahttps%3A%2F%2Fmyhdf5.hdfgroup.org%2F90bd6fa6-5c27-405f-add5-488873315377
        #################################

        dataset_path = Path(dataset_path)
        dataset_path.parent.mkdir(parents=True, exist_ok=True)
        
        mode = "a"  # append mode so multiple episodes can be saved
        with h5py.File(dataset_path, mode) as f:
            if "data" not in f:
                data_group = f.create_group("data")
                env_args = {
                    "env_name": "TransferCubeTask",
                    "env_kwargs": {
                        "robots": ["panda"],
                        "controller_configs": {"control_delta": True},
                    }
                }
                f["data"].attrs["env_args"] = json.dumps(env_args)
            else:
                data_group = f["data"]
            
            demo_id = len(data_group)
            demo_name = f"demo_{demo_id}"
            demo_group = data_group.create_group(demo_name)
            
            # Save actions
            actions_array = np.array(self.current_episode_data["actions"], dtype=np.float32)
            demo_group.create_dataset("actions", data=actions_array)
            
            # Save dummy rewards
            rewards = np.zeros(len(actions_array), dtype=np.float32)
            demo_group.create_dataset("rewards", data=rewards)
            
            # Save observations
            obs_group = demo_group.create_group("obs")
            first_obs = self.current_episode_data["observations"][0]
            for cam_name in first_obs.keys():
                cam_images = [obs[cam_name] for obs in self.current_episode_data["observations"]]
                cam_array = np.stack(cam_images, axis=0).astype(np.uint8)
                obs_group.create_dataset(f"{cam_name}_image", data=cam_array, compression="gzip")
            
            # Save proprioception
            qpos_array = np.array(self.current_episode_data["qpos"], dtype=np.float32)
            obs_group.create_dataset("prop", data=qpos_array)
            
            print(f"💾 Saved {demo_name} to {dataset_path}")
        
        # ✅ Return the path to the dataset
        return dataset_path



    def list_episodes(self) -> List[Dict]:
        """List all recorded episodes"""
        episodes = []
        for episode_dir in sorted(self.base_path.iterdir()):
            if episode_dir.is_dir() and (episode_dir / 'meta.json').exists():
                with open(episode_dir / 'meta.json') as f:
                    meta = json.load(f)
                episodes.append({
                    'id': episode_dir.name,
                    'name': meta.get('episode_name', episode_dir.name),
                    'num_steps': meta.get('num_steps', 0),
                    'duration': meta.get('duration', 0),
                    'path': str(episode_dir)
                })
        return episodes
    
    def delete_episode(self, episode_id: str) -> bool:
        """Delete an episode"""
        episode_dir = self.base_path / episode_id
        if episode_dir.exists():
            import shutil
            shutil.rmtree(episode_dir)
            print(f"✗ Deleted episode: {episode_id}")
            return True
        return False
