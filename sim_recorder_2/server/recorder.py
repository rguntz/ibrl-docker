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


class Recorder:
    """Records episodes with FPS-controlled sampling"""
    
    def __init__(self, camera_manager, data_receiver=None, base_path='data'):
        self.camera_manager = camera_manager
        self.data_receiver = data_receiver
        self.base_path = Path(base_path)
        self.base_path.mkdir(parents=True, exist_ok=True)
        
        # Recording state
        self._recording = False
        self._recording_thread = None
        self.current_episode_name = None
        self.current_episode_data = None
        self.fps = 15
        
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
            'actions': []
        }
        
        self._recording = True
        self._recording_thread = threading.Thread(target=self._recording_loop)
        self._recording_thread.start()
        
        print(f"🔴 RECORDING STARTED: {episode_name} @ {fps} FPS")
        return True
    
    def stop_recording(self) -> Optional[Path]:
        """Stop recording and save episode"""
        if not self._recording:
            print("Not recording!")
            return None
        
        self._recording = False
        if self._recording_thread:
            self._recording_thread.join()
        
        # Save episode
        episode_path = self._save_episode()
        
        print(f"✅ RECORDING STOPPED: {len(self.current_episode_data['observations'])} steps")
        
        self.current_episode_data = None
        self.current_episode_name = None
        
        return episode_path
    
    def _recording_loop(self):
        """Background thread that samples at FPS"""
        dt = 1.0 / self.fps
        
        while self._recording:
            loop_start = time.time()
            
            # Get latest data from receiver
            if self.data_receiver:
                data = self.data_receiver.get_latest_data()
                if data:
                    # Use the complete data packet
                    observation = data['cameras'].copy()
                    observation.update(data['robot_state'])  # Add qpos, qvel
                    
                    self.current_episode_data['observations'].append(observation)
                    self.current_episode_data['actions'].append(data['action'])
                else:
                    # No data available, skip this frame
                    time.sleep(dt)
                    continue
            else:
                # Fallback to old method (camera only)
                frames = self.camera_manager.get_all_frames()
                action = np.zeros(7)  # Placeholder
                self.current_episode_data['observations'].append(frames)
                self.current_episode_data['actions'].append(action)
            
            # Sleep to maintain FPS
            elapsed = time.time() - loop_start
            sleep_time = dt - elapsed
            if sleep_time > 0:
                time.sleep(sleep_time)
    
    def _save_episode(self) -> Path:
        """Save episode to disk"""
        episode_dir = self.base_path / self.current_episode_data['name']
        episode_dir.mkdir(parents=True, exist_ok=True)
        
        # Convert observations list to numpy arrays
        num_steps = len(self.current_episode_data['observations'])
        
        if num_steps > 0:
            # Separate cameras from robot state
            first_obs = self.current_episode_data['observations'][0]
            
            # Build observation dict
            obs_dict = {}
            
            # Handle camera images
            camera_keys = [k for k in first_obs.keys() if k.startswith('cam_')]
            for cam_name in camera_keys:
                cam_images = [obs[cam_name] for obs in self.current_episode_data['observations']]
                obs_dict[cam_name] = np.array(cam_images)
            
            # Handle robot state (qpos, qvel)
            if 'qpos' in first_obs:
                qpos_list = [obs['qpos'] for obs in self.current_episode_data['observations']]
                obs_dict['qpos'] = np.array(qpos_list)
            
            if 'qvel' in first_obs:
                qvel_list = [obs['qvel'] for obs in self.current_episode_data['observations']]
                obs_dict['qvel'] = np.array(qvel_list)
            
            # Save observations
            np.savez_compressed(episode_dir / 'observations.npz', **obs_dict)
            
            # Save actions
            actions_array = np.array(self.current_episode_data['actions'])
            np.save(episode_dir / 'actions.npy', actions_array)
            
            # Save metadata
            metadata = {
                'episode_name': self.current_episode_data['name'],
                'num_steps': num_steps,
                'start_time': self.current_episode_data['start_time'],
                'end_time': time.time(),
                'duration': time.time() - self.current_episode_data['start_time'],
                'fps': self.fps,
                'cameras': camera_keys,
                'has_robot_state': 'qpos' in first_obs
            }
            
            with open(episode_dir / 'meta.json', 'w') as f:
                json.dump(metadata, f, indent=2)
        
        return episode_dir
    
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
