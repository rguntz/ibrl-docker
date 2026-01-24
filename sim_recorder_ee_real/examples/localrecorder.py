import time
import numpy as np
from pathlib import Path
import json
import cv2
from scipy.spatial.transform import Rotation as R
from collections import OrderedDict, deque
from typing import Optional
import threading
import pyrealsense2 as rs
import h5py
from datetime import datetime


class LocalRecorder:
    """
    Local episode recorder - saves data directly to HDF5 with FPS-controlled sampling.
    Runs in background thread to avoid blocking the teleop loop.
    Records only *new* states pushed via `push_state`, up to a max FPS.
    All images are downsampled to 128x128 to reduce memory.
    """
    def __init__(self, base_path='data/dataset.hdf5', fps=35, max_queue_size=None, img_size=128):
        server_root = Path(__file__).resolve().parents[1]  # sim_recorder/server
        self.base_path = server_root / "server" / base_path
        
        # Recording state
        self._recording = False
        self._recording_thread = None
        self.fps = 35 # hardcode it to be sure that its correct. 
        self.max_queue_size = max_queue_size
        self.img_size = img_size
        self.current_episode_name = None
        self.current_episode_data = None
        
        # Thread-safe state queue
        self.state_queue = deque()
        self._queue_lock = threading.Lock()
        
    def is_recording(self) -> bool:
        return self._recording
    
    def start_recording(self, episode_name: str = None) -> bool:
        """Start recording a new episode"""
        if self._recording:
            print("⚠️  Already recording!")
            return False
        
        if episode_name is None:
            episode_name = f"episode_{datetime.now().strftime('%Y%m%d_%H%M%S')}"
        
        self.current_episode_name = episode_name
        self.current_episode_data = {
            'name': episode_name,
            'observations': [],
            'actions': [],
            'qpos': [],
            'qvel': [],
            'robot0_eef_pos': [],
            'robot0_eef_quat': [],
            'robot0_gripper_qpos': [],
            'robot0_eef_aa' : [], 
            'reward': []
        }
        
        self._recording = True
        self._recording_thread = threading.Thread(target=self._recording_loop, daemon=False)
        self._recording_thread.start()
        
        print(f"🔴 LOCAL RECORDING STARTED: {episode_name} @ {self.fps} FPS (images → {self.img_size}×{self.img_size})")
        return True
    
    def stop_recording(self) -> Path:
        """Stop recording and save episode to HDF5"""
        if not self._recording:
            print("⚠️  Not recording!")
            return None
        
        self._recording = False
        if self._recording_thread:
            self._recording_thread.join()
        
        # Save episode to HDF5
        save_path = self._save_episode_hdf5()
        
        num_steps = len(self.current_episode_data['observations'])
        print(f"✅ LOCAL RECORDING STOPPED: {num_steps} steps saved to {save_path}")
        
        self.current_episode_data = None
        self.current_episode_name = None
        
        return save_path

    def push_state(self, frames: dict, qpos: np.ndarray, qvel: np.ndarray, 
                action: np.ndarray, robot0_eef_pos: np.ndarray, 
                robot0_eef_quat: np.ndarray, robot0_gripper_qpos: np.ndarray, 
                robot0_eef_aa : np.ndarray,
                reward: float):
        state = {
            'frames': {k: v.copy() for k, v in frames.items()},
            'qpos': np.array(qpos, copy=True),
            'qvel': np.array(qvel, copy=True),
            'action': np.array(action, copy=True),
            'robot0_eef_pos': np.array(robot0_eef_pos, copy=True),
            'robot0_eef_quat': np.array(robot0_eef_quat, copy=True),
            'robot0_gripper_qpos': np.array(robot0_gripper_qpos, copy=True),
            'robot0_eef_aa' : np.array(robot0_eef_aa, copy=True), 
            'reward': float(reward)
        }
        with self._queue_lock:
            # Only enforce max size if it's set
            if self.max_queue_size is not None and len(self.state_queue) >= self.max_queue_size:
                self.state_queue.popleft()  # drop oldest
            self.state_queue.append(state)
    
    def _recording_loop(self):
        """Background thread that samples new states at most `fps` times per second"""
        dt = 1.0 / self.fps
        
        while self._recording:
            loop_start = time.time()
            
            state = None
            with self._queue_lock:
                if self.state_queue:
                    state = self.state_queue.popleft()
            
            if state is not None:
                self.current_episode_data['observations'].append(state['frames'])
                self.current_episode_data['actions'].append(state['action'])
                self.current_episode_data['qpos'].append(state['qpos'])
                self.current_episode_data['qvel'].append(state['qvel'])
                self.current_episode_data['robot0_eef_pos'].append(state['robot0_eef_pos'])
                self.current_episode_data['robot0_eef_quat'].append(state['robot0_eef_quat'])
                self.current_episode_data['robot0_gripper_qpos'].append(state['robot0_gripper_qpos'])
                self.current_episode_data['robot0_eef_aa'].append(state['robot0_eef_aa'])
                self.current_episode_data['reward'].append(state['reward'])
            
            elapsed = time.time() - loop_start
            sleep_time = dt - elapsed
            if sleep_time > 0:
                time.sleep(sleep_time)
    
    def _save_episode_hdf5(self) -> Path:
        """Save current episode to HDF5 file in robomimic format"""
        dataset_path = self.base_path
        dataset_path.parent.mkdir(parents=True, exist_ok=True)
        
        mode = "a"
        with h5py.File(dataset_path, mode) as f:
            if "data" not in f:
                data_group = f.create_group("data")
                env_args = {
                    "env_name": "TransferCubeEETask",
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
            
            # Save rewards
            rewards_array = np.array(self.current_episode_data["reward"], dtype=np.float32)
            demo_group.create_dataset("rewards", data=rewards_array)
            
            # Save observations
            obs_group = demo_group.create_group("obs")
            first_obs = self.current_episode_data["observations"][0]
            for cam_name in first_obs.keys():
                cam_images = [obs[cam_name] for obs in self.current_episode_data["observations"]]
                cam_array = np.stack(cam_images, axis=0).astype(np.uint8)
                # Shape: (T, H, W, C) → (T, C, H, W)
                cam_array = np.transpose(cam_array, (0, 3, 1, 2))
                obs_group.create_dataset(f"{cam_name}_image", data=cam_array, compression="gzip")
            
            # Save proprioception
            qpos_array = np.array(self.current_episode_data["qpos"], dtype=np.float32)
            qvel_array = np.array(self.current_episode_data["qvel"], dtype=np.float32)
            robot0_eef_pos_array = np.array(self.current_episode_data["robot0_eef_pos"], dtype=np.float32)
            robot0_eef_quat_array = np.array(self.current_episode_data["robot0_eef_quat"], dtype=np.float32)
            robot0_gripper_qpos_array = np.array(self.current_episode_data["robot0_gripper_qpos"], dtype=np.float32)
            robot0_eef_aa_array = np.array(self.current_episode_data["robot0_eef_aa"], dtype=np.float32)
            
            obs_group.create_dataset("qpos", data=qpos_array)
            obs_group.create_dataset("qvel", data=qvel_array)
            obs_group.create_dataset("robot0_eef_pos", data=robot0_eef_pos_array)
            obs_group.create_dataset("robot0_eef_quat", data=robot0_eef_quat_array)
            obs_group.create_dataset("robot0_gripper_qpos", data=robot0_gripper_qpos_array)
            obs_group.create_dataset("robot0_eef_aa", data=robot0_eef_aa_array)
            
            print(f"💾 Saved {demo_name} to {dataset_path}")
        
        return dataset_path