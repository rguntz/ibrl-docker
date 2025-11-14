"""
Camera Manager - Buffers camera frames with timestamps
"""

import numpy as np
import threading
import time
from typing import Optional, Dict, List


class CameraManager:
    """Manages multiple camera buffers with latest frame storage"""
    
    def __init__(self, num_cameras: int = 4):
        self.num_cameras = num_cameras
        self.frames: Dict[int, Optional[np.ndarray]] = {i: None for i in range(num_cameras)}
        self.timestamps: Dict[int, float] = {i: 0.0 for i in range(num_cameras)}
        self.lock = threading.Lock()
        
        # Camera names (default)
        self.camera_names = {
            0: 'cam_high',
            1: 'cam_low',
            2: 'cam_left_wrist',
            3: 'cam_right_wrist'
        }
    
    def update_frame(self, cam_id: int, frame: np.ndarray):
        """Update latest frame for camera"""
        with self.lock:
            self.frames[cam_id] = frame.copy()
            self.timestamps[cam_id] = time.time()
    
    def get_frame(self, cam_id: int) -> Optional[np.ndarray]:
        """Get latest frame for camera"""
        with self.lock:
            return self.frames[cam_id].copy() if self.frames[cam_id] is not None else None
    
    def get_all_frames(self) -> Dict[str, np.ndarray]:
        """Get all latest frames with camera names"""
        with self.lock:
            return {
                self.camera_names[cam_id]: frame.copy()
                for cam_id, frame in self.frames.items()
                if frame is not None
            }
    
    def get_active_cameras(self) -> List[str]:
        """Get list of cameras that have received frames"""
        with self.lock:
            return [
                self.camera_names[cam_id]
                for cam_id, frame in self.frames.items()
                if frame is not None
            ]
    
    def get_timestamp(self, cam_id: int) -> float:
        """Get timestamp of latest frame"""
        with self.lock:
            return self.timestamps[cam_id]
