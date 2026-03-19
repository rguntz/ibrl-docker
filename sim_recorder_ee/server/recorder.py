"""
Recorder - Core recording engine with FPS-controlled sampling
"""

import numpy as np
import threading
import time
from pathlib import Path
import json
from typing import Optional, Dict, List
import h5py
import os
import cv2

import logging

# Disable Flask's default request logging
log = logging.getLogger('werkzeug')
log.setLevel(logging.ERROR)  # only show errors


# ---------------------------------------------------------------------------
# Default paths – override by passing arguments to Recorder.__init__
# ---------------------------------------------------------------------------
DEFAULT_DATASET_PATH = "data_multi_task/dataset.hdf5"
DEFAULT_SAVE_DIR = "/home/qtf5422/Desktop/AIRE/ibrl-docker/sim_recorder/server/data"


class Recorder:
    """Records episodes with FPS-controlled sampling.

    Parameters
    ----------
    camera_manager:
        Object that exposes ``get_all_frames()``.
    dataset_path:
        Path to the HDF5 file used for episode storage.
    save_dir:
        Directory used by :meth:`save_frame` to persist individual camera frames.
    """

    def __init__(
        self,
        camera_manager,
        dataset_path: str = DEFAULT_DATASET_PATH,
        save_dir: str = DEFAULT_SAVE_DIR,
    ):
        self.camera_manager = camera_manager
        self.dataset_path = Path(dataset_path)
        self.save_dir = Path(save_dir)

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

    # ------------------------------------------------------------------
    # Public helpers
    # ------------------------------------------------------------------

    def is_recording(self) -> bool:
        return self._recording

    def get_num_steps(self) -> int:
        if self.current_episode_data:
            return len(self.current_episode_data['observations'])
        return 0

    # ------------------------------------------------------------------
    # Recording lifecycle
    # ------------------------------------------------------------------

    def start_recording(self, episode_name: str, fps: float = 15) -> bool:
        """Start recording a new episode."""
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
            'qpos': [],
            'qvel': [],
            'reward': [],
            'robot0_eef_pos': [],
            'robot0_eef_quat': [],
            'robot0_gripper_qpos': [],
        }

        self._recording = True
        self._recording_thread = threading.Thread(target=self._recording_loop)
        self._recording_thread.start()

        print(f"🔴 RECORDING STARTED: {episode_name} @ {fps} FPS")
        return True

    def stop_recording(self) -> Optional[Path]:
        """Stop recording and persist the episode to HDF5."""
        if not self._recording:
            print("Not recording!")
            return None

        self._recording = False
        if self._recording_thread:
            self._recording_thread.join()

        episode_path = self._save_episode_hdf5()

        print(f"✅ RECORDING STOPPED: {len(self.current_episode_data['observations'])} steps")

        self.current_episode_data = None
        self.current_episode_name = None

        return episode_path

    # ------------------------------------------------------------------
    # State synchronisation
    # ------------------------------------------------------------------

    def set_latest_state(
        self,
        qpos: np.ndarray,
        qvel: np.ndarray,
        action: np.ndarray,
        robot0_eef_pos: np.ndarray,
        robot0_eef_quat: np.ndarray,
        robot0_gripper_qpos: np.ndarray,
        reward: np.float64,
    ) -> None:
        """Push the most recent robot state from the client/server."""
        with self._state_lock:
            self.latest_state = {
                'qpos': np.array(qpos, copy=True),
                'qvel': np.array(qvel, copy=True),
                'action': np.array(action, copy=True),
                'robot0_eef_pos': np.array(robot0_eef_pos, copy=True),
                'robot0_eef_quat': np.array(robot0_eef_quat, copy=True),
                'robot0_gripper_qpos': np.array(robot0_gripper_qpos, copy=True),
                'reward': np.float64(reward),
            }

    def get_latest_state(self) -> Optional[Dict[str, np.ndarray]]:
        """Return a deep copy of the latest state, or ``None``."""
        with self._state_lock:
            if self.latest_state is None:
                return None
            return {k: np.array(v, copy=True) for k, v in self.latest_state.items()}

    # ------------------------------------------------------------------
    # Frame saving
    # ------------------------------------------------------------------

    def save_frame(self, frame: np.ndarray, prefix: str = "cam_high") -> None:
        """Save a single image frame to :attr:`save_dir`."""
        self.save_dir.mkdir(parents=True, exist_ok=True)

        existing = [
            f for f in os.listdir(self.save_dir)
            if f.startswith(prefix) and f.endswith(".png")
        ]
        next_id = len(existing)

        filepath = self.save_dir / f"{prefix}_{next_id:05d}.png"
        # OpenCV expects BGR; input is RGB
        cv2.imwrite(str(filepath), frame[:, :, ::-1])

    # ------------------------------------------------------------------
    # Internal recording loop
    # ------------------------------------------------------------------

    def _recording_loop(self) -> None:
        """Background thread that samples at :attr:`fps`."""
        dt = 1.0 / self.fps

        while self._recording:
            loop_start = time.time()

            frames = self.camera_manager.get_all_frames()

            latest = self.get_latest_state()
            if latest is not None:
                action = latest.get('action')
                qpos = latest.get('qpos')
                qvel = latest.get('qvel')
                robot0_eef_pos = latest.get('robot0_eef_pos')
                robot0_eef_quat = latest.get('robot0_eef_quat')
                robot0_gripper_qpos = latest.get('robot0_gripper_qpos')
                reward = latest.get('reward')
            else:
                action = qpos = qvel = reward = None
                robot0_eef_pos = robot0_eef_quat = robot0_gripper_qpos = None

            # Fallback defaults when state pieces are missing
            if action is None:
                action = np.zeros(14, dtype=float)
            if qpos is None:
                qpos = np.zeros(16, dtype=float)
            if qvel is None:
                qvel = np.zeros(16, dtype=float)
            if robot0_eef_pos is None:
                robot0_eef_pos = np.zeros(6, dtype=float)
            if robot0_eef_quat is None:
                robot0_eef_quat = np.zeros(8, dtype=float)
            if robot0_gripper_qpos is None:
                robot0_gripper_qpos = np.zeros(4, dtype=float)
            if reward is None:
                reward = np.float64(0.0)

            self.current_episode_data['observations'].append(frames)
            self.current_episode_data['actions'].append(action)
            self.current_episode_data['qpos'].append(qpos)
            self.current_episode_data['qvel'].append(qvel)
            self.current_episode_data['robot0_eef_pos'].append(robot0_eef_pos)
            self.current_episode_data['robot0_eef_quat'].append(robot0_eef_quat)
            self.current_episode_data['robot0_gripper_qpos'].append(robot0_gripper_qpos)
            self.current_episode_data['reward'].append(reward)

            elapsed = time.time() - loop_start
            sleep_time = dt - elapsed
            if sleep_time > 0:
                time.sleep(sleep_time)

    # ------------------------------------------------------------------
    # HDF5 persistence
    # ------------------------------------------------------------------

    def _save_episode_hdf5(self) -> Path:
        """Save the current episode in robomimic-compatible HDF5 format.

        The destination file is :attr:`dataset_path`, which is set at
        construction time – no path is hard-coded here.
        """
        self.dataset_path.parent.mkdir(parents=True, exist_ok=True)

        with h5py.File(self.dataset_path, "a") as f:
            if "data" not in f:
                data_group = f.create_group("data")
                env_args = {
                    "env_name": "TransferCubeEETask",
                    "env_kwargs": {
                        "robots": ["panda"],
                        "controller_configs": {"control_delta": True},
                    },
                }
                f["data"].attrs["env_args"] = json.dumps(env_args)
            else:
                data_group = f["data"]

            demo_id = len(data_group)
            demo_name = f"demo_{demo_id}"
            demo_group = data_group.create_group(demo_name)

            actions_array = np.array(self.current_episode_data["actions"], dtype=np.float32)
            demo_group.create_dataset("actions", data=actions_array)

            rewards_array = np.array(self.current_episode_data["reward"], dtype=np.float32)
            demo_group.create_dataset("rewards", data=rewards_array)

            obs_group = demo_group.create_group("obs")
            first_obs = self.current_episode_data["observations"][0]
            for cam_name in first_obs.keys():
                cam_images = [obs[cam_name] for obs in self.current_episode_data["observations"]]
                cam_array = np.stack(cam_images, axis=0).astype(np.uint8)
                # (T, H, W, C) → (T, C, H, W)
                cam_array = np.transpose(cam_array, (0, 3, 1, 2))
                obs_group.create_dataset(f"{cam_name}_image", data=cam_array, compression="gzip")

            obs_group.create_dataset("qpos",
                data=np.array(self.current_episode_data["qpos"], dtype=np.float32))
            obs_group.create_dataset("qvel",
                data=np.array(self.current_episode_data["qvel"], dtype=np.float32))
            obs_group.create_dataset("robot0_eef_pos",
                data=np.array(self.current_episode_data["robot0_eef_pos"], dtype=np.float32))
            obs_group.create_dataset("robot0_eef_quat",
                data=np.array(self.current_episode_data["robot0_eef_quat"], dtype=np.float32))
            obs_group.create_dataset("robot0_gripper_qpos",
                data=np.array(self.current_episode_data["robot0_gripper_qpos"], dtype=np.float32))

            print(f"💾 Saved {demo_name} to {self.dataset_path}")

        return self.dataset_path

    # ------------------------------------------------------------------
    # Episode management
    # ------------------------------------------------------------------

    def list_episodes(self) -> List[Dict]:
        """List all recorded episodes from :attr:`dataset_path`."""
        episodes = []

        if not self.dataset_path.exists():
            return episodes

        try:
            with h5py.File(self.dataset_path, "r") as f:
                if "data" not in f:
                    return episodes

                for demo_name in sorted(f["data"].keys()):
                    demo_group = f["data"][demo_name]
                    num_steps = (
                        len(demo_group["actions"]) if "actions" in demo_group else 0
                    )
                    episodes.append({
                        'id': demo_name,
                        'name': demo_name,
                        'num_steps': num_steps,
                        'duration': num_steps / self.fps,
                        'path': str(self.dataset_path),
                    })
        except Exception as e:
            print(f"⚠️  Error reading episodes from HDF5: {e}")

        return episodes

    def delete_episode(self, episode_id: str) -> bool:
        """Delete *episode_id* from :attr:`dataset_path`."""
        if not self.dataset_path.exists():
            print(f"⚠️  dataset file not found: {self.dataset_path}")
            return False

        try:
            with h5py.File(self.dataset_path, "r+") as f:
                if "data" not in f:
                    print("⚠️  No 'data' group in HDF5 file")
                    return False

                data_group = f["data"]
                if episode_id not in data_group:
                    print(f"⚠️  Episode '{episode_id}' not found in HDF5 file")
                    return False

                del data_group[episode_id]
                print(f"✗ Deleted episode: {episode_id} from HDF5")
                return True

        except Exception as e:
            print(f"❌ Error deleting episode '{episode_id}': {e}")
            return False