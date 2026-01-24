"""
Gym-compatible wrapper for Trossen AI TransferCubeEETask environment.

Converts dm_control TimeStep observations to gym-compatible dict format
and provides consistent interface with RobomimicImageWrapper.
"""

from typing import Optional, Dict, Tuple, Any
import numpy as np
import gym
from gym import spaces
import cv2

from matplotlib import pyplot as plt
import h5py


from trossen_arm_mujoco.ee_sim_env import plot_observation_images, set_observation_images

# Camera configurations for different tasks
GOOD_CAMERAS = {
    "TransferCubeEETask": ["cam_high", "cam_low", "cam_left_wrist", "cam_right_wrist"],
}
REAL_SETUP = False

class TrossenImageWrapper(gym.Env):
    """
    Adapter wrapper that makes TransferCubeEETask (dm_control) compatible
    with the gym interface expected by diffusion_policy.
    
    Converts:
    - dm_control TimeStep → gym-style dict observations
    - dm_control reset/step → gym-style reset/step returns
    - Image and robot state observations to standardized format
    
    Args:
        env: TransferCubeEETask environment from make_sim_env()
        shape_meta: Dict with 'action' and 'obs' specifications
        init_state: Optional initial state for deterministic resets
        render_obs_key: Which observation key contains the image to render
        dataset_path: Path to HDF5 dataset for loading actions
        demo_idx: Demo index to use from dataset (default: 0)
    """
    
    def __init__(
        self,
        env,
        shape_meta: dict,
        init_state: Optional[np.ndarray] = None,
        render_obs_key: str = 'cam_high',
        dataset_path: Optional[str] = "/home/qtf5422/Desktop/AIRE/ibrl-docker/diffusion_policy/diffusion_policy/data/trossen/200_demos_no_filtering/dataset_tresholded.hdf5",
        demo_idx: int = 0,
    ):
        self.env = env
        self.shape_meta = shape_meta
        self.init_state = init_state
        self.render_obs_key = render_obs_key
        self.render_cache = None
        self._seed = None
        self.seed_state_map = dict()
        self.has_reset_before = False
        
        # Dataset action loading
        self.dataset_path = dataset_path
        self.demo_idx = demo_idx
        self.dataset_actions = None
        self.dataset_obs = None
        self.action_step_counter = 0
        
        # # # Load dataset actions if path is provided
        # if self.dataset_path is not None:
        #     self._load_dataset_actions()
        
        # setup spaces
        action_shape = shape_meta['action']['shape']
        action_space = spaces.Box(
            low=-1,
            high=1,
            shape=action_shape,
            dtype=np.float32
        )
        self.action_space = action_space

        observation_space = spaces.Dict()
        for key, value in shape_meta['obs'].items():
            shape = value['shape']
            min_value, max_value = -1, 1
            if key.endswith('image'):
                min_value, max_value = 0, 1
            elif key.endswith('quat'):
                min_value, max_value = -1, 1
            elif key.endswith('qpos'):
                min_value, max_value = -1, 1
            elif key.endswith('pos'):
                # better range?
                min_value, max_value = -1, 1
            else:
                raise RuntimeError(f"Unsupported type {key}")
            
            this_space = spaces.Box(
                low=min_value,
                high=max_value,
                shape=shape,
                dtype=np.float32
            )
            observation_space[key] = this_space
        self.observation_space = observation_space

    def _load_dataset_actions(self):
        """Load actions and observations from the HDF5 dataset for the specified demo."""
        try:
            with h5py.File(self.dataset_path, "r") as f:
                demo_key = f"demo_{self.demo_idx}"
                if demo_key not in f["data"]:
                    raise KeyError(f"Demo '{demo_key}' not found in dataset")
                
                self.dataset_actions = f["data"][demo_key]["actions"][:]
                
                # Load all observation data from dataset
                self.dataset_obs = {}
                obs_group = f["data"][demo_key]["obs"]
                for key in obs_group.keys():
                    self.dataset_obs[key] = obs_group[key][:]
                
                print(f"Loaded {len(self.dataset_actions)} actions and observations from {demo_key}")

        except Exception as e:
            print(f"Warning: Could not load dataset actions/obs: {e}")
            self.dataset_actions = None
            self.dataset_obs = None

    def _get_dataset_action(self) -> Optional[np.ndarray]:
        """
        Get the current action from the dataset based on step counter.
        
        Returns:
            action: (16,) array or None if dataset not loaded or counter out of bounds
        """
        if self.dataset_actions is None:
            return None
        
        if self.action_step_counter >= len(self.dataset_actions):
            print(f"Warning: Step counter {self.action_step_counter} exceeds dataset length {len(self.dataset_actions)}")
            return None
        
        action = self.dataset_actions[self.action_step_counter].astype(np.float32)
        return action
      
    def _extract_obs(self, ts) -> Dict[str, np.ndarray]:
        obs_dict = {}
        raw_obs = ts.observation

        # 1. Determine target resolution from image specs in shape_meta
        target_h, target_w = None, None
        for key, spec in self.shape_meta['obs'].items():
            if key.endswith('image'):
                target_h, target_w = spec['shape'][-2], spec['shape'][-1]
                break
        if target_h is None or target_w is None:
            # Only raise if at least one image is expected
            has_image = any(k.endswith('image') for k in self.shape_meta['obs'])
            if has_image:
                raise RuntimeError("No image observation found in shape_meta to infer target resolution")

        # 2. Build reverse mapping: policy_key → env_key
        # e.g., 'cam_high_image' → 'cam_high'
        policy_to_env_key = {}
        for env_key in ['cam_high', 'cam_low', 'cam_left_wrist', 'cam_right_wrist']:
            policy_key = env_key + '_image'
            policy_to_env_key[policy_key] = env_key

        # 3. Extract ONLY the keys in shape_meta
        for policy_key in self.shape_meta['obs']:
            if policy_key.endswith('image'):
                # Map policy key to env key
                if policy_key not in policy_to_env_key:
                    raise KeyError(f"Don't know how to extract image observation '{policy_key}'")
                env_key = policy_to_env_key[policy_key]

                if 'images' not in raw_obs or env_key not in raw_obs['images']:
                    raise KeyError(f"Expected image '{env_key}' not found in env observation")

                img = raw_obs['images'][env_key]  # (H, W, 3), uint8

                # Resize if needed
                if target_h is not None and (img.shape[0] != target_h or img.shape[1] != target_w): # because trossen outputs 640 by 400 so its not good for us. 
                    resized = cv2.resize(
                        img,
                        (target_w, target_h),
                        interpolation=cv2.INTER_AREA
                    ).astype(np.float32)
                else:
                    resized = img.astype(np.float32)

                # CHW format
                resized = np.transpose(resized, (2, 0, 1))
                obs_dict[policy_key] = resized

            else:
                # Low-dimensional state
                if policy_key not in raw_obs:
                    raise KeyError(f"Expected observation '{policy_key}' not found in env output")
                obs_dict[policy_key] = raw_obs[policy_key].astype(np.float32)

        return obs_dict
        
    import os
    from datetime import datetime

    def _init_live_plots(self):
        import matplotlib.pyplot as plt
        import os
        from datetime import datetime

        self._plot_hist = {
            "eef_pos_ts": [],
            "eef_pos_data": [],
            "quat_ts": [],
            "quat_data": [],
            "gripper_ts": [],
            "gripper_data": [],
        }

        plt.ion()

        self._fig, self._axes = plt.subplots(6, 4, figsize=(20, 18))
        self._axes = self._axes.flatten()

        titles = (
            [f"EEF pos dim {i}" for i in range(6)] +
            [f"Quat dim {i}" for i in range(8)] +
            [f"Gripper dim {i}" for i in range(2)]
        )

        for ax, title in zip(self._axes, titles):
            ax.set_title(title)
            ax.set_xlabel("timestep")
            ax.set_ylabel("value")
            ax.grid(True)

        self._fig.tight_layout()

        # ---- SAVE CONFIG ----
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        self._plot_save_dir = f"plots/ts_vs_dataset_{timestamp}"
        os.makedirs(self._plot_save_dir, exist_ok=True)

        self._plot_save_every = 10  # save every N steps

    def _update_live_plots(
        self,
        robot0_eef_pos_ts,
        robot0_eef_quat_ts,
        robot0_gripper_qpos_ts,
        robot0_eef_pos_data,
        robot0_eef_quat_data,
        robot0_gripper_qpos_data,
    ):
        import numpy as np
        import os

        step_idx = len(self._plot_hist["eef_pos_ts"])

        # --- Append history ---
        self._plot_hist["eef_pos_ts"].append(robot0_eef_pos_ts.copy())
        self._plot_hist["eef_pos_data"].append(robot0_eef_pos_data.copy())

        self._plot_hist["quat_ts"].append(robot0_eef_quat_ts.copy())
        self._plot_hist["quat_data"].append(robot0_eef_quat_data.copy())

        self._plot_hist["gripper_ts"].append(robot0_gripper_qpos_ts.copy())
        self._plot_hist["gripper_data"].append(robot0_gripper_qpos_data.copy())

        # --- Convert to arrays ---
        eef_pos_ts = np.asarray(self._plot_hist["eef_pos_ts"])
        eef_pos_data = np.asarray(self._plot_hist["eef_pos_data"])

        quat_ts = np.asarray(self._plot_hist["quat_ts"])
        quat_data = np.asarray(self._plot_hist["quat_data"])

        grip_ts = np.asarray(self._plot_hist["gripper_ts"])
        grip_data = np.asarray(self._plot_hist["gripper_data"])

        ax_idx = 0

        # --- EEF POS (6) ---
        for d in range(6):
            ax = self._axes[ax_idx]
            ax.cla()
            ax.plot(eef_pos_ts[:, d], label="sim", color="blue")
            ax.plot(eef_pos_data[:, d], label="dataset", color="orange")
            ax.set_title(f"EEF pos dim {d}")
            ax.legend()
            ax.grid(True)
            ax_idx += 1

        # --- QUAT (8) ---
        for d in range(8):
            ax = self._axes[ax_idx]
            ax.cla()
            ax.plot(quat_ts[:, d], label="sim", color="blue")
            ax.plot(quat_data[:, d], label="dataset", color="orange")
            ax.set_title(f"Quat dim {d}")
            ax.legend()
            ax.grid(True)
            ax_idx += 1

        # --- GRIPPER (2) ---
        for d in range(2):
            ax = self._axes[ax_idx]
            ax.cla()
            ax.plot(grip_ts[:, d], label="sim", color="blue")
            ax.plot(grip_data[:, d], label="dataset", color="orange")
            ax.set_title(f"Gripper dim {d}")
            ax.legend()
            ax.grid(True)
            ax_idx += 1

        # --- LIVE UPDATE ---
        self._fig.canvas.draw()
        self._fig.canvas.flush_events()

        # --- SAVE TO DISK (every N steps) ---
        if step_idx % self._plot_save_every == 0:
            save_path = os.path.join(
                self._plot_save_dir, f"step_{step_idx:05d}.png"
            )
            self._fig.savefig(save_path, dpi=150)


    def reset(self) -> Dict[str, np.ndarray]:
        """
        Reset the environment.
        
        Handles three reset modes:
        1. init_state provided: Reset to specific state (for train demos)
        2. _seed set: Reset with specific seed (for test reproducibility)
        3. Default: Random reset
        
        Returns:
            obs: Dict of observations
        """
        # Reset action step counter
        self.action_step_counter = 0
        
        if self.init_state is not None:
            # Mode 1: Reset to specific init_state
            if not self.has_reset_before:
                # Perform full reset once to ensure proper initialization
                ts = self.env.reset()
                self.has_reset_before = True
            
            # Reset to specific state via physics
            # Note: dm_control doesn't have reset_to() like robomimic,
            # so we reset and then set physics state if available
            ts = self.env.reset()
            
        elif self._seed is not None:
            # Mode 2: Reset with specific seed for reproducibility
            seed = self._seed
            if seed in self.seed_state_map:
                # Use cached state if available
                ts = self.env.reset()
                # Apply cached physics state if dm_control supports it
                try:
                    self.env.physics.set_state(self.seed_state_map[seed])
                except (AttributeError, RuntimeError):
                    # If set_state not available, just use seed-based reset
                    np.random.seed(seed)
                    ts = self.env.reset()
            else:
                # Perform reset with seed
                np.random.seed(seed)
                ts = self.env.reset()
                try:
                    state = self.env.physics.get_state()
                    self.seed_state_map[seed] = state
                except AttributeError:
                    pass  # State caching not available
            
            self._seed = None
        else:
            # Mode 3: Random reset
            ts = self.env.reset()

        ##########################################################
        ## Added initial position of the dataset : 
        if not REAL_SETUP : 
            file = "/home/qtf5422/Desktop/AIRE/ibrl-docker/diffusion_policy/diffusion_policy/data/trossen/real/small_distribution/dataset_tresholded_gripper_normed_reward_dones_cut_end_image_transpose.hdf5"
            self.action_dim = 16
        with h5py.File(file, "r") as f:
            f_data = f["data"]

            sum = np.zeros(self.action_dim, dtype=float)
            n = 0 

            for i in range(len(f_data)) : 
            # for i in range(1) : # only to replay the actions, we only need the demo 0. 
                f_demo_0 = f_data[f"demo_{i}"]
                obs = f_demo_0["obs"]

                for j in range(1) : 
                    if not REAL_SETUP : # simulation : use the quaternion
                        action_j = np.concatenate([obs["robot0_eef_pos"][j, :3], obs["robot0_eef_quat"][j, :4], np.array([obs["robot0_gripper_qpos"][j, 0]]), 
                                            obs["robot0_eef_pos"][j, -3:],obs["robot0_eef_quat"][j, -4:], np.array([obs["robot0_gripper_qpos"][j, 1]])])    
                    sum += action_j 
                    n += 1

            mean = sum/n 

            self.mean = mean

        for i in range(10) : 
            ts = self.env.step(mean)
            self.ts = ts

        ###################################
        obs_display = ts.observation.copy() 
        self._plt_fig_ts = plt.figure()
        self._plt_imgs_ts = plot_observation_images(obs_display, GOOD_CAMERAS["TransferCubeEETask"])

        # Extract and return observations
        obs = self._extract_obs(ts)

        self.saved_actions_dataset = []
        self.saved_actions_policy = []

        return obs
    
    def _get_dataset_ts(self) -> Optional[Any]:
        if self.dataset_obs is None:
            return None
        
        step_idx = self.action_step_counter
        max_steps = len(next(iter(self.dataset_obs.values())))
        if step_idx >= max_steps:
            return None

        observation = {
            'images': {}
        }

        # Handle images: dataset is NOT transposed → images are (3, H, W) per frame
        for img_key in ['cam_high_image', 'cam_low_image', 'cam_left_wrist_image', 'cam_right_wrist_image']:
            if img_key in self.dataset_obs:
                img_chw = self.dataset_obs[img_key][step_idx]  # (3, 128, 128)
                # Convert CHW → HWC for compatibility with env observation format and visualization
                img_hwc = np.transpose(img_chw, (1, 2, 0))     # (128, 128, 3)
                # Ensure uint8
                if img_hwc.dtype != np.uint8:
                    img_hwc = (np.clip(img_hwc, 0, 1) * 255).astype(np.uint8)
                cam_name = img_key.replace('_image', '')
                observation['images'][cam_name] = img_hwc

        # Low-dimensional observations (no change needed)
        observation['qpos'] = self.dataset_obs['qpos'][step_idx].astype(np.float32)
        observation['qvel'] = self.dataset_obs['qvel'][step_idx].astype(np.float32)
        observation['robot0_eef_pos'] = self.dataset_obs['robot0_eef_pos'][step_idx].astype(np.float32)
        observation['robot0_eef_quat'] = self.dataset_obs['robot0_eef_quat'][step_idx].astype(np.float32)
        observation['robot0_gripper_qpos'] = self.dataset_obs['robot0_gripper_qpos'][step_idx].astype(np.float32)

        # Dummy TimeStep to match dm_control interface
        class DummyTimeStep:
            def __init__(self, obs):
                self.observation = obs
                self.reward = 0.0
            def last(self):
                return step_idx >= max_steps - 1

        return DummyTimeStep(observation)
    
    def _update_live_image_diff_plots(self, images_ts, images_data):
        import numpy as np
        import cv2

        step_idx = len(next(iter(self._plot_hist["img_mae_ts_data"].values())))

        ax_idx = self._image_plot_ax_start  # index AFTER your state plots

        for cam in self._image_keys:
            img_ts = images_ts[cam].astype(np.float32) / 255.0
            img_data = images_data[cam].astype(np.float32) / 255.0

            # --- Pixel MAE ---
            diff = np.abs(img_ts - img_data)
            mae = diff.mean()

            self._plot_hist["img_mae_ts_data"][cam].append(mae)

            # --- Edge MAE ---
            edges_ts = cv2.Canny((img_ts * 255).astype(np.uint8), 50, 150)
            edges_data = cv2.Canny((img_data * 255).astype(np.uint8), 50, 150)

            edge_mae = np.mean(np.abs(edges_ts.astype(np.float32) - edges_data.astype(np.float32)))
            self._plot_hist["edge_mae_ts_data"][cam].append(edge_mae)

            # --- Convert to arrays ---
            mae_arr = np.asarray(self._plot_hist["img_mae_ts_data"][cam])
            edge_arr = np.asarray(self._plot_hist["edge_mae_ts_data"][cam])

            # --- PLOT PIXEL MAE ---
            ax = self._axes[ax_idx]
            ax.cla()
            ax.plot(mae_arr, label="pixel MAE", color="red")
            ax.set_title(f"{cam} – Pixel diff")
            ax.grid(True)
            ax.legend()
            ax_idx += 1

            # --- PLOT EDGE MAE ---
            ax = self._axes[ax_idx]
            ax.cla()
            ax.plot(edge_arr, label="edge MAE", color="purple")
            ax.set_title(f"{cam} – Edge diff")
            ax.grid(True)
            ax.legend()
            ax_idx += 1

        # --- LIVE UPDATE ---
        self._fig.canvas.draw()
        self._fig.canvas.flush_events()


    def capture_cameras(self, ts, images_keys):
        """Capture all camera images from self.ts and resize to server resolution"""
        images = {}
        for cam_name in images_keys : 
            image = ts.observation['images'][cam_name]
            # Resize to 128x128
            resized = cv2.resize(image, (128, 128), interpolation=cv2.INTER_AREA)
            images[cam_name] = resized.copy()

        ts.observation['images'] = images # reassign to ts. 
            
        return ts
    
    def step(self, action: np.ndarray) -> Tuple[Dict[str, np.ndarray], float, bool, Dict]:

        images_keys =['cam_high', 'cam_low', 'cam_left_wrist', 'cam_right_wrist']

        """
        Execute one step of the environment.
        
        Args:
            action: (16,) or (14,) action array for dual-arm control
                    (ignored if dataset actions are being used)
            
        Returns:
            obs: Dict of observations
            reward: Scalar reward
            done: Episode termination flag
            info: Additional info dict
        """
        # # Increment action counter for next step
        self.action_step_counter += 1

        # Step the environment
        action[:8] = self.mean[:8] # keep the left robot immobile => only block the left arm for single arm manipulation. 
        ts = self.env.step(action)
        ts = self.capture_cameras(ts, images_keys)
        images_ts = ts.observation["images"]

        # Plot the sim action : 
        self._plt_imgs_ts = set_observation_images(ts.observation, self._plt_imgs_ts, GOOD_CAMERAS["TransferCubeEETask"])

        # Override observation with dataset observation if available
        obs = self._extract_obs(ts)

        # Extract reward and done flag
        reward = float(ts.reward) if ts.reward is not None else 0.0
        done = ts.last()


        return obs, reward, done, {}
    
    def render(self, mode: str = 'rgb_array') -> np.ndarray:
        """
        Render the current observation.
        
        Args:
            mode: Render mode (only 'rgb_array' supported)
            
        Returns:
            img: RGB image as (H, W, 3) uint8 array in [0, 255]
        """
        if self.render_cache is None:
            raise RuntimeError('Must call reset() or step() before render()')
        
        # Convert from float [0, 1] to uint8 [0, 255]
        img = (self.render_cache * 255).astype(np.uint8)
        return img
    
    def seed(self, seed: Optional[int] = None) -> None:
        """
        Set the random seed for the environment.
        
        Args:
            seed: Random seed value
        """
        np.random.seed(seed)
        self._seed = seed

    def close_plot(self):
        """
        Close the matplotlib figure when episode is done.
        Should be called when done=True to clean up visualization resources.
        """
        if hasattr(self, '_plt_fig_ts') and self._plt_fig_ts is not None:
            plt.close(self._plt_fig_ts)
            self._plt_fig_ts = None
            self._plt_imgs_ts = None
            plt.close('all')  # Close all figures to be safe
            plt.pause(0.001)  # Force GUI update