from typing import Optional
from collections import defaultdict, deque

import torch
import numpy as np
from common_utils import ibrl_utils as utils
import common_utils

#from dm_control import suite
from trossen_arm_mujoco.utils import make_sim_env
from trossen_arm_mujoco.ee_sim_env import TransferCubeEETask
from trossen_arm_mujoco.ee_sim_env import plot_observation_images, set_observation_images
from matplotlib import pyplot as plt
import time
import os
from PIL import Image
import torch
import h5py
from scipy.spatial.transform import Rotation as R
from scipy.optimize import minimize

from env.trossen_env import Trossen_env 
from env.quintic_planner import SmoothTrajectoryGenerator
from env.trajectory_thread import TrajectoryExecutor

DATASET_PATH = "/home/qtf5422/Desktop/AIRE/ibrl-docker/sim_recorder_ee_real/server/data/dataset_tresholded_wr_shifted_delta_gripper_normed_cut_end_normalized.hdf5"

GRIPPER_MIN = 0.0
GRIPPER_MAX = 0.04

DENORMALIZE_GRIPPER = False

# Camera configurations for different tasks
GOOD_CAMERAS = {
    "TransferCubeEETask": ["cam_high", "cam_low", "cam_left_wrist", "cam_right_wrist"],
}

DEFAULT_CAMERA = "cam_high"

# State keys for observation
DEFAULT_STATE_KEYS = ["qpos", "qvel", "env_state"]
STATE_KEYS = {
    "TransferCubeEETask": DEFAULT_STATE_KEYS,
}

# State shape: qpos(16) + qvel(16) + env_state(7) = 39
STATE_SHAPE = {
    "TransferCubeEETask": (39,),
}

USE_QUINTIC_POLYNOMIAL = False

# Proprioceptive keys: robot0_eef_pos (6) and robot0_eef_quat(8) and robot0_gripper_qpos(2). 
PROP_KEYS = ["robot0_eef_pos", "robot0_eef_quat", "robot0_gripper_qpos"]
PROP_DIM = 16 
GRIPPER_INDICES = [6, 13] 


class PixelTrossen:
    def __init__(
        self,
        env_name,
        robots, 
        episode_length,
        *,
        reward_shaping=False,
        image_size=224,
        rl_image_size=96,
        device="cuda",
        camera_names=None,
        rl_cameras=None,
        env_reward_scale=1.0,
        end_on_success=True,
        use_state=False,
        obs_stack=1,
        state_stack=1,
        prop_stack=1,
        cond_action=0,
        flip_image=False,
        ctrl_delta=True,
        record_sim_state: bool = False,
        onscreen_render: bool = True,
        denormalization_path_bc : str = "",
        initial_position_file : str = "", 
        denormalization_path_rl : str = ""
    ):
        
        print("rl_image_size inside trossen is : ", rl_image_size)
        print("image_size inside trossen : ", image_size)
        self._plt_fig = None
        self._plt_imgs = None

        self.max_reward_counter = 0
        self.max_reward_target = 10

        self.ts = None


        if camera_names is None:
            camera_names = [DEFAULT_CAMERA]
        if rl_cameras is None:
            rl_cameras = ["cam_high"]
            
        assert isinstance(camera_names, list)
        self.camera_names = camera_names
        self.ctrl_delta = ctrl_delta
        self.record_sim_state = record_sim_state
        self.onscreen_render = onscreen_render
        self.denormalization_path_bc = denormalization_path_bc
        self.initial_position_file = initial_position_file
        self.denormalization_path_rl = denormalization_path_rl
        
        # Map environment names to task classes
        task_map = {
            "TransferCubeEETask": TransferCubeEETask,
        }
        
        if env_name not in task_map:
            print("the env is : ", env_name)
            raise ValueError(f"Unknown environment: {env_name}")
        
        # Create the Trossen environment
        cam_list = ["cam_high", "cam_low", "cam_left_wrist", "cam_right_wrist"]

        self.control_frequency = 20
        follower_left_ip = '192.168.1.5'
        follower_right_ip ='192.168.1.3'
        control_dt = 1.0 / self.control_frequency
        model = "wxai_v0"
        max_number_steps = 1000
        self.env = Trossen_env(follower_left_ip, follower_right_ip, control_dt, model, max_episode_steps=max_number_steps)

    
        self.rl_cameras = rl_cameras if isinstance(rl_cameras, list) else [rl_cameras]
        self.image_size = image_size
        self.rl_image_size = rl_image_size or image_size
        self.env_reward_scale = env_reward_scale
        self.end_on_success = end_on_success
        self.use_state = use_state
        self.state_keys = STATE_KEYS[env_name]
        self.prop_keys = PROP_KEYS
        self.flip_image = flip_image
        self.episode_length = episode_length

        self.resize_transform = None
        if self.rl_image_size != self.image_size:
            self.resize_transform = utils.get_rescale_transform((self.rl_image_size, self.rl_image_size))

        # Action dimension: 16 (3d position, 3d angle axis and 1d gripper for both arms)
        self.action_dim: int = 14
        self._observation_shape: tuple[int, ...] = (3 * obs_stack, rl_image_size, rl_image_size)
        print("self._observation_shape inside trossen wrapper : ", self._observation_shape)
        self._state_shape: tuple[int] = (STATE_SHAPE[env_name][0] * state_stack,)
        self.prop_shape: tuple[int] = (PROP_DIM * prop_stack,)
        self.device = device

        self.time_step = 0
        self.episode_reward = 0
        self.episode_extra_reward = 0
        self.terminal = True

        self.obs_stack = obs_stack
        self.state_stack = state_stack
        self.prop_stack = prop_stack
        self.cond_action = cond_action
        self.past_obses = defaultdict(list)
        self.past_actions = deque(maxlen=self.cond_action)

####################################################################
        self.outside_counter = 0
        self.OUTSIDE_THRESHOLD = 5
####################################################################

        self.speed_norms = []  # stores tuples: (speed_norm, color)


    @property
    def observation_shape(self):
        """Get the shape of the observation: pixel or state."""
        if self.use_state:
            return self._state_shape
        else:
            return self._observation_shape

    def _extract_images(self, obs):
        """Extract and process observations from the environment."""
        high_res_images = {}
        rl_obs = {}

        if self.use_state: #  not our case we dont enter this if section. 
            states = []
            for key in self.state_keys:
                if key in obs:
                    states.append(obs[key])
            state = torch.from_numpy(np.concatenate(states).astype(np.float32))
            
            self.past_obses["state"].append(state)
            rl_obs["state"] = utils.concat_obs(
                len(self.past_obses["state"]) - 1, self.past_obses["state"], self.state_stack
            ).to(self.device)

        # Extract proprioceptive information
        props = []
        for key in self.prop_keys: # here we extract the prop keys from the simulator. 
            props.append(obs[key])
        prop = torch.from_numpy(np.concatenate(props).astype(np.float32))
        
        self.past_obses["prop"].append(prop) # here we gather the proprio sensors into one : mocap_pose, gripper_ctrl ...
        rl_obs["prop"] = utils.concat_obs(
            len(self.past_obses["prop"]) - 1, self.past_obses["prop"], self.prop_stack
        ).to(self.device)

        # Process camera images
        for camera_name in self.camera_names:                
            image_obs = obs[camera_name]
            if self.flip_image:
                print("enters the flipping module : ")
                image_obs = image_obs[::-1]
            image_obs = torch.from_numpy(image_obs.copy()).permute([2, 0, 1])

            # Keep high-res version for rendering
            high_res_images[camera_name] = image_obs
            if camera_name not in self.rl_cameras:
                continue

            rl_image_obs = image_obs

            if self.resize_transform is not None: 
                rl_image_obs = self.resize_transform(rl_image_obs.to(self.device)) 
            self.past_obses[camera_name].append(rl_image_obs)
            rl_obs[camera_name] = utils.concat_obs(
                len(self.past_obses[camera_name]) - 1,
                self.past_obses[camera_name],
                self.obs_stack,
            )

        if self.record_sim_state:
            # Record full simulator state
            sim_state = obs.get("env_state", np.array([]))
            rl_obs["sim_state"] = torch.from_numpy(sim_state)
            
            for key in self.state_keys:
                rl_obs[key] = torch.from_numpy(obs[key])

        return rl_obs, high_res_images

    def load_episode_from_hdf5(self, hdf5_path: str = "/home/qtf5422/Desktop/AIRE/ibrl-docker/sim_recorder_ee_real/server/data/dataset.hdf5", demo_id: int = 0):
        """Load actions from a specific demo in HDF5."""
        with h5py.File(hdf5_path, 'r') as f:
            demo_group = f[f'data/demo_{demo_id}']
            actions = np.array(demo_group['actions'])  # Shape: (T, action_dim)
            print(f"Loaded demo_{demo_id} with {len(actions)} steps")
            return actions

    def load_demo_actions_and_obs(self, dataset_path, demo_name="demo_0"):
        """
        Loads actions and observations from the HDF5 dataset for a specific demo.
        
        Returns:
            actions: (T, 16) array of actions
            obs_dict: dict with keys 'cam_high', 'cam_low', 'cam_left_wrist', 'cam_right_wrist', 'prop'
                    images are (T, 128, 128, 3) after transposing channels
        """
        with h5py.File(dataset_path, "r") as f:
            demo = f[f"data/{demo_name}"]
            actions = demo["actions"][:]

            quat_group = None

            obs_group = demo["obs"]
            obs_dict = {
                "cam_high": np.transpose(obs_group["cam_high_image"][:], (0, 2, 3, 1)),
                "cam_low": np.transpose(obs_group["cam_low_image"][:], (0, 2, 3, 1)),
                "cam_left_wrist": np.transpose(obs_group["cam_left_wrist_image"][:], (0, 2, 3, 1)),
                "cam_right_wrist": np.transpose(obs_group["cam_right_wrist_image"][:], (0, 2, 3, 1)),
                "qpos": obs_group["qpos"][:], 
                "qvel": obs_group["qvel"][:],
                "robot0_eef_pos" : obs_group["robot0_eef_pos"][:], 
                "robot0_eef_quat" : obs_group["robot0_eef_quat"][:], 
                "robot0_gripper_qpos" : obs_group["robot0_gripper_qpos"][:]
            }      

        return actions, obs_dict, quat_group
  
    def denormalize_gripper_actions(self, actions):
        """
        Denormalize gripper dimensions (indices 7 and 15) from [-1, 1] to [0, 0.04].
        Other action dimensions are unchanged.

        Args:
            actions (np.ndarray): Shape (..., 8), grippers assumed in [-1, 1]

        Returns:
            np.ndarray: Same shape, with grippers in [0, 0.04]
        """
        actions = np.array(actions, copy=True)
        grippers = actions[..., GRIPPER_INDICES]

        # Denormalize: [-1, 1] → [0, 0.04]
        grippers_denorm = (grippers + 1) / 2 * (GRIPPER_MAX - GRIPPER_MIN) + GRIPPER_MIN

        # Optional: clamp to [0, 0.04] to handle numerical errors
        grippers_denorm = np.clip(grippers_denorm, GRIPPER_MIN, GRIPPER_MAX)

        actions[..., GRIPPER_INDICES] = grippers_denorm
        return actions

    def clip_action_cartesian_positions(self, action):
        """
        Clips the Cartesian positions in a full bimanual action vector to workspace bounds:
        - x, y ∈ [-0.605, 0.605]
        - z   ∈ [0.0,   0.4]

        Expected action structure (length = 16):
        [lx, ly, lz, lqx, lqy, lqz, lqw, lgrip, rx, ry, rz, rqx, rqy, rqz, rqw, rgrip]

        Args:
            action (array-like): Full action vector with left/right cartesian + quaternion + gripper

        Returns:
            np.ndarray: Action vector with clipped Cartesian positions
        """
        action = np.array(action, copy=True)  # avoid modifying original if not intended

        # Clip left arm (indices 0, 1, 2)
        action[0] = np.clip(action[0], -0.605, 0.605)
        action[1] = np.clip(action[1], -0.605, 0.605)
        action[2] = np.clip(action[2], 0.0,     0.4)

        # Clip right arm (indices 8, 9, 10)
        action[8] = np.clip(action[8], -0.605, 0.605)
        action[9] = np.clip(action[9], -0.605, 0.605)
        action[10] = np.clip(action[10], 0.0,    0.4)

        return action

    def cube_outside_initial_box(self):
        terminal = False

        # Define all collision geom names that belong to the red cube
        RED_CUBE_GEOMS = {"subcube1", "subcube2", "subcube3", "subcube4"}
        GRIPPER_GEOM = "right/gripper_follower_left"

        # Build set of contact pairs
        contact_pairs = set()
        for i in range(self.env.physics.data.ncon):
            g1_id = self.env.physics.data.contact[i].geom1
            g2_id = self.env.physics.data.contact[i].geom2
            g1_name = self.env.physics.model.id2name(g1_id, "geom")
            g2_name = self.env.physics.model.id2name(g2_id, "geom")
            if g1_name and g2_name:
                contact_pairs.add((g1_name, g2_name))
                contact_pairs.add((g2_name, g1_name))

        # Check if ANY red cube geom touches the right gripper
        touch_right_gripper = any(
            (cube_geom, GRIPPER_GEOM) in contact_pairs
            for cube_geom in RED_CUBE_GEOMS
        )

        obs = self.env.task.get_observation(self.env.physics)
        env_state = obs["env_state"]
        cube_x_y = env_state[:3]

        x_range = [-0.1, 0.2]
        y_range = [-0.15, 0.025]

        # Check if cube is outside allowed zone and not touching gripper
        outside_zone = (
            (cube_x_y[0] < x_range[0] or cube_x_y[0] > x_range[1]) or
            (cube_x_y[1] < y_range[0] or cube_x_y[1] > y_range[1])
        ) and not touch_right_gripper and cube_x_y[2] <= 0.15 # changed to 0.02

        if outside_zone:
            self.outside_counter += 1
            print(f"Cube outside limits: {self.outside_counter}/{self.OUTSIDE_THRESHOLD}")
        else:
            # Reset counter if cube comes back inside
            self.outside_counter = 0

        # Trigger terminal only if outside for 5 consecutive checks
        if self.outside_counter >= self.OUTSIDE_THRESHOLD:
            print("Cube has been outside for too long, restart episode")
            terminal = True
            self.outside_counter = 0  # reset counter after triggering

        return terminal

    def quat_to_angle_axis(self, quats):
        """
        Convert quaternions [w, x, y, z] to angle-axis (rotvec).
        Input: (N, 4) — w first (MuJoCo convention)
        Output: (N, 3)
        """
        if quats.ndim != 2 or quats.shape[-1] != 4:
            raise ValueError(f"Expected shape (N, 4), got {quats.shape}")
        # Convert to SciPy order: [x, y, z, w]
        quats_xyzw = np.stack([quats[:, 1], quats[:, 2], quats[:, 3], quats[:, 0]], axis=-1)
        rot = R.from_quat(quats_xyzw)
        return rot.as_rotvec()  # (N, 3)

    def action_aa_to_quat(self, action_aa: np.ndarray) -> np.ndarray:
        """
        Convert a single 14D action (angle-axis) to 16D action (quaternion).

        Input:
            action_aa: np.ndarray of shape (14,)
                    [l_pos(3), l_aa(3), l_grip(1), r_pos(3), r_aa(3), r_grip(1)]

        Output:
            action_quat: np.ndarray of shape (16,)
                        [l_pos(3), l_quat(4), l_grip(1), r_pos(3), r_quat(4), r_grip(1)]
                        Quaternions in [w, x, y, z] order (MuJoCo convention)
        """
        if action_aa.shape != (14,):
            raise ValueError(f"Expected shape (14,), got {action_aa.shape}")

        # Unpack
        l_pos = action_aa[0:3]
        l_aa = action_aa[3:6]
        l_grip = action_aa[6:7]

        r_pos = action_aa[7:10]
        r_aa = action_aa[10:13]
        r_grip = action_aa[13:14]

        # Convert angle-axis → rotation → quat (SciPy: [x,y,z,w])
        l_quat_xyzw = R.from_rotvec(l_aa).as_quat()   # (4,)
        r_quat_xyzw = R.from_rotvec(r_aa).as_quat()   # (4,)

        # Reorder to MuJoCo convention: [w, x, y, z]
        l_quat_wxyz = np.array([l_quat_xyzw[3], l_quat_xyzw[0], l_quat_xyzw[1], l_quat_xyzw[2]])
        r_quat_wxyz = np.array([r_quat_xyzw[3], r_quat_xyzw[0], r_quat_xyzw[1], r_quat_xyzw[2]])

        # Assemble 16D action
        action_quat = np.concatenate([
            l_pos, l_quat_wxyz, l_grip,
            r_pos, r_quat_wxyz, r_grip
        ])

        return action_quat  # (16,)

    def denormalize_action_deltas(self, action_normalized: np.ndarray, path_json = "/home/qtf5422/Desktop/AIRE/ibrl-docker/data/cube_picking_and_placing_ee/delta/delta_action_stats.json") -> np.ndarray:
        """
        Denormalize delta position, rotation, and gripper from [-1, 1] to original ranges.
        
        Formula: a_raw = (a_norm + 1) * (max - min) / 2 + min
        
        Args:
            action_normalized: (14,) array [pos_L_3d, rot_L_3d, grip_L_1d, pos_R_3d, rot_R_3d, grip_R_1d]
        
        Returns:
            action_denorm: (14,) array with denormalized position, rotation, and gripper values
        """
        import json
        from pathlib import Path
        
        if action_normalized.shape != (14,):
            raise ValueError(f"Expected shape (14,), got {action_normalized.shape}")
        
        action_denorm = action_normalized.copy()
        
        # === Load stats from JSON file ===
        stats_file = Path(path_json)
        with open(stats_file, 'r') as f:
            stats = json.load(f)
        
        # Extract Position/Rotation Delta Min/Max from JSON
        pos_x_min, pos_x_max = stats["Pos X Delta"]["min"], stats["Pos X Delta"]["max"]
        pos_y_min, pos_y_max = stats["Pos Y Delta"]["min"], stats["Pos Y Delta"]["max"]
        pos_z_min, pos_z_max = stats["Pos Z Delta"]["min"], stats["Pos Z Delta"]["max"]
        
        rot_x_min, rot_x_max = stats["Rot X Delta"]["min"], stats["Rot X Delta"]["max"]
        rot_y_min, rot_y_max = stats["Rot Y Delta"]["min"], stats["Rot Y Delta"]["max"]
        rot_z_min, rot_z_max = stats["Rot Z Delta"]["min"], stats["Rot Z Delta"]["max"]
        
        # === Gripper Min/Max (normalized with min=0, max=0.04) ===
        grip_min, grip_max = 0.0, 0.04
        
        # Denormalization formula: a_raw = (a_norm + 1) * (max - min) / 2 + min
        
        # --- LEFT ARM ---
        # Position deltas (indices 0, 1, 2)
        action_denorm[0] = (action_normalized[0] + 1.0) * (pos_x_max - pos_x_min) / 2.0 + pos_x_min
        action_denorm[1] = (action_normalized[1] + 1.0) * (pos_y_max - pos_y_min) / 2.0 + pos_y_min
        action_denorm[2] = (action_normalized[2] + 1.0) * (pos_z_max - pos_z_min) / 2.0 + pos_z_min
        
        # Rotation deltas (indices 3, 4, 5)
        action_denorm[3] = (action_normalized[3] + 1.0) * (rot_x_max - rot_x_min) / 2.0 + rot_x_min
        action_denorm[4] = (action_normalized[4] + 1.0) * (rot_y_max - rot_y_min) / 2.0 + rot_y_min
        action_denorm[5] = (action_normalized[5] + 1.0) * (rot_z_max - rot_z_min) / 2.0 + rot_z_min
        
        # Gripper (index 6) - denormalize to [0, 0.04]
        action_denorm[6] = (action_normalized[6] + 1.0) * (grip_max - grip_min) / 2.0 + grip_min

        # --- RIGHT ARM ---
        # Position deltas (indices 7, 8, 9)
        action_denorm[7] = (action_normalized[7] + 1.0) * (pos_x_max - pos_x_min) / 2.0 + pos_x_min
        action_denorm[8] = (action_normalized[8] + 1.0) * (pos_y_max - pos_y_min) / 2.0 + pos_y_min
        action_denorm[9] = (action_normalized[9] + 1.0) * (pos_z_max - pos_z_min) / 2.0 + pos_z_min
        
        # Rotation deltas (indices 10, 11, 12)
        action_denorm[10] = (action_normalized[10] + 1.0) * (rot_x_max - rot_x_min) / 2.0 + rot_x_min
        action_denorm[11] = (action_normalized[11] + 1.0) * (rot_y_max - rot_y_min) / 2.0 + rot_y_min
        action_denorm[12] = (action_normalized[12] + 1.0) * (rot_z_max - rot_z_min) / 2.0 + rot_z_min
        
        # Gripper (index 13) - denormalize to [0, 0.04]
        action_denorm[13] = (action_normalized[13] + 1.0) * (grip_max - grip_min) / 2.0 + grip_min
        
        return action_denorm

    def _apply_delta_rotation_to_quat(self, q_curr: np.ndarray, a_delta: np.ndarray) -> np.ndarray:
        """
        Apply delta rotation (axis-angle) to current quaternion.
        
        Applies local frame rotation: q_target = q_curr ⊗ q_delta
        
        Args:
            q_curr: (4,) quaternion in [w, x, y, z] format (MuJoCo convention)
            a_delta: (3,) axis-angle delta rotation vector
        
        Returns:
            q_target: (4,) quaternion in [w, x, y, z] format
        """
        # Convert current quat to scipy format [x, y, z, w]
        q_curr_scipy = np.array([q_curr[1], q_curr[2], q_curr[3], q_curr[0]])
        
        # Convert axis-angle delta to quaternion (scipy format)
        q_delta_scipy = R.from_rotvec(a_delta).as_quat()  # [x, y, z, w]
        
        # Compose: q_target = q_curr ⊗ q_delta (local frame)
        R_curr = R.from_quat(q_curr_scipy)
        R_delta = R.from_quat(q_delta_scipy)
        R_target = R_curr * R_delta  # right-multiply = local frame
        q_target_scipy = R_target.as_quat()
        
        # Convert back to MuJoCo format [w, x, y, z]
        q_target = np.array([
            q_target_scipy[3],  # w
            q_target_scipy[0],  # x
            q_target_scipy[1],  # y
            q_target_scipy[2]   # z
        ])
        
        return q_target

    def delta_to_absolute_action(self, action_delta: np.ndarray, obs_current: dict) -> np.ndarray:
        """
        Convert 14D delta action to 16D absolute action for simulator.
        
        Converts:
        - Delta positions to absolute positions: pos_target = pos_current + delta_pos
        - Delta rotations to absolute quaternions: applies delta rotation to current quaternion
        - Keeps gripper absolute (no modification)
        
        Args:
            action_delta: (14,) [pos_delta_L_3d, rot_delta_L_3d, grip_L_1d, pos_delta_R_3d, rot_delta_R_3d, grip_R_1d]
            obs_current: dict with 'robot0_eef_pos' (6,), 'robot0_eef_quat' (8,), 'robot0_gripper_qpos' (2,)
        
        Returns:
            action_absolute: (16,) [pos_L_3d, quat_L_4d, grip_L_1d, pos_R_3d, quat_R_4d, grip_R_1d]
        """
        if action_delta.shape != (14,):
            raise ValueError(f"Expected shape (14,), got {action_delta.shape}")
        
        # Extract current observations
        eef_pos = obs_current["robot0_eef_pos"]  # (6,)
        eef_quat = obs_current["robot0_eef_quat"]  # (8,) [wxyz_left, wxyz_right]

        # --- LEFT ARM ---
        pos_curr_left = eef_pos[0:3]
        quat_curr_left = eef_quat[0:4]  # [w, x, y, z]
        grip_left = action_delta[6:7]  # Absolute gripper (no change needed)
        
        # Absolute position
        pos_delta_left = action_delta[0:3]
        pos_target_left = pos_curr_left + pos_delta_left
        
        # Absolute quaternion
        rot_delta_left = action_delta[3:6]  # axis-angle
        quat_target_left = self._apply_delta_rotation_to_quat(quat_curr_left, rot_delta_left)
        
        # --- RIGHT ARM ---
        pos_curr_right = eef_pos[3:6]
        quat_curr_right = eef_quat[4:8]  # [w, x, y, z]
        grip_right = action_delta[13:14]  # Absolute gripper
        
        # Absolute position
        pos_delta_right = action_delta[7:10]
        pos_target_right = pos_curr_right + pos_delta_right
        
        # Absolute quaternion
        rot_delta_right = action_delta[10:13]  # axis-angle
        quat_target_right = self._apply_delta_rotation_to_quat(quat_curr_right, rot_delta_right)

        # Assemble 16D action
        action_absolute = np.concatenate([
            pos_target_left, quat_target_left, grip_left,
            pos_target_right, quat_target_right, grip_right
        ])
    
        return action_absolute

    def convert_action_for_simulator(self, action: np.ndarray, obs_current: dict) -> np.ndarray:
        """
        Full pipeline to convert normalized delta action to absolute action for simulator.
        
        Steps:
        1. Denormalize action deltas (including gripper)
        2. Apply deltas to current observations to get absolute positions/quaternions
        3. Return 16D absolute action
        
        Args:
            action: (14,) normalized delta action from dataset
            obs_current: dict with current observations
        
        Returns:
            action_sim: (16,) absolute action for simulator
        """
        # Denormalize
        print("action shape : ", action.shape)
        action_denorm = self.denormalize_action_deltas(action)

        # Convert to absolute
        action_sim = self.delta_to_absolute_action(action_denorm, obs_current)
        
        return action_sim

    def is_eef_outside_table(self, ts, x_lim=0.605, y_lim=0.38, z_max=0.6):

        pos = ts.observation["robot0_eef_pos"]
        x_l, y_l, z_l = pos[0], pos[1], pos[2]
        x_r, y_r, z_r = pos[3], pos[4], pos[5]
        
        # Check if left arm is in valid workspace
        left_valid = (
            (-x_lim <= x_l <= x_lim) and
            (-y_lim <= y_l <= y_lim) and
            (z_l <= z_max)
        )

        # Check if right arm is in valid workspace
        right_valid = (
            (-x_lim <= x_r <= x_lim) and
            (-y_lim <= y_r <= y_lim) and
            (z_r <= z_max)
        )

        if not right_valid : 
            print("the right arm is in a wrong configuration")
        
        # Return 1 if either arm is outside valid workspace (i.e., should stop episode)
        return 0 if right_valid else 1

    def delta_action_to_absolute(self, delta_action, current_left_pos, current_left_quat, 
                                current_right_pos, current_right_quat):
        """
        Convert 14D delta action to 14D absolute action (pos + axis-angle + gripper per arm).
        
        Args:
            delta_action: (14,) — [dpos_L(3), drot_L(3), grip_L(1), dpos_R(3), drot_R(3), grip_R(1)]
            current_left_pos: (3,) — current left EE position
            current_left_quat: (4,) — current left EE quat [w, x, y, z]
            current_right_pos: (3,) — current right EE position
            current_right_quat: (4,) — current right EE quat [w, x, y, z]
            
        Returns:
            absolute_action: (14,) — [pos_L(3), aa_L(3), grip_L(1), pos_R(3), aa_R(3), grip_R(1)]
        """
        # === Left arm ===
        dpos_L = delta_action[0:3]
        drot_L_aa = delta_action[3:6]
        grip_L = delta_action[6:7]

        # New absolute position
        pos_L_abs = current_left_pos + dpos_L

        # Convert current quat to Rotation
        qL_cur_scipy = [current_left_quat[1], current_left_quat[2], current_left_quat[3], current_left_quat[0]]
        R_L_cur = R.from_quat(qL_cur_scipy)

        # Delta rotation (from axis-angle)
        R_L_delta = R.from_rotvec(drot_L_aa)

        # Apply delta: R_new = R_delta * R_current
        R_L_new = R_L_delta * R_L_cur

        # Convert new orientation to axis-angle
        aa_L_abs = R_L_new.as_rotvec()

        # === Right arm ===
        dpos_R = delta_action[7:10]
        drot_R_aa = delta_action[10:13]
        grip_R = delta_action[13:14]

        pos_R_abs = current_right_pos + dpos_R

        qR_cur_scipy = [current_right_quat[1], current_right_quat[2], current_right_quat[3], current_right_quat[0]]
        R_R_cur = R.from_quat(qR_cur_scipy)
        R_R_delta = R.from_rotvec(drot_R_aa)
        R_R_new = R_R_delta * R_R_cur
        aa_R_abs = R_R_new.as_rotvec()

        # Concatenate into 14D absolute action
        absolute_action = np.concatenate([
            pos_L_abs, aa_L_abs, grip_L,
            pos_R_abs, aa_R_abs, grip_R
        ])

        return absolute_action.astype(np.float64)  # or float32 if your env expects it

    def get_current_ee_state(self, time_step = 4 ): # time step is a dummy variable
        """
        Returns current end-effector poses as:
            (left_pos, left_quat, right_pos, right_quat)
        All as np.ndarray of shape (3,) or (4,)
        Quaternions in [w, x, y, z] format.
        """
        obs = self.env._get_observation()
        eef_pos = obs['robot0_eef_pos']    # (6,)
        eef_quat = obs['robot0_eef_quat']    # (8,)

        # obs = self.obs_dataset
        # eef_pos = obs['robot0_eef_pos'][time_step]      # (6,)
        # eef_quat = obs['robot0_eef_quat'][time_step]    # (8,)

        left_pos = eef_pos[:3]
        right_pos = eef_pos[3:]
        left_quat = eef_quat[:4]
        right_quat = eef_quat[4:]
        
        return left_pos, left_quat, right_pos, right_quat
    
    def get_velocitiy(self) : 
        obs = self.env._get_velocities()

        return obs["velocity"]
    
    def limit_action_step_norm(self, action_step, norm_limit=0.01):
        """
        Limits the norm of the left and right positions in action_step
        while keeping their direction.

        Parameters:
            action_step (list or array): The full action vector.
            norm_limit (float): Maximum allowed norm for left and right positions.

        Returns:
            np.array: Modified action_step with limited norms.
        """
        action_step = np.array(action_step, dtype=float)  # ensure it's a numpy array

        # Extract left and right positions
        left_position = action_step[:3]
        right_position = action_step[7:10]

        # Function to limit norm
        def limit_norm(vector, max_norm):
            norm = np.linalg.norm(vector)
            if norm > max_norm:
                return vector / norm * max_norm
            return vector

        # Apply norm limit
        action_step[:3] = limit_norm(left_position, norm_limit)
        action_step[7:10] = limit_norm(right_position, norm_limit)

        return action_step

    def delta_to_absolute_action_real_setup(self, delta_action, time_step, warmup_bool) : 
        
        if warmup_bool : # running bc
            # print("using bc file")
            # print("delta action shape : ", delta_action.shape)
            # denormalize the action => use the denormalization constants based on the teleoperation values, not the processed file. 
            delta_action = self.denormalize_action_deltas(delta_action, path_json= self.denormalization_path_bc)
        else : # training rl 
            # print("using scaled file")
            delta_action = self.denormalize_action_deltas(delta_action, path_json= self.denormalization_path_rl)

        # self.print_norm(delta_action)

        # delta_action = self.limit_action_step_norm(action_step=delta_action) # => limit the action of the model so that it's not too rushed. 

        # self.print_norm(delta_action)

        # Get real-time EE state from robot
        left_pos, left_quat, right_pos, right_quat = self.get_current_ee_state(time_step)

        # Convert delta → absolute (pos + axis-angle + gripper)
        absolute_action = self.delta_action_to_absolute(
            delta_action, left_pos, left_quat, right_pos, right_quat
        )

        return absolute_action
    
    def constrain_position_to_sphere(self, action_14d, center=np.array([0.0, 0.0, 0.0]), diameter=1.3):
        """
        Clamp the 3D end-effector positions in a 14D absolute action to lie within a sphere.
        
        Args:
            action_14d: np.ndarray of shape (14,)
                Format: [pos_L(3), aa_L(3), grip_L(1), pos_R(3), aa_R(3), grip_R(1)]
            center: np.ndarray of shape (3,) — sphere center in Cartesian space
            diameter: float — diameter of the sphere (default: 0.75 m → radius = 0.375 m)
        
        Returns:
            clamped_action: np.ndarray of shape (14,) with positions constrained
        """
        radius = diameter / 2.0
        action = np.copy(action_14d)

        # Extract positions
        pos_L = action[0:3]
        pos_R = action[7:10]

        # Constrain left arm
        pos_L_rel = pos_L - center
        dist_L = np.linalg.norm(pos_L_rel)
        if dist_L > radius:
            pos_L = center + (pos_L_rel / dist_L) * radius

        # Constrain right arm
        pos_R_rel = pos_R - center
        dist_R = np.linalg.norm(pos_R_rel)
        if dist_R > radius:
            pos_R = center + (pos_R_rel / dist_R) * radius

        # Write back
        action[0:3] = pos_L
        action[7:10] = pos_R

        return action

    def is_feasible(self, p, tol=1e-9):
        """
        Check if point p satisfies all inequality constraints g(p) >= 0
        """
        constraints = self.combined_constraints(p)
        return all(c >= -tol for c in constraints)

    def combined_constraints(self, p):
        x, y, z = p
        constraints = []

        # Sphere constraint
        center = np.array([0.0, 0.0, 0.0])
        radius = 1.3 / 2.0
        constraints.append(radius - np.linalg.norm(p - center))

        # 2D boundary constraints in x-y
        # Remove inner circle and replace with x >= 0
        constraints.append(x - (0.25))                  # x >= 0.2
        constraints.append(0.8 - np.sqrt(x**2 + y**2))  # outer circle r <= 0.8
        constraints.append(2.5*x + 0.2 - y)    # y <= 2.5*x + 0.2
        constraints.append(y - (-2.5*x - 0.2)) # y >= -2.5*x - 0.2
        constraints.append(0.3 - y)            # y <= 0.4
        constraints.append(y - (-0.3))         # y >= -0.4

        # z constraint
        constraints.append(z - (-0.02))        # z >= -0.02

        return constraints

    def distance_to_target_3d(self, p, target):
        return np.sum((p - target)**2)

    def closest_point_3d(self, target):
        # ✅ If already feasible, return directly
        if self.is_feasible(target):
            closest_point = target.copy()
            # Keep your safety rule on z if you want
            return closest_point

        # Otherwise, solve constrained optimization
        num_constraints = len(self.combined_constraints(np.zeros(3)))
        cons = [
            {'type': 'ineq', 'fun': lambda p, i=i: self.combined_constraints(p)[i]}
            for i in range(num_constraints)
        ]

        x0 = np.array([0.5, 0.0, 0.0])  # feasible starting point
        res = minimize(
            self.distance_to_target_3d,
            x0=x0,
            args=(target,),
            constraints=cons
        )

        if res.success:
            closest_point = res.x.copy()
            closest_point[2] = max(closest_point[2], 0.0)
            return closest_point
        else:
            raise ValueError("Optimization failed!")
   
    def constrain_absolute_target(self, action) : 
        len_one_arn = len(action) // 2  # ensures integer
        left_position = action[:len_one_arn][:3]
        right_position = action[len_one_arn:][:3]


        action[:3] = self.closest_point_3d(left_position)
        action[7:10] = self.closest_point_3d(right_position)

        return action

    def append_action_to_npy(self, action_step, file_path):
        action_step = np.asarray(action_step)

        # Ensure shape is (1, action_dim)
        if action_step.ndim == 1:
            action_step = action_step[None, :]

        # Create directory if it doesn't exist
        dir_path = os.path.dirname(file_path)
        if dir_path and not os.path.exists(dir_path):
            os.makedirs(dir_path, exist_ok=True)

        if os.path.exists(file_path):
            actions = np.load(file_path)
            actions = np.vstack((actions, action_step))
        else:
            actions = action_step

        np.save(file_path, actions)

    def print_norm(self, action_step) : 
        # Extract left and right positions
        left_position = action_step[:3]
        right_position = action_step[7:10]

        # Convert to numpy arrays for norm computation
        left_position = np.array(left_position)
        right_position = np.array(right_position)

        # Compute norms
        left_norm = np.linalg.norm(left_position)
        right_norm = np.linalg.norm(right_position)

        print("left_norm : ", left_norm)
        print("right_norm : ", right_norm)

    def append_speed_norm(self, velocities, color="blue"):
        """
        Store speed norms in memory for plotting later.

        Args:
            velocities (np.ndarray): array of joint velocities.
            color (str): "blue" or "red" for the point color.
        """
        speed_norm = np.linalg.norm(velocities[6:9])
        self.speed_norms.append((speed_norm, color))

    def plot_speed_norms(self, filename="/home/qtf5422/Desktop/AIRE/ibrl-docker/rl_debug/speed_norms.png"):
        """
        Plot all stored speed norms after the run and save to disk.

        Args:
            filename (str): path where the plot image will be saved.
        """
        import matplotlib.pyplot as plt

        fig, ax = plt.subplots()
        ax.set_xlabel("Time step")
        ax.set_ylabel("Speed norm")
        ax.set_title("Speed Norms After Run")

        for idx, (speed, color) in enumerate(self.speed_norms):
            ax.scatter(idx, speed, color=color)

        plt.savefig(filename)  # save to disk
        plt.show()             # optionally display

    def reset(self) -> tuple[dict[str, torch.Tensor], dict[str, torch.Tensor]]:
        """Reset the environment."""
        self.time_step = 0
        self.episode_reward = 0
        self.episode_extra_reward = 0
        self.terminal = False
        self.past_obses.clear()
        self.past_actions.clear()
        
        for _ in range(self.cond_action):
            self.past_actions.append(torch.zeros(self.action_dim))

        # Reset environment - dm_control returns TimeStep
        ts = self.env.reset()

        ##  ---------------------------------------------
        ## Added initial position of the dataset : 
        file = self.initial_position_file
        with h5py.File(file, "r") as f:
            f_data = f["data"]

            sum = np.zeros(self.action_dim, dtype=float)
            n = 0 

            for i in range(len(f_data)) : 
                f_demo_0 = f_data[f"demo_{i}"]
                obs = f_demo_0["obs"]

                for j in range(10) : 
                    action_j = np.concatenate([obs["robot0_eef_pos"][j, :3], obs["robot0_eef_aa"][j, :3], np.array([obs["robot0_gripper_qpos"][j, 0]]), 
                                        obs["robot0_eef_pos"][j, -3:],obs["robot0_eef_aa"][j, -3:], np.array([obs["robot0_gripper_qpos"][j, 1]])])    
                    sum += action_j 
                    n += 1

            mean = sum/n 

            self.left_aa_origin = mean[3:6]
            self.right_aa_origin = mean[10:]

            print("mean is : ", mean, "mean size : ", mean.shape)

        for i in range(10) : 
            print("First steps : ", i)
            ts = self.env.step(mean)
            self.ts = ts
        
        obs = ts.observation
        obs_display = obs.copy() 

        if 'images' in obs:
            for cam_name, cam_data in obs['images'].items():
                obs[cam_name] = cam_data
            del obs['images']
        
        rl_obs, high_res_images = self._extract_images(obs)

        if self.onscreen_render and self._plt_fig is None:
            self._plt_fig = plt.figure()
            self._plt_imgs = plot_observation_images(obs_display, GOOD_CAMERAS["TransferCubeEETask"])
        

        if self.cond_action > 0:
            past_action = torch.from_numpy(np.stack(self.past_actions)).to(self.device)
            rl_obs["past_action"] = past_action

        if USE_QUINTIC_POLYNOMIAL : 
            self.traj_gen = SmoothTrajectoryGenerator(duration=0.01, num_steps=20)
            self.trajectory_executor = TrajectoryExecutor(self.env)

        return rl_obs, high_res_images
    
    def stop_left_arm_action(self, action_step) : 
        action_step[:7] = np.array([2.53257828e-01, -2.23311606e-04,  1.62405673e-01, -3.15363064e-03,
        2.70620997e-02, -2.59994248e-04,  3.99996154e-02])
        return action_step
        

    def step(self, actions: torch.Tensor, check_od_movement: bool = False, warmup_bool : bool = True) -> tuple[dict, float, bool, bool, dict]:

        if actions.dim() == 1:
            actions = actions.unsqueeze(0)
        num_action = actions.size(0)

        rl_obs = {}
        
        # Record actions
        if self.cond_action > 0:
            for i in range(actions.size(0)):
                self.past_actions.append(actions[i])
            past_action = torch.stack(list(self.past_actions)).to(self.device)
            rl_obs["past_action"] = past_action

        actions = actions.numpy()

        reward = 0
        success = False
        terminal = False
        high_res_images = {}
        
        for i in range(num_action):
            self.time_step += 1

            action_step = actions[i]
            
            print("warmup is : ", warmup_bool)

            action_step = self.delta_to_absolute_action_real_setup(action_step, self.time_step, warmup_bool)

            self.append_action_to_npy(
                action_step,
                "/home/qtf5422/Desktop/AIRE/ibrl-docker/rl_debug/actions_log_before_constrain.npy"
            ) 

            action_step = self.constrain_absolute_target(action_step)

            action_step = self.stop_left_arm_action(action_step) # constrain the left arm to no movement. 

            self.append_action_to_npy(
                action_step,
                "/home/qtf5422/Desktop/AIRE/ibrl-docker/rl_debug/actions_log_after_constrain.npy"
            ) 

            if USE_QUINTIC_POLYNOMIAL : # in this case, we use the thread that executes the trajectory for the motion. 

                ##########################################
                # Generate trajectory
                current_state = self.get_current_ee_state()
                velocities = self.get_velocitiy()
                self.append_speed_norm(velocities, color="red")
                trajectory_actions = self.traj_gen.generate_trajectory(current_state, action_step, velocities)

                # Send trajectory to executor thread
                self.trajectory_executor.send_trajectory(trajectory_actions)
                
                # Get the latest timestep from the executor
                ts = self.trajectory_executor.get_latest_ts()
                
                # Wait until we have a valid timestep
                while ts is None:
                    ts = self.trajectory_executor.get_latest_ts()
                ##########################################

            else : # not using the quintic, simply send to the env :  
                ts = self.env.step(action_step)

            obs = ts.observation
            obs_display = obs.copy()

            if 'images' in obs:
                for cam_name, cam_data in obs['images'].items():
                    obs[cam_name] = cam_data
                del obs['images']    
                
            step_reward = ts.reward if ts.reward is not None else 0
            terminal = ts.last()
            
            curr_rl_obs, curr_high_res_images = self._extract_images(obs)

            if self.onscreen_render and self._plt_imgs is not None:
                self._plt_imgs = set_observation_images(obs_display, self._plt_imgs, GOOD_CAMERAS["TransferCubeEETask"])

            if i == num_action - 1:
                rl_obs.update(curr_rl_obs)
                high_res_images.update(curr_high_res_images)

            reward += step_reward
            self.episode_reward += step_reward

            if step_reward == 1:
                success = True
                if self.end_on_success:
                    terminal = True    

            if self.time_step == self.episode_length : 
                terminal = True
                    
            if terminal:
                break

        reward = reward * self.env_reward_scale
        self.terminal = terminal
        return rl_obs, reward, terminal, success, high_res_images


if __name__ == "__main__":
    from torchvision.utils import save_image

    env = PixelTrossen(
        "TransferCube",
        episode_length=200,
        image_size=256,
        camera_names=GOOD_CAMERAS["TransferCube"],
        rl_cameras=["cam_high"],
        
    )
    
    rl_obs, high_res = env.reset()
    
    # Get the first camera image
    first_camera = GOOD_CAMERAS["TransferCube"][0]
    if first_camera in rl_obs:
        x = rl_obs[first_camera].float() / 255
        print(x.dtype)
        print(x.shape)
        save_image(x, "test_trossen_env.png")
    
    # Test a random step
    action = torch.randn(env.action_dim)
    rl_obs, reward, terminal, success, high_res = env.step(action)
    print(f"Reward: {reward}, Terminal: {terminal}, Success: {success}")