"""
Single-process runner for Trossen AI TransferCubeEETask.

Runs sequential environment rollouts with reward collection.
"""

import os
import numpy as np
import torch
import time
from collections import defaultdict

from diffusion_policy.model.common.rotation_transformer import RotationTransformer
from diffusion_policy.policy.base_image_policy import BaseImagePolicy
from diffusion_policy.common.pytorch_util import dict_apply
from diffusion_policy.env_runner.base_image_runner import BaseImageRunner
from scipy.spatial.transform import Rotation as R

from trossen_arm_mujoco.utils import make_sim_env
from trossen_arm_mujoco.ee_sim_env import TransferCubeEETask
from diffusion_policy.env.trossen.trossen_image_wrapper import TrossenImageWrapper
from diffusion_policy.gym_util.multistep_wrapper import MultiStepWrapper

# Gripper normalization constants (matching your dataset processing)
GRIPPER_MIN = 0.0
GRIPPER_MAX = 0.04


class MultiprocessTrossenImageRunner(BaseImageRunner):
    """
    Single-process runner for Trossen AI bimanual manipulation tasks.
    
    Runs evaluations sequentially for evaluation.
    
    Args:
        output_dir: Directory to save outputs (logs)
        shape_meta: Dict specifying action and observation shapes/types
        n_test: Number of test eval conditions
        test_start_seed: Starting seed for test conditions
        max_steps: Maximum steps per episode
        n_obs_steps: Number of observation steps to stack
        n_action_steps: Number of action steps to execute
        render_obs_key: Which camera to use for rendering
        fps: Frame rate for video recording
        crf: Video quality (lower = better, 0-51)
        past_action: Whether to include past actions in observations
        abs_action: Whether actions are absolute (vs delta)
        cam_list: List of camera names to capture
        xml_path: Path to Mujoco XML scene file
        num_proc: Number of parallel processes to use (ignored in single-process version)
    """
    
    def __init__(
        self,
        output_dir: str,
        shape_meta: dict,
        n_test: int = 22,
        test_start_seed: int = 10000,
        max_steps: int = 200,
        n_obs_steps: int = 2,
        n_action_steps: int = 8,
        render_obs_key: str = 'cam_high',
        fps: int = 10,
        crf: int = 22,
        past_action: bool = False,
        abs_action: bool = False,
        tqdm_interval_sec: float = 1.0,
        cam_list: list = None,
        xml_path: str = "trossen_ai_scene.xml",
        num_proc: int = 10,  # Kept for compatibility but ignored
    ):
        super().__init__(output_dir)
        
        if cam_list is None:
            cam_list = ["cam_high", "cam_low", "cam_left_wrist", "cam_right_wrist"]
        
        # Compute rendering parameters
        dm_control_fps = 20
        steps_per_render = max(dm_control_fps // fps, 1)
        
        # Store configuration
        self.shape_meta = shape_meta
        self.render_obs_key = render_obs_key
        self.cam_list = cam_list
        self.xml_path = xml_path
        self.fps = fps
        self.crf = crf
        self.steps_per_render = steps_per_render
        self.n_obs_steps = n_obs_steps
        self.n_action_steps = n_action_steps
        self.past_action = past_action
        self.max_steps = max_steps
        self.abs_action = abs_action
        self.n_test = n_test
        
        # Setup rotation transformer if needed
        self.rotation_transformer = None
        if abs_action:
            self.rotation_transformer = RotationTransformer('axis_angle', 'rotation_6d')
        
        # Setup evaluation seeds
        self.test_seeds = list(range(test_start_seed, test_start_seed + n_test))
        
        # Create environment
        self.env = self.create_env()
        
        print(f"Initialized single-process runner with {n_test} test conditions")
    
    def create_env(self):
        """Create and wrap the Trossen environment."""
        trossen_env = make_sim_env(
            TransferCubeEETask,
            self.xml_path,
            onscreen_render=False,
            cam_list=self.cam_list,
        )
        
        wrapped_env = TrossenImageWrapper(
            env=trossen_env,
            shape_meta=self.shape_meta,
            init_state=None,
            render_obs_key=self.render_obs_key,
        )
        
        env = MultiStepWrapper(
            wrapped_env,
            n_obs_steps=self.n_obs_steps,
            n_action_steps=self.n_action_steps,
            max_episode_steps=self.max_steps,
        )
        return env
    
    def run(self, policy: BaseImagePolicy) -> dict:
        """
        Execute evaluation rollouts with the given policy sequentially.
        
        Args:
            policy: The policy to evaluate
            
        Returns:
            log_data: Dict of logged metrics
        """
        device = policy.device
        
        results = {}
        t_start = time.time()
        
        with torch.no_grad():
            policy.eval()
            
            for seed in self.test_seeds:
                # Seed environment
                self.env.env.seed(seed)
                
                # Reset environment
                obs = self.env.reset()
                
                # Run episode
                done = False
                max_reward = 0.0
                
                while not done:
                    # Prepare observation (add batch dimension)
                    obs_dict = {k: v[None, ...] for k, v in obs.items()}
                    obs_dict = dict_apply(
                        obs_dict,
                        lambda x: torch.from_numpy(x).to(device=device)
                    )
                    
                    # Get action from policy
                    action_dict = policy.predict_action(obs_dict)
                    
                    # Convert to numpy
                    np_action_dict = dict_apply(
                        action_dict,
                        lambda x: x.detach().to('cpu').numpy(),
                    )
                    action = np_action_dict['action']
                    
                    # Check for invalid actions
                    if not np.all(np.isfinite(action)):
                        raise RuntimeError("Nan or Inf action detected")
                    
                    # Transform actions if needed
                    if self.abs_action:
                        action = self.undo_transform_action(action)
                    
                    # Remove batch dimension
                    action = action[0]
                    
                    # Step environment
                    obs, reward, done, info = self.env.step(action)
                
                # Get final rewards
                rewards = self.env.get_attr('reward')
                max_reward = np.max(rewards)
                results[seed] = float(max_reward)
                
                print(f"Seed {seed}: reward = {max_reward:.3f}")
        
        print(f"Total evaluation time: {time.time() - t_start:.2f}s")
        
        # Compute metrics
        all_rewards = []
        log_data = {}
        
        for seed in sorted(results.keys()):
            max_reward = results[seed]
            all_rewards.append(max_reward)
            log_data[f'test/sim_max_reward_{seed}'] = max_reward
        
        # Aggregate metrics
        mean_reward = np.mean(all_rewards)
        log_data['test/mean_score'] = mean_reward
        print(f"test/mean_score: {mean_reward:.3f}")
        
        return log_data
    
    def convert_angle_axis_to_quaternion(self, action: np.ndarray) -> np.ndarray:
        """
        Convert angle-axis rotation to quaternion for dual-arm actions.
        
        Input format: [pos_left(3), aa_left(3), gripper_left(1), 
                       pos_right(3), aa_right(3), gripper_right(1)] = 14D
        Output format: [pos_left(3), quat_left(4), gripper_left(1), 
                        pos_right(3), quat_right(4), gripper_right(1)] = 16D
        
        Quaternion output format: [w, x, y, z]
        """
        original_shape = action.shape
        action = action.reshape(-1, 14)
        
        N = action.shape[0]
        action_quat = np.zeros((N, 16), dtype=action.dtype)
        
        for i in range(N):
            pos_left = action[i, 0:3]
            aa_left = action[i, 3:6]
            gripper_left = action[i, 6]
            
            pos_right = action[i, 7:10]
            aa_right = action[i, 10:13]
            gripper_right = action[i, 13]
            
            # Convert angle-axis to quaternion
            rot_left = R.from_rotvec(aa_left)
            rot_right = R.from_rotvec(aa_right)
            
            quat_left_scipy = rot_left.as_quat()
            quat_right_scipy = rot_right.as_quat()
            
            # Convert to [w, x, y, z] format
            quat_left = np.array([quat_left_scipy[3], quat_left_scipy[0], 
                                  quat_left_scipy[1], quat_left_scipy[2]])
            quat_right = np.array([quat_right_scipy[3], quat_right_scipy[0], 
                                   quat_right_scipy[1], quat_right_scipy[2]])
            
            # Assemble 16D action
            action_quat[i, 0:3] = pos_left
            action_quat[i, 3:7] = quat_left
            action_quat[i, 7] = gripper_left
            action_quat[i, 8:11] = pos_right
            action_quat[i, 11:15] = quat_right
            action_quat[i, 15] = gripper_right
        
        output_shape = original_shape[:-1] + (16,)
        return action_quat.reshape(output_shape)
    
    def denormalize_gripper(self, action: np.ndarray) -> np.ndarray:
        """
        Denormalize gripper values from [-1, 1] to physical range [0, 0.044].
        
        Gripper indices in 16D action: [7, 15]
        """
        action = action.copy()
        gripper_indices = [7, 15]
        
        grippers_norm = action[..., gripper_indices]
        grippers_physical = ((grippers_norm + 1) / 2) * (GRIPPER_MAX - GRIPPER_MIN) + GRIPPER_MIN
        grippers_physical = np.clip(grippers_physical, GRIPPER_MIN, GRIPPER_MAX)
        
        action[..., gripper_indices] = grippers_physical
        
        return action
    
    def undo_transform_action(self, action: np.ndarray) -> np.ndarray:
        """
        Transform action from policy representation to environment action.
        
        Pipeline:
        1. Transform rotation from 6D to angle-axis
        2. Convert angle-axis to quaternion
        3. Denormalize gripper values
        
        Input: shape (..., 20) for dual-arm with 6D rotation
        Output: shape (..., 16) for dual-arm with quaternion
        """
        raw_shape = action.shape
        
        if raw_shape[-1] == 20:
            action = action.reshape(-1, 2, 10)
        
        # Step 1: 6D rotation to angle-axis
        d_rot = action.shape[-1] - 4
        pos = action[..., :3]
        rot = action[..., 3:3+d_rot]
        gripper = action[..., [-1]]
        
        rot_aa = self.rotation_transformer.inverse(rot)
        action_14d = np.concatenate([pos, rot_aa, gripper], axis=-1)
        
        if raw_shape[-1] == 20:
            action_14d = action_14d.reshape(*raw_shape[:-1], 14)
        
        # Step 2: angle-axis to quaternion
        action_16d = self.convert_angle_axis_to_quaternion(action_14d)
        
        # Step 3: denormalize gripper
        action_final = self.denormalize_gripper(action_16d)
        
        return action_final