"""
Simplified single-environment runner for Trossen AI TransferCubeEETask.

Manages sequential environment rollouts with reward collection.
"""

import os
import wandb
import numpy as np
import torch
import collections
import pathlib
import tqdm

from diffusion_policy.gym_util.multistep_wrapper import MultiStepWrapper
from diffusion_policy.gym_util.video_recording_wrapper import VideoRecordingWrapper, VideoRecorder
from diffusion_policy.model.common.rotation_transformer import RotationTransformer
from diffusion_policy.policy.base_image_policy import BaseImagePolicy
from diffusion_policy.common.pytorch_util import dict_apply
from diffusion_policy.env_runner.base_image_runner import BaseImageRunner
from diffusion_policy.env.trossen.trossen_image_wrapper import TrossenImageWrapper
from scipy.spatial.transform import Rotation as R

# Gripper normalization constants (matching your dataset processing)
GRIPPER_MIN = 0.0
GRIPPER_MAX = 0.04


class SimpleTrossenImageRunner(BaseImageRunner):
    """
    Simple single-environment runner for Trossen AI bimanual manipulation tasks.
    
    Runs evaluations sequentially on one environment at a time.
    
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
        tqdm_interval_sec: Progress bar update interval
        cam_list: List of camera names to capture
        xml_path: Path to Mujoco XML scene file
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
        tqdm_interval_sec: float = 5.0,
        cam_list: list = None,
        xml_path: str = "trossen_ai_scene.xml",
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
        self.tqdm_interval_sec = tqdm_interval_sec
        
        # Setup rotation transformer if needed
        self.rotation_transformer = None
        if abs_action:
            self.rotation_transformer = RotationTransformer('axis_angle', 'rotation_6d')
        
        # Setup evaluation conditions (test only)
        self.eval_conditions = []
        for i in range(n_test):
            self.eval_conditions.append({
                'seed': test_start_seed + i,
                'prefix': 'test/'
            })
        
        print(f"Initialized runner with {len(self.eval_conditions)} test conditions")
    
    def create_env(self):
        """Create a single wrapped environment instance."""
        from trossen_arm_mujoco.utils import make_sim_env
        from trossen_arm_mujoco.ee_sim_env import TransferCubeEETask
        
        # Create base environment
        trossen_env = make_sim_env(
            TransferCubeEETask,
            self.xml_path,
            onscreen_render=False,
            cam_list=self.cam_list,
        )
        
        # Wrap in gym-compatible wrapper
        wrapped_env = TrossenImageWrapper(
            env=trossen_env,
            shape_meta=self.shape_meta,
            init_state=None,
            render_obs_key=self.render_obs_key,
        )
        
        # Apply wrapper chain
        env = MultiStepWrapper(
            VideoRecordingWrapper(
                wrapped_env,
                video_recoder=VideoRecorder.create_h264(
                    fps=self.fps,
                    codec='h264',
                    input_pix_fmt='rgb24',
                    crf=self.crf,
                    thread_type='FRAME',
                    thread_count=1
                ),
                file_path=None,
                steps_per_render=self.steps_per_render,
            ),
            n_obs_steps=self.n_obs_steps,
            n_action_steps=self.n_action_steps,
            max_episode_steps=self.max_steps,
        )
        
        return env
    
    def run_single_episode(self, env, policy: BaseImagePolicy, seed: int) -> np.ndarray:
        """
        Run a single episode with the given policy.
        
        Args:
            env: The environment instance
            policy: The policy to evaluate
            seed: Seed for deterministic reset
            
        Returns:
            rewards: Reward array for the episode
        """
        device = policy.device
        
        # Disable video recording
        env.env.file_path = None
        
        # Seed environment
        env.env.env.seed(seed)
        
        # Reset policy and environment
        policy.reset()
        obs = env.reset()
        
        past_action = None
        
        # Run episode
        done = False    
        step_count = 0
        
        pbar = tqdm.tqdm(
            total=self.max_steps,
            desc=f"Episode seed={seed}",
            leave=False,
            mininterval=self.tqdm_interval_sec,
        )
        
        while not done:
            # Prepare observations - add batch dimension
            np_obs_dict = {}
            for key, value in obs.items():
                # Add batch dimension: (T, ...) -> (1, T, ...)
                np_obs_dict[key] = value[None, ...]
            
            if self.past_action and (past_action is not None):
                np_obs_dict['past_action'] = past_action[
                    :, -(self.n_obs_steps-1):
                ].astype(np.float32)
            
            # Transfer to device
            obs_dict = dict_apply(
                np_obs_dict,
                lambda x: torch.from_numpy(x).to(device=device),
            )
            
            # Get action from policy
            with torch.no_grad():
                action_dict = policy.predict_action(obs_dict)
            
            # Transfer back to CPU
            np_action_dict = dict_apply(
                action_dict,
                lambda x: x.detach().to('cpu').numpy(),
            )
            
            action = np_action_dict['action']
            
            # Check for invalid actions
            if not np.all(np.isfinite(action)):
                print(f"Invalid action detected: {action}")
                raise RuntimeError("Nan or Inf action detected")
            
            # Transform action if needed
            env_action = action
            if self.abs_action:
                env_action = self.undo_transform_action(action)
            
            # Step environment
            obs, reward, done, info = env.step(env_action)
            past_action = action
            
            # Update progress
            step_count += action.shape[1]
            pbar.update(action.shape[1])
        
        pbar.close()
        
        # Get total rewards
        rewards = env.get_attr('reward')
        
        return rewards
    
    def run(self, policy: BaseImagePolicy) -> dict:
        """
        Execute evaluation rollouts with the given policy.
        
        Runs evaluation on all test conditions sequentially.
        
        Args:
            policy: The policy to evaluate
            
        Returns:
            log_data: Dict of logged metrics
        """
        # Create single environment
        env = self.create_env()
        
        # Storage for results
        all_rewards = []
        log_data = dict()
        
        # Run all evaluation conditions
        for i, condition in enumerate(self.eval_conditions):
            print("---------------------------")
            print("simulation number : ", i)
            print("---------------------------")
            seed = condition['seed']
            prefix = condition['prefix']
            
            print(f"Running evaluation {i+1}/{len(self.eval_conditions)} (seed={seed})")
            
            # Run episode
            rewards = self.run_single_episode(env, policy, seed)
            
            # Compute max reward
            max_reward = np.max(rewards)
            all_rewards.append(max_reward)
            
            # Log results
            log_data[prefix + f'sim_max_reward_{seed}'] = max_reward
        
        # Log aggregate metrics
        mean_reward = np.mean(all_rewards)
        log_data['test/mean_score'] = mean_reward
        print(f"test/mean_score: {mean_reward:.3f}")
        
        return log_data
    
    def convert_angle_axis_to_quaternion(self, action: np.ndarray) -> np.ndarray:
        """
        Convert angle-axis rotation to quaternion for dual-arm actions.
        
        This reverses the conversion done in convert_quat_to_angle_axis_in_dataset.
        
        Input format: [pos_left(3), aa_left(3), gripper_left(1), 
                       pos_right(3), aa_right(3), gripper_right(1)] = 14D
        Output format: [pos_left(3), quat_left(4), gripper_left(1), 
                        pos_right(3), quat_right(4), gripper_right(1)] = 16D
        
        Quaternion output format: [w, x, y, z] (matching your recording code)
        
        Args:
            action: Action array with angle-axis rotations, shape (..., 14)
            
        Returns:
            Action array with quaternions, shape (..., 16)
        """
        original_shape = action.shape
        action = action.reshape(-1, 14)  # Flatten to (N, 14)
        
        N = action.shape[0]
        action_quat = np.zeros((N, 16), dtype=action.dtype)
        
        for i in range(N):
            # Extract components
            pos_left = action[i, 0:3]
            aa_left = action[i, 3:6]
            gripper_left = action[i, 6]
            
            pos_right = action[i, 7:10]
            aa_right = action[i, 10:13]
            gripper_right = action[i, 13]
            
            # Convert angle-axis to quaternion
            rot_left = R.from_rotvec(aa_left)
            rot_right = R.from_rotvec(aa_right)
            
            # Get quaternions in scipy format [x, y, z, w]
            quat_left_scipy = rot_left.as_quat()
            quat_right_scipy = rot_right.as_quat()
            
            # Convert to [w, x, y, z] format (your recording format)
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
        
        # Reshape back to original batch shape
        output_shape = original_shape[:-1] + (16,)
        return action_quat.reshape(output_shape)
    
    def denormalize_gripper(self, action: np.ndarray) -> np.ndarray:
        """
        Denormalize gripper values from [-1, 1] to physical range [0, 0.044].
        
        This reverses the normalization: x ↦ 2*(x / 0.044) - 1
        Denormalization: x ↦ ((x + 1) / 2) * 0.044
        
        Gripper indices in 16D action: [6, 15] (left and right grippers)
        
        Args:
            action: Action array with normalized grippers, shape (..., 16)
            
        Returns:
            Action array with denormalized grippers, shape (..., 16)
        """
        action = action.copy()  # Don't modify input
        gripper_indices = [6, 15]
        
        # Extract normalized grippers
        grippers_norm = action[..., gripper_indices]
        
        # Denormalize: [-1, 1] → [0, 0.044]
        grippers_physical = ((grippers_norm + 1) / 2) * (GRIPPER_MAX - GRIPPER_MIN) + GRIPPER_MIN
        
        # Clamp to valid range (safety)
        grippers_physical = np.clip(grippers_physical, GRIPPER_MIN, GRIPPER_MAX)
        
        # Write back
        action[..., gripper_indices] = grippers_physical
        
        return action
    
    def undo_transform_action(self, action: np.ndarray) -> np.ndarray:
        """
        Transform action from policy representation to environment action.
        
        Pipeline:
        1. Transform rotation from 6D to angle-axis (via rotation_transformer)
        2. Convert angle-axis to quaternion (for simulator)
        3. Denormalize gripper values ([-1,1] to [0, 0.044])
        
        Input: Policy action with 6D rotation, shape (..., 20) for dual-arm
               [pos_left(3), rot6d_left(6), gripper_left(1),
                pos_right(3), rot6d_right(6), gripper_right(1)]
        
        Output: Environment action with quaternion, shape (..., 16)
                [pos_left(3), quat_left(4), gripper_left(1),
                 pos_right(3), quat_right(4), gripper_right(1)]
        
        Args:
            action: Policy output action
            
        Returns:
            Transformed action for environment
        """
        raw_shape = action.shape
        
        # Handle dual arm reshaping if needed
        if raw_shape[-1] == 20:
            # Dual arm: (B, T, 20) -> (B*T, 2, 10)
            action = action.reshape(-1, 2, 10)
        
        # Step 1: Transform 6D rotation to angle-axis
        d_rot = action.shape[-1] - 4  # 6 for 6D rotation
        pos = action[..., :3]
        rot = action[..., 3:3+d_rot]
        gripper = action[..., [-1]]
        
        # Transform rotation representation (6D -> angle-axis)
        rot_aa = self.rotation_transformer.inverse(rot)
        
        # Concatenate to get 14D action
        action_14d = np.concatenate([pos, rot_aa, gripper], axis=-1)
        
        # Reshape back if dual arm
        if raw_shape[-1] == 20:
            action_14d = action_14d.reshape(*raw_shape[:-1], 14)
        
        # Step 2: Convert angle-axis to quaternion (14D -> 16D)
        action_16d = self.convert_angle_axis_to_quaternion(action_14d)
        
        # Step 3: Denormalize gripper values
        action_final = self.denormalize_gripper(action_16d)
        
        return action_final