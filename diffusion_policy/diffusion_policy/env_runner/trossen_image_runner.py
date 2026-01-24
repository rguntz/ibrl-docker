"""
Environment runner for Trossen AI TransferCubeEETask.

Manages parallel environment rollouts with video recording and reward collection.
Heavily based on RobomimicImageRunner but adapted for dm_control/Trossen environments.
"""

import os
import wandb
import numpy as np
import torch
import collections
import pathlib
import tqdm
import math
import dill 
import wandb.sdk.data_types.video as wv

from diffusion_policy.gym_util.async_vector_env import AsyncVectorEnv
from diffusion_policy.gym_util.sync_vector_env import SyncVectorEnv
from diffusion_policy.gym_util.multistep_wrapper import MultiStepWrapper
from diffusion_policy.gym_util.video_recording_wrapper import VideoRecordingWrapper, VideoRecorder
from diffusion_policy.model.common.rotation_transformer import RotationTransformer

from diffusion_policy.policy.base_image_policy import BaseImagePolicy
from diffusion_policy.common.pytorch_util import dict_apply
from diffusion_policy.env_runner.base_image_runner import BaseImageRunner
from diffusion_policy.env.trossen.trossen_image_wrapper import TrossenImageWrapper

import sys
import os

import os

# Force headless rendering
os.environ["MUJOCO_GL"] = "egl"      # or "osmesa"
os.environ["EGL_DEVICE_ID"] = "0"    # optional: specify GPU if multi-GPU
os.environ["DISPLAY"] = ""           # disable X11 entirely

# Optional safety
os.environ["MUJOCO_EGL_RENDER_OFFSCREEN"] = "1"


class TrossenImageRunner(BaseImageRunner):
    """
    Environment runner for Trossen AI bimanual manipulation tasks.
    
    Manages parallel environment rollouts with:
    - Multi-camera image recording
    - Reward collection
    - Seed-based deterministic evaluation
    - Video generation for visualization
    
    This runner is designed for evaluating policies trained via diffusion_policy
    on Trossen AI tasks created with TransferCubeEETask.
    
    Args:
        output_dir: Directory to save outputs (videos, logs)
        shape_meta: Dict specifying action and observation shapes/types
        n_train: Number of training eval conditions
        n_train_vis: Number of training conditions to visualize
        n_test: Number of test eval conditions
        n_test_vis: Number of test conditions to visualize
        test_start_seed: Starting seed for test conditions
        max_steps: Maximum steps per episode
        n_obs_steps: Number of observation steps to stack
        n_action_steps: Number of action steps to execute
        render_obs_key: Which camera to use for video recording
        fps: Video frame rate
        crf: Video quality (lower = better, 0-51)
        past_action: Whether to include past actions in observations
        abs_action: Whether actions are absolute (vs delta)
        tqdm_interval_sec: Progress bar update interval
        n_envs: Number of parallel environments
        cam_list: List of camera names to capture
        xml_path: Path to Mujoco XML scene file
    """
    
    def __init__(
        self,
        output_dir: str,
        shape_meta: dict,
        n_train: int = 10,
        n_train_vis: int = 3,
        n_test: int = 22,
        n_test_vis: int = 6,
        test_start_seed: int = 10000,
        max_steps: int = 400,
        n_obs_steps: int = 2,
        n_action_steps: int = 8,
        render_obs_key: str = 'cam_high',
        fps: int = 10,
        crf: int = 22,
        past_action: bool = False,
        abs_action: bool = False,
        tqdm_interval_sec: float = 5.0,
        n_envs: int = None,
        cam_list: list = None,
        xml_path: str = "trossen_ai_scene.xml",
    ):
        print("-----------------------------------------------------------")
        print("we are initializing the trossen image runner")
        super().__init__(output_dir)
        
        if n_envs is None:
            n_envs = n_train + n_test
        
        if cam_list is None:
            cam_list = ["cam_high", "cam_low", "cam_left_wrist", "cam_right_wrist"]
        
        # Compute rendering parameters
        dm_control_fps = 20  # dm_control default physics FPS
        steps_per_render = max(dm_control_fps // fps, 1)
        
        # Create minimal environment metadata for compatibility
        self.env_meta = {
            'env_name': 'TransferCubeEE',
            'cam_list': cam_list,
            'xml_path': xml_path,
        }
        
        # Setup action transformation if needed
        rotation_transformer = None
        if abs_action:
            rotation_transformer = RotationTransformer('axis_angle', 'rotation_6d')
        print("arrives 1")
        # Factory function for creating environments with rendering
        def env_fn():
            print("arrives here and ")
            from trossen_arm_mujoco.utils import make_sim_env
            from trossen_arm_mujoco.ee_sim_env import TransferCubeEETask
            
            # Create base environment
            trossen_env = make_sim_env(
                TransferCubeEETask,
                xml_path,
                onscreen_render=False,
                cam_list=cam_list,
            )

            print("trossen env done")
            
            # Wrap in gym-compatible wrapper
            wrapped_env = TrossenImageWrapper(
                env=trossen_env,
                shape_meta=shape_meta,
                init_state=None,
                render_obs_key=render_obs_key,
            )

            print("trossen image wrapper done")
            
            # Apply standard diffusion_policy wrapper chain
            return MultiStepWrapper(
                VideoRecordingWrapper(
                    wrapped_env,
                    video_recoder=VideoRecorder.create_h264(
                        fps=fps,
                        codec='h264',
                        input_pix_fmt='rgb24',
                        crf=crf,
                        thread_type='FRAME',
                        thread_count=1
                    ),
                    file_path=None,
                    steps_per_render=steps_per_render,
                ),
                n_obs_steps=n_obs_steps,
                n_action_steps=n_action_steps,
                max_episode_steps=max_steps,
            )
        
        # Factory function for dummy env (no rendering, for space initialization)
        def dummy_env_fn():
            from trossen_arm_mujoco.utils import make_sim_env
            from trossen_arm_mujoco.ee_sim_env import TransferCubeEETask
            
            trossen_env = make_sim_env(
                TransferCubeEETask,
                xml_path,
                onscreen_render=False,
                cam_list=cam_list,
            )

            print("trossen env done")
            
            wrapped_env = TrossenImageWrapper(
                env=trossen_env,
                shape_meta=shape_meta,
                init_state=None,
                render_obs_key=render_obs_key,
            )

            print("trossen image wrapper done")
            
            return MultiStepWrapper(
                VideoRecordingWrapper(
                    wrapped_env,
                    video_recoder=VideoRecorder.create_h264(
                        fps=fps,
                        codec='h264',
                        input_pix_fmt='rgb24',
                        crf=crf,
                        thread_type='FRAME',
                        thread_count=1
                    ),
                    file_path=None,
                    steps_per_render=steps_per_render,
                ),
                n_obs_steps=n_obs_steps,
                n_action_steps=n_action_steps,
                max_episode_steps=max_steps,
            )
        
        # Create parallel environments
        print("arrives 2")
        env_fns = [env_fn] * n_envs
        print("arrives 3")
        env_seeds = []
        env_prefixs = []
        env_init_fn_dills = []
        
        # Training conditions (deterministic resets with seeds)
        for i in range(n_train):
            train_seed = i
            enable_render = i < n_train_vis
            print("i : ", i,  "n_train_vis : ", n_train_vis)
            
            def init_fn(env, seed=train_seed, enable_render=enable_render):
                # Setup video recording
                assert isinstance(env.env, VideoRecordingWrapper)
                env.env.video_recoder.stop()
                env.env.file_path = None
                
                if enable_render:
                    filename = pathlib.Path(output_dir).joinpath(
                        'media', wv.util.generate_id() + ".mp4"
                    )
                    filename.parent.mkdir(parents=True, exist_ok=True)
                    env.env.file_path = str(filename)
                
                # Set seed for deterministic reset
                assert isinstance(env.env.env, TrossenImageWrapper)
                env.env.env.seed(seed)
            
            env_seeds.append(train_seed)
            env_prefixs.append('train/')
            env_init_fn_dills.append(dill.dumps(init_fn))
        print("end of the loop 1 ")
        # Test conditions (seed-based deterministic evaluation)
        for i in range(n_test):
            print("i, 2nd loop : ", i)
            test_seed = test_start_seed + i
            enable_render = i < n_test_vis
            
            def init_fn(env, seed=test_seed, enable_render=enable_render):
                # Setup video recording
                assert isinstance(env.env, VideoRecordingWrapper)
                env.env.video_recoder.stop()
                env.env.file_path = None
                
                if enable_render:
                    filename = pathlib.Path(output_dir).joinpath(
                        'media', wv.util.generate_id() + ".mp4"
                    )
                    filename.parent.mkdir(parents=True, exist_ok=True)
                    env.env.file_path = str(filename)
                
                # Set seed for deterministic reset
                assert isinstance(env.env.env, TrossenImageWrapper)
                env.env.env.seed(seed)
            
            env_seeds.append(test_seed)
            env_prefixs.append('test/')
            env_init_fn_dills.append(dill.dumps(init_fn))

        print("end loop 2 ")

        # Create vector environment with explicit spaces
        env = SyncVectorEnv(env_fns)

        print("crashed before : ")
        
        # Store for later use
        self.env = env
        self.env_fns = env_fns
        self.env_seeds = env_seeds
        self.env_prefixs = env_prefixs
        self.env_init_fn_dills = env_init_fn_dills
        self.fps = fps
        self.crf = crf
        self.n_obs_steps = n_obs_steps
        self.n_action_steps = n_action_steps
        self.past_action = past_action
        self.max_steps = max_steps
        self.rotation_transformer = rotation_transformer
        self.abs_action = abs_action
        self.tqdm_interval_sec = tqdm_interval_sec

        print("end of the init")
    
    def run(self, policy: BaseImagePolicy) -> dict:
        """
        Execute evaluation rollouts with the given policy.
        
        Runs evaluation on all training and test conditions in parallel chunks,
        collects videos and rewards, and logs results to wandb.
        
        Args:
            policy: The policy to evaluate
            
        Returns:
            log_data: Dict of logged metrics and videos
        """
        device = policy.device
        dtype = policy.dtype
        env = self.env
        
        # Plan rollouts in chunks to match number of parallel envs
        n_envs = len(self.env_fns)
        n_inits = len(self.env_init_fn_dills)
        n_chunks = math.ceil(n_inits / n_envs)
        
        # Allocate storage for results
        all_video_paths = [None] * n_inits
        all_rewards = [None] * n_inits
        
        for chunk_idx in range(n_chunks):
            start = chunk_idx * n_envs
            end = min(n_inits, start + n_envs)
            this_global_slice = slice(start, end)
            this_n_active_envs = end - start
            this_local_slice = slice(0,this_n_active_envs)
            
            this_init_fns = self.env_init_fn_dills[this_global_slice]
            n_diff = n_envs - len(this_init_fns)
            if n_diff > 0:
                this_init_fns.extend([self.env_init_fn_dills[0]]*n_diff)
            assert len(this_init_fns) == n_envs
            
            # Initialize environments for this chunk
            env.call_each(
                'run_dill_function',
                args_list=[(fn,) for fn in this_init_fns]
            )

            print("before reset done")
            
            # Start rollout
            obs = env.reset()
            print("env reset done")
            past_action = None
            policy.reset()
            
            env_name = self.env_meta['env_name']
            pbar = tqdm.tqdm(
                total=self.max_steps,
                desc=f"Eval {env_name} {chunk_idx+1}/{n_chunks}",
                leave=False,
                mininterval=self.tqdm_interval_sec,
            )
            
            done = False
            while not done:
                print("stepping into the env")
                # Prepare observations for policy
                np_obs_dict = dict(obs)
                if self.past_action and (past_action is not None):
                    np_obs_dict['past_action'] = past_action[
                        :, -(self.n_obs_steps-1):
                    ].astype(np.float32)
                
                # Transfer to policy device
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
                if not np.all(np.isfinite(action)):
                    print(action)
                    raise RuntimeError("Nan or Inf action detected")
                
                # Apply action transformation if needed
                env_action = action
                if self.abs_action:
                    env_action = self.undo_transform_action(action)
                
                # Step environment
                obs, reward, done, info = env.step(env_action)
                done = np.all(done)
                past_action = action
                
                # Update progress bar
                pbar.update(action.shape[1])
            
            pbar.close()
            
            # Collect results for this chunk
            all_video_paths[this_global_slice] = env.render()[this_local_slice]
            all_rewards[this_global_slice] = env.call(
                'get_attr', 'reward'
            )[this_local_slice]
        
        # Reset environment to clear video buffers
        _ = env.reset()
        
        # Aggregate and log results
        max_rewards = collections.defaultdict(list)
        log_data = dict()
        
        for i in range(n_inits):
            seed = self.env_seeds[i]
            prefix = self.env_prefixs[i]
            max_reward = np.max(all_rewards[i])
            
            max_rewards[prefix].append(max_reward)
            log_data[prefix + f'sim_max_reward_{seed}'] = max_reward
            
            # Log video if available
            video_path = all_video_paths[i]
            if video_path is not None:
                sim_video = wandb.Video(video_path)
                log_data[prefix + f'sim_video_{seed}'] = sim_video
        
        # Log aggregate metrics
        for prefix, rewards in max_rewards.items():
            mean_reward = np.mean(rewards)
            log_data[prefix + 'mean_score'] = mean_reward
        
        return log_data
    
    def undo_transform_action(self, action: np.ndarray) -> np.ndarray:
        """
        Transform action from representation used by policy to environment action.
        
        Handles rotation representation conversion (6D rotation → axis-angle)
        and dual-arm action reshaping.
        
        Args:
            action: Policy output action
            
        Returns:
            Transformed action for environment
        """
        raw_shape = action.shape
        if raw_shape[-1] == 20:
            # Dual arm with different representation
            action = action.reshape(-1, 2, 10)
        
        d_rot = action.shape[-1] - 4
        pos = action[..., :3]
        rot = action[..., 3:3+d_rot]
        gripper = action[..., [-1]]
        
        # Transform rotation representation
        rot = self.rotation_transformer.inverse(rot)
        
        uaction = np.concatenate([pos, rot, gripper], axis=-1)
        
        if raw_shape[-1] == 20:
            # Reshape back to dual arm
            uaction = uaction.reshape(*raw_shape[:-1], 14)
        
        return uaction
