"""
Multiprocess runner for Trossen AI TransferCubeEETask.

Manages parallel environment rollouts with reward collection across multiple processes.
"""

import os
import numpy as np
import torch
import torch.multiprocessing as mp
import collections
import pathlib
import time
from collections import defaultdict

# Only set start method if not already set
try:
    if mp.get_start_method(allow_none=True) is None:
        mp.set_start_method("spawn")
except RuntimeError:
    # Start method already set, continue
    pass

from diffusion_policy.model.common.rotation_transformer import RotationTransformer
from diffusion_policy.policy.base_image_policy import BaseImagePolicy
from diffusion_policy.common.pytorch_util import dict_apply
from diffusion_policy.env_runner.base_image_runner import BaseImageRunner
from scipy.spatial.transform import Rotation as R

# Gripper normalization constants (matching your dataset processing)
GRIPPER_MIN = 0.0
GRIPPER_MAX = 0.04


class EvalProcess:
    """Single evaluation process that runs episodes for assigned seeds."""
    
    def __init__(
        self,
        seeds: list,
        process_id: int,
        env_config: dict,
        terminal_queue: mp.Queue,
    ):
        self.seeds = seeds
        self.process_id = process_id
        self.env_config = env_config
        self.terminal_queue = terminal_queue
        self.send_queue = mp.Queue()  # Send obs to main process
        self.recv_queue = mp.Queue()  # Receive actions from main process
        
    def create_env(self):
        # Import ONLY here — inside the subprocess
        from trossen_arm_mujoco.utils import make_sim_env
        from trossen_arm_mujoco.ee_sim_env import TransferCubeEETask
        from diffusion_policy.env.trossen.trossen_image_wrapper import TrossenImageWrapper
        from diffusion_policy.gym_util.multistep_wrapper import MultiStepWrapper

        # Now create env
        trossen_env = make_sim_env(
            TransferCubeEETask,
            self.env_config['xml_path'],
            onscreen_render=False,
            cam_list=self.env_config['cam_list'],
        )
        
        wrapped_env = TrossenImageWrapper(
            env=trossen_env,
            shape_meta=self.env_config['shape_meta'],
            init_state=None,
            render_obs_key=self.env_config['render_obs_key'],
        )
        
        env = MultiStepWrapper(
            wrapped_env,
            n_obs_steps=self.env_config['n_obs_steps'],
            n_action_steps=self.env_config['n_action_steps'],
            max_episode_steps=self.env_config['max_steps'],
        )
        return env
    
    def start(self):
        """Main process loop - runs episodes for all assigned seeds."""
        env = self.create_env()
        
        results = {}
        
        for seed in self.seeds:
            # Seed environment
            env.env.seed(seed)
            
            # Reset environment
            obs = env.reset()
            
            # Run episode
            done = False
            max_reward = 0.0
            
            while not done:
                # Send observation to main process (add batch dimension)
                obs_to_send = {k: v[None, ...] for k, v in obs.items()}
                self.send_queue.put((self.process_id, obs_to_send))
                
                # Receive action from main process
                action = self.recv_queue.get()
                
                # Step environment
                obs, reward, done, info = env.step(action)
            
            # Get final rewards
            rewards = env.get_attr('reward')
            max_reward = np.max(rewards)
            results[seed] = float(max_reward)
        
        # Send results back to main process
        self.terminal_queue.put((self.process_id, results))
        return


class MultiprocessTrossenImageRunner(BaseImageRunner):
    """
    Multiprocess runner for Trossen AI bimanual manipulation tasks.
    
    Runs evaluations in parallel across multiple processes for faster evaluation.
    
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
        num_proc: Number of parallel processes to use
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
        tqdm_interval_sec: float = 1.0,  # Added for compatibility
        cam_list: list = None,
        xml_path: str = "trossen_ai_scene.xml",
        num_proc: int = 10,
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
        self.num_proc = num_proc
        
        # Setup rotation transformer if needed
        self.rotation_transformer = None
        if abs_action:
            self.rotation_transformer = RotationTransformer('axis_angle', 'rotation_6d')
        
        # Setup evaluation seeds
        assert n_test % num_proc == 0, f"n_test ({n_test}) must be divisible by num_proc ({num_proc})"
        self.test_seeds = list(range(test_start_seed, test_start_seed + n_test))
        
        print(f"Initialized multiprocess runner with {n_test} test conditions across {num_proc} processes")
    
    def get_env_config(self) -> dict:
        """Get environment configuration dict for subprocess initialization."""
        return {
            'xml_path': self.xml_path,
            'cam_list': self.cam_list,
            'shape_meta': self.shape_meta,
            'render_obs_key': self.render_obs_key,
            'fps': self.fps,
            'crf': self.crf,
            'steps_per_render': self.steps_per_render,
            'n_obs_steps': self.n_obs_steps,
            'n_action_steps': self.n_action_steps,
            'max_steps': self.max_steps,
        }
    
    def run(self, policy: BaseImagePolicy) -> dict:
        """
        Execute evaluation rollouts with the given policy in parallel.
        
        Args:
            policy: The policy to evaluate
            
        Returns:
            log_data: Dict of logged metrics
        """
        device = policy.device
        
        # Distribute seeds across processes
        seeds_per_proc = len(self.test_seeds) // self.num_proc
        terminal_queue = mp.Queue()
        
        eval_procs = []
        for i in range(self.num_proc):
            proc_seeds = self.test_seeds[i * seeds_per_proc : (i + 1) * seeds_per_proc]
            eval_procs.append(
                EvalProcess(
                    seeds=proc_seeds,
                    process_id=i,
                    env_config=self.get_env_config(),
                    terminal_queue=terminal_queue,
                )
            )
        
        # Setup communication queues
        put_queues = {i: proc.recv_queue for i, proc in enumerate(eval_procs)}
        get_queues = {i: proc.send_queue for i, proc in enumerate(eval_procs)}
        
        # Start all processes
        processes = {i: mp.Process(target=proc.start) for i, proc in enumerate(eval_procs)}
        for _, p in processes.items():
            p.start()
        
        print(f"Started {self.num_proc} evaluation processes")
        
        # Storage for past actions (per process)
        past_actions = {i: None for i in range(self.num_proc)}
        
        t = time.time()
        results = {}
        
        # Main policy inference loop
        with torch.no_grad():
            policy.eval()
            
            while len(processes) > 0:
                # Check for completed processes
                while not terminal_queue.empty():
                    term_idx, proc_results = terminal_queue.get()
                    results.update(proc_results)
                    processes[term_idx].join()
                    processes.pop(term_idx)
                    get_queues.pop(term_idx)
                    put_queues.pop(term_idx)
                    past_actions.pop(term_idx)
                
                # Collect observations from all active processes
                obses = defaultdict(list)
                idxs = []
                proc_past_actions = []
                
                for proc_id, get_queue in get_queues.items():
                    if get_queue.empty():
                        continue
                    
                    data = get_queue.get()
                    proc_id_recv = data[0]
                    obs_dict = data[1]
                    
                    idxs.append(proc_id_recv)
                    proc_past_actions.append(past_actions[proc_id_recv])
                    
                    # Stack observations
                    for k, v in obs_dict.items():
                        obses[k].append(v)
                
                if len(obses) == 0:
                    continue
                
                # Batch observations
                batch_obs = {}
                for k, v_list in obses.items():
                    # v_list is list of (1, T, ...) -> concat to (B, T, ...)
                    batch_obs[k] = torch.from_numpy(
                        np.concatenate(v_list, axis=0)
                    ).to(device=device)
                
                # Get batch actions from policy
                action_dict = policy.predict_action(batch_obs)

                np_action_dict = dict_apply(
                    action_dict,
                    lambda x: x.detach().to('cpu').numpy(),
                )
                batch_actions = np_action_dict['action']
                
                # Check for invalid actions
                if not np.all(np.isfinite(batch_actions)):
                    raise RuntimeError("Nan or Inf action detected")
                
                # Transform actions if needed
                if self.abs_action:
                    batch_actions = self.undo_transform_action(batch_actions)
                
                # Send actions back to processes
                for idx, action in zip(idxs, batch_actions):
                    # Extract single action from batch (remove batch dim)
                    single_action = action  # Shape: (T, action_dim)
                    put_queues[idx].put(single_action)
                    past_actions[idx] = action  # Store for next iteration
        
        print(f"Total evaluation time: {time.time() - t:.2f}s")
        
        # Compute metrics
        all_rewards = []
        log_data = {}
        
        for seed in sorted(results.keys()):
            max_reward = results[seed]
            all_rewards.append(max_reward)
            log_data[f'test/sim_max_reward_{seed}'] = max_reward
            print(f"Seed {seed}: reward = {max_reward:.3f}")
        
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
        
        Gripper indices in 16D action: [6, 15] ???????
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
        
