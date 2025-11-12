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


# Camera configurations for different tasks
GOOD_CAMERAS = {
    "TransferCubeEETask": ["cam_high", "cam_low", "cam_left_wrist", "cam_right_wrist"],
}

DEFAULT_CAMERA = "cam_high"


# State keys for observation
DEFAULT_STATE_KEYS = ["qpos", "qvel", "env_state", "mocap_pose_left", "mocap_pose_right", "gripper_ctrl"]
STATE_KEYS = {
    "TransferCubeEETask": DEFAULT_STATE_KEYS,
}

# State shape: qpos(16) + qvel(16) + env_state(7) + mocap_left(7) + mocap_right(7) + gripper_ctrl(2) = 55
STATE_SHAPE = {
    "TransferCubeEETask": (55,),
}

# Proprioceptive keys: mocap poses + gripper control
PROP_KEYS = ["mocap_pose_left", "mocap_pose_right", "gripper_ctrl"]
PROP_DIM = 16  # mocap_left(7) + mocap_right(7) + gripper_ctrl(2) = 16


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
        flip_image=True,
        ctrl_delta=True,
        record_sim_state: bool = False,
        onscreen_render: bool = True,
    ):
        self._plt_fig = None
        self._plt_imgs = None

        print("the camera is : ", camera_names)
        if camera_names is None:
            camera_names = [DEFAULT_CAMERA]
        if rl_cameras is None:
            rl_cameras = ["cam_high"]
            
        assert isinstance(camera_names, list)
        self.camera_names = camera_names
        self.ctrl_delta = ctrl_delta
        self.record_sim_state = record_sim_state
        self.onscreen_render = onscreen_render
        
        # Map environment names to task classes
        task_map = {
            "TransferCubeEETask": TransferCubeEETask,
        }
        
        if env_name not in task_map:
            raise ValueError(f"Unknown environment: {env_name}")
        
        # Create the Trossen environment
        onscreen_render = True
        cam_list = ["cam_high", "cam_low", "cam_left_wrist", "cam_right_wrist"]
        self.env = make_sim_env(
            TransferCubeEETask,
            task_name="sim_transfer_cube",
            onscreen_render=onscreen_render,
            cam_list=cam_list,
            max_steps=episode_length,
        )
        
        self.rl_cameras = rl_cameras if isinstance(rl_cameras, list) else [rl_cameras]
        self.image_size = image_size
        self.rl_image_size = rl_image_size or image_size
        self.env_reward_scale = env_reward_scale
        self.end_on_success = end_on_success
        self.use_state = use_state
        print("the env name is : ", env_name)
        self.state_keys = STATE_KEYS[env_name]
        self.prop_keys = PROP_KEYS
        self.flip_image = flip_image
        self.episode_length = episode_length

        self.resize_transform = None
        if self.rl_image_size != self.image_size:
            self.resize_transform = utils.get_rescale_transform((self.rl_image_size, self.rl_image_size))

        # Action dimension: 23 (pos(3) + quat(4) + gripper(1)) * 2 arms + extra dim
        # Based on ee_sim_env.py test, action is 23-dimensional
        self.action_dim: int = 23
        self._observation_shape: tuple[int, ...] = (3 * obs_stack, rl_image_size, rl_image_size)
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
        for key in self.prop_keys:
            if key in obs:
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
        obs = ts.observation

        obs_display = obs.copy()


        if 'images' in obs:
            for cam_name, cam_data in obs['images'].items():
                obs[cam_name] = cam_data
            del obs['images']

        
        rl_obs, high_res_images = self._extract_images(obs)

        # Rendering : 
        if self.onscreen_render and self._plt_fig is None:
            self._plt_fig = plt.figure()
            self._plt_imgs = plot_observation_images(obs_display, self.camera_names)

        if self.cond_action > 0:
            past_action = torch.from_numpy(np.stack(self.past_actions)).to(self.device)
            rl_obs["past_action"] = past_action

        return rl_obs, high_res_images

    def step(self, actions: torch.Tensor) -> tuple[dict, float, bool, bool, dict]:

        """
        Step the environment with given actions.
        All inputs and outputs are tensors.

        Structure of the returned obs : 
        obs : OrderedDict([
        ('images', {...}),
        ('qpos', array(...)),
        ('qvel', array(...)),
        ('env_state', array(...)),
        ('mocap_pose_left', array(...)),
        ('mocap_pose_right', array(...)),
        ('gripper_ctrl', array(...))
        ])

        """
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
            
            # Clip each action dimension to [-0.1, 0.1]
            clipped_action = np.clip(actions[i], -0.1, 0.1) # clip action is done. 
            ts = self.env.step(clipped_action)

            obs = ts.observation
            obs_display = obs.copy()

            if 'images' in obs:
                for cam_name, cam_data in obs['images'].items():
                    obs[cam_name] = cam_data
                del obs['images']

            #print("obs are : ", obs)

            # Rendering : 
            if self.onscreen_render and self._plt_imgs is not None:
                self._plt_imgs = set_observation_images(obs_display, self._plt_imgs, self.camera_names)
                
            step_reward = ts.reward if ts.reward is not None else 0
            terminal = ts.last() # common in dm control. 
            # NOTE: extract images every step for potential obs stacking
            # this is not efficient
            curr_rl_obs, curr_high_res_images = self._extract_images(obs)

            if i == num_action - 1:
                rl_obs.update(curr_rl_obs)
                high_res_images.update(curr_high_res_images)

            reward += step_reward
            self.episode_reward += step_reward

            # Check for success (max reward indicates successful transfer)
            if step_reward == self.env.task.max_reward:
                success = True
                if self.end_on_success:
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