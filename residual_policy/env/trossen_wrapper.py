import gymnasium as gym
from gymnasium import spaces
import numpy as np
import torch
from env.utils import plot_observation_images, set_observation_images
from matplotlib import pyplot as plt

class Trossen_wrapper(gym.Env):
    """
    Wrapper that matches the target observation space format.
    """

    def __init__(
        self,
        env,
        state_dim = 16,
        action_dim = 14,
        img_shape=(1, 3, 256, 256),
        horizon: int = 500,
    ):
        super().__init__()
        self.env = env

        # -------- action space --------
        self.action_space = spaces.Box(
            low=-1.0,
            high=1.0,
            shape=(1, action_dim),
            dtype=np.float32,
        )

        # -------- observation space --------
        self.observation_space = spaces.Dict({
            "observation.state": spaces.Box(
                low=-np.inf,
                high=np.inf,
                shape=(1, state_dim),
                dtype=np.float32,
            ),
            "observation.images.video": spaces.Box(
                low=0.0,
                high=1.0,
                shape=img_shape,
                dtype=np.float32,
            ),
            "observation.images.video_2": spaces.Box(
                low=0.0,
                high=1.0,
                shape=img_shape,
                dtype=np.float32,
            ),
            "observation.images.wrist_video": spaces.Box(
                low=0.0,
                high=1.0,
                shape=img_shape,
                dtype=np.float32,
            ),
            "observation.images.wrist_video_2": spaces.Box(
                low=0.0,
                high=1.0,
                shape=img_shape,
                dtype=np.float32,
            ),
        })

        # -------- metadata --------
        self.metadata = {"horizon": horizon}

        self.plot = None
        self._plt_fig = None

    
    # ------------------------------------------------
    # Dummy pass-throughs (intentionally minimal)
    # ------------------------------------------------
    def reset(self, *args, **kwargs):
        timestep = self.env.reset(*args, **kwargs)

        obs = timestep.observation

        if self._plt_fig is None: 
            self._plt_fig = plt.figure()
            self._plt_imgs = plot_observation_images(obs, ["cam_high", "cam_low", "cam_left_wrist", "cam_right_wrist"])

        # HARDCODE DEVICE
        device = torch.device("cuda")

        state = np.zeros(16, dtype=np.float32)

        left_pos = obs["robot0_eef_pos"][:3]
        right_pos = obs["robot0_eef_pos"][3:]
        left_quat = obs["robot0_eef_quat"][:4]
        right_quat = obs["robot0_eef_quat"][4:]
        left_grip = obs["robot0_gripper_qpos"][0]
        right_grip = obs["robot0_gripper_qpos"][1]

        state[0:3] = left_pos
        state[3:7] = left_quat
        state[7] = left_grip
        state[8:11] = right_pos
        state[11:15] = right_quat
        state[15] = right_grip

        images = obs["images"]


        raw_obs = {
            "observation.state": torch.tensor(
                state, dtype=torch.float32, device=device
            ).unsqueeze(0),  # (1, 16)

            # Normalize images to [0,1] before converting to tensors
            "observation.images.wrist_video_2": torch.tensor(
                images["cam_left_wrist"].astype(np.float32) / 255.0, dtype=torch.float32, device=device
            ).permute(2, 0, 1).unsqueeze(0),  # (1, C, H, W)

            "observation.images.wrist_video": torch.tensor(
                images["cam_right_wrist"].astype(np.float32) / 255.0, dtype=torch.float32, device=device
            ).permute(2, 0, 1).unsqueeze(0),

            "observation.images.video": torch.tensor(
                images["cam_high"].astype(np.float32) / 255.0, dtype=torch.float32, device=device
            ).permute(2, 0, 1).unsqueeze(0),

            "observation.images.video_2": torch.tensor(
                images["cam_low"].astype(np.float32) / 255.0, dtype=torch.float32, device=device
            ).permute(2, 0, 1).unsqueeze(0),
        }

        return raw_obs, {}

    def step(self, action):
        action = action[0]

        action_np = action.detach().cpu().numpy()

        timestep = self.env.step(action_np)
        obs = timestep.observation

        if self._plt_imgs is not None:
            self._plt_imgs = set_observation_images(obs, self._plt_imgs, ["cam_high", "cam_low", "cam_left_wrist", "cam_right_wrist"]) 

        # HARDCODE DEVICE to cuda
        device = torch.device("cuda")

        state = np.zeros(16, dtype=np.float32)
        # populate state as before
        left_pos = obs["robot0_eef_pos"][:3]
        right_pos = obs["robot0_eef_pos"][3:]
        left_quat = obs["robot0_eef_quat"][:4]
        right_quat = obs["robot0_eef_quat"][4:]
        left_grip = obs["robot0_gripper_qpos"][0]
        right_grip = obs["robot0_gripper_qpos"][1]

        state[0:3] = left_pos
        state[3:7] = left_quat
        state[7] = left_grip
        state[8:11] = right_pos
        state[11:15] = right_quat
        state[15] = right_grip

        images = obs["images"]

        raw_obs = {
            "observation.state": torch.tensor(
                state, dtype=torch.float32, device=device
            ).unsqueeze(0),  # (1, 16)

            # Normalize images to [0,1] before converting to tensors
            "observation.images.wrist_video_2": torch.tensor(
                images["cam_left_wrist"].astype(np.float32) / 255.0, dtype=torch.float32, device=device
            ).permute(2, 0, 1).unsqueeze(0),  # (1, C, H, W)

            "observation.images.wrist_video": torch.tensor(
                images["cam_right_wrist"].astype(np.float32) / 255.0, dtype=torch.float32, device=device
            ).permute(2, 0, 1).unsqueeze(0),

            "observation.images.video": torch.tensor(
                images["cam_high"].astype(np.float32) / 255.0, dtype=torch.float32, device=device
            ).permute(2, 0, 1).unsqueeze(0),

            "observation.images.video_2": torch.tensor(
                images["cam_low"].astype(np.float32) / 255.0, dtype=torch.float32, device=device
            ).permute(2, 0, 1).unsqueeze(0),
        }

        # Convert reward, terminated, truncated to tensors on CUDA with shape (1,)
        reward = torch.tensor([timestep.reward], dtype=torch.float32, device=device)
        terminated = torch.tensor([timestep.terminated], dtype=torch.bool, device=device)
        truncated = torch.tensor([timestep.truncated], dtype=torch.bool, device=device)

        return raw_obs, reward, terminated, truncated, {}

    def close(self):
        if hasattr(self.env, "close"):
            self.env.close()