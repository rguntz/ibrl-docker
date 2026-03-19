# Copyright Amazon.com, Inc. or its affiliates. All Rights Reserved.
# SPDX-License-Identifier: CC-BY-NC-4.0

"""
Environment wrapper that includes a base policy, enabling residual RL training
to be done in a standard way without explicit base policy handling in the training loop.

This wrapper assumes:
- The environment is already vectorized (batched)
- All inputs/outputs are torch tensors
- The environment comes from create_vectorized_env
"""

import gymnasium as gym
import numpy as np
import torch

from resfit.dexmg.environments.dexmg import VectorizedEnvWrapper
from resfit.lerobot.policies.act.modeling_act import ACTPolicy


class BasePolicyVecEnvWrapper:
    """
    Wraps a vectorized environment with a base policy to enable standard RL training of residual policies.

    This wrapper:
    1. Takes raw observations from the vectorized environment
    2. Passes them through the base policy to get base actions
    3. Augments observations with base actions for the residual policy
    4. Computes the DIPO action-gradient improved action from the current obs
    5. Returns augmented observations that include base actions

    Assumes the environment is already vectorized and works with torch tensors.
    """

    def __init__(
        self,
        vec_env: VectorizedEnvWrapper,
        base_policy: ACTPolicy,
        action_scaler,
        state_standardizer,
    ):
        """
        Args:
            vec_env: Vectorized environment from create_vectorized_env
            base_policy: Base policy (e.g., ACTPolicy) to augment with residual actions
            action_scaler: ActionScaler object for scaling/unscaling actions (REQUIRED)
            state_standardizer: StateStandardizer object for standardizing states (REQUIRED)
        """
        assert action_scaler is not None, "action_scaler is required for consistent normalization"
        assert state_standardizer is not None, "state_standardizer is required for consistent normalization"

        self.vec_env = vec_env
        self.base_policy = base_policy
        self.action_scaler = action_scaler
        self.state_standardizer = state_standardizer

        # Get action dimension from the environment
        self.action_dim = vec_env.action_space.shape[-1]

        # Store image keys from base policy config
        self.image_keys = list(base_policy.config.image_features.keys())

        # Create modified observation space that includes base actions
        self._setup_observation_space()

    def _setup_observation_space(self):
        """Setup observation space to include base actions in the state."""

        orig_obs_space = self.vec_env.observation_space
        self.action_space = self.vec_env.action_space

        obs_spaces = {}
        for key, space in orig_obs_space.spaces.items():
            if key == "observation.state":
                orig_shape = list(space.shape)
                new_shape = orig_shape.copy()
                obs_spaces[key] = gym.spaces.Box(low=-np.inf, high=np.inf, shape=tuple(new_shape), dtype=space.dtype)
            else:
                obs_spaces[key] = space

        self.observation_space = gym.spaces.Dict(obs_spaces)

    def reset(self, **kwargs) -> tuple[dict[str, torch.Tensor], dict]:
        """Reset environment and base policy."""
        raw_obs, info = self.vec_env.reset(**kwargs)

        self.base_policy.reset()

        with torch.no_grad():
            base_action = self.base_policy.select_action(raw_obs)

        base_naction = self.action_scaler.scale(base_action)

        augmented_obs = self._augment_obs(raw_obs, base_naction)

        # Store the full augmented obs so step() can read feat + state from it
        self._last_augmented_obs = augmented_obs
        self._last_base_naction = base_naction

        return augmented_obs, info

    def step(
        self,
        agent,  # QAgent instance (DIPO critic-only)
        improvement : bool = False
    ) -> tuple[dict[str, torch.Tensor], torch.Tensor, torch.Tensor, torch.Tensor, dict]:
        """
        Step the environment using a DIPO action-gradient improved action.

        The improved action is computed as:
            a* = clip(a_buffer + η · ∇_a min_i Q_ψi(s, a), -1, 1)
        starting from the last base action, using the current observation's
        encoded features and proprioceptive state.

        Args:
            agent: QAgent (DIPO) instance, used to encode the current
                   observation and run _action_gradient_step.

        Returns:
            augmented_obs: Observations augmented with base actions
            reward: Reward tensor
            terminated: Terminated tensor
            truncated: Truncated tensor
            info: Info dict
        """
        # Encode the current observation with the agent's encoder.
        # We call _encode on the stored augmented obs (which already has
        # standardized states and the base_action key).
        # No augmentation at eval/interaction time.
        with torch.no_grad():
            feat = agent._encode(self._last_augmented_obs, augment=False)

        state = self._last_augmented_obs["observation.state"]

        # Compute the DIPO action-gradient improved action.
        # _action_gradient_step uses torch.enable_grad() internally.
        if improvement : 
            improved_action = agent._action_gradient_step(
                feat=feat,
                state=state,
                a_init=self._last_base_naction,
            )
        else : 
            improved_action = self._last_base_naction

        # Unscale back to original action space for environment execution
        env_action = self.action_scaler.unscale(improved_action)

        # Step the underlying vectorized environment
        raw_obs, reward, terminated, truncated, info = self.vec_env.step(env_action)

        # Store the scaled action for the replay buffer
        info["scaled_action"] = improved_action

        # Get next base action from the base policy
        with torch.no_grad():
            base_action = self.base_policy.select_action(raw_obs)

        base_naction = self.action_scaler.scale(base_action)

        # Handle policy reset for terminated environments
        if terminated.any():
            self.base_policy.reset()

        # Augment next observations with next base action and standardize state
        augmented_obs = self._augment_obs(raw_obs, base_naction)

        # Handle final_obs in info dict to ensure consistent shapes
        if "final_obs" in info:
            info = self._process_final_obs_in_info(info, improved_action.device)

        # Store for next step
        self._last_augmented_obs = augmented_obs
        self._last_base_naction = base_naction

        return augmented_obs, reward, terminated, truncated, info

    def _augment_obs(self, raw_obs: dict[str, torch.Tensor], base_naction: torch.Tensor) -> dict[str, torch.Tensor]:
        """Augment observations with base actions."""
        augmented_obs = raw_obs.copy()
        augmented_obs["observation.base_action"] = base_naction
        augmented_obs["observation.state"] = self.state_standardizer.standardize(augmented_obs["observation.state"])
        return augmented_obs

    def _process_final_obs_in_info(self, info: dict, device: torch.device) -> dict:
        """Pad final_obs state with zeros to match augmented observation format."""
        if "final_obs" not in info or info["final_obs"] is None:
            return info

        for final_obs_dict in info["final_obs"]:
            if final_obs_dict is not None and "observation.state" in final_obs_dict:
                final_obs_dict["observation.base_action"] = torch.zeros(
                    self.action_dim, device=device, dtype=torch.float32
                )

        return info

    def render(self):
        return self.vec_env.render()

    def close(self):
        return self.vec_env.close()

    def __getattr__(self, name: str):
        """Delegate unknown attributes to the underlying vectorized environment."""
        return getattr(self.vec_env, name)