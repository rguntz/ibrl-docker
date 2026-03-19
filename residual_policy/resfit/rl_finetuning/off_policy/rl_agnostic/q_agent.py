# Copyright Amazon.com, Inc. or its affiliates. All Rights Reserved.
# SPDX-License-Identifier: CC-BY-NC-4.0

from __future__ import annotations

import copy

import torch
from torch import nn

from resfit.rl_finetuning.config.rlpd import QAgentConfig
from resfit.rl_finetuning.off_policy import common_utils
from resfit.rl_finetuning.off_policy.common_utils import utils
from resfit.rl_finetuning.off_policy.networks.encoder import VitEncoder
from resfit.rl_finetuning.off_policy.rl.critic import Critic


class QAgent(nn.Module):
    def __init__(
        self,
        obs_shape: tuple[int, int, int],
        prop_shape: tuple[int],
        action_dim: int,
        rl_cameras: list[str] | str,
        cfg: QAgentConfig,
    ):
        super().__init__()

        if isinstance(rl_cameras, str):
            rl_cameras = [rl_cameras]
        assert len(rl_cameras) > 0, "At least one camera must be provided"

        self.rl_cameras = rl_cameras
        self.cfg = cfg

        self.encoders: nn.ModuleList = self._build_encoders(obs_shape)

        sample_encoder = self.encoders[0]
        repr_dim_single = int(sample_encoder.repr_dim)  # type: ignore[attr-defined]
        patch_repr_dim = int(sample_encoder.patch_repr_dim)  # type: ignore[attr-defined]

        repr_dim = repr_dim_single * len(self.rl_cameras)
        print("encoder output dim: ", repr_dim)
        print("patch output dim: ", patch_repr_dim)

        assert len(prop_shape) == 1
        prop_dim = prop_shape[0] if cfg.use_prop else 0

        self.critic = Critic(
            repr_dim=repr_dim,
            patch_repr_dim=patch_repr_dim,
            prop_dim=prop_dim,
            action_dim=action_dim,
            cfg=self.cfg.critic,
        )
        self.critic_target = copy.deepcopy(self.critic)

        print(common_utils.wrap_ruler("encoder weights"))
        print(self.encoders)
        common_utils.count_parameters(self.encoders)

        print(common_utils.wrap_ruler("critic weights"))
        print(self.critic)
        common_utils.count_parameters(self.critic)

        # Freeze encoder parameters if requested
        if getattr(self.cfg, "freeze_encoder", False):
            for param in self.encoders.parameters():
                param.requires_grad = False
            print("🧊 Encoder parameters frozen - no gradient updates will be performed")

        self.encoder_opt = torch.optim.AdamW(self.encoders.parameters(), lr=self.cfg.critic_lr)
        self.critic_opt = torch.optim.AdamW(self.critic.parameters(), lr=self.cfg.critic_lr)

        # LR schedulers for warmup (if warmup is enabled)
        self.encoder_scheduler = None
        self.critic_scheduler = None

        if self.cfg.lr_warmup_steps > 0:
            warmup_start = self.cfg.lr_warmup_start
            critic_start_factor = warmup_start / self.cfg.critic_lr if self.cfg.critic_lr > 0 else 1e-8
            critic_start_factor = max(critic_start_factor, 1e-8)

            self.encoder_scheduler = torch.optim.lr_scheduler.LinearLR(
                self.encoder_opt, start_factor=critic_start_factor, total_iters=self.cfg.lr_warmup_steps
            )
            self.critic_scheduler = torch.optim.lr_scheduler.LinearLR(
                self.critic_opt, start_factor=critic_start_factor, total_iters=self.cfg.lr_warmup_steps
            )

        # data augmentation
        self.aug = common_utils.RandomShiftsAug(pad=4)

        # DIPO action-gradient hyper-parameters
        self.action_gradient_step_size: float = 0.1
        self.action_gradient_steps: int = 1

        self.critic_target.train(False)
        self.train(True)
        self.to(self.cfg.device)

    def _build_encoders(self, obs_shape):
        encoders = nn.ModuleList()
        for _ in self.rl_cameras:
            if self.cfg.enc_type == "vit":
                enc = VitEncoder(obs_shape, self.cfg.vit).to(self.cfg.device)
            else:
                raise AssertionError(f"Unknown encoder type {self.cfg.enc_type}.")
            encoders.append(enc)
        return encoders

    def train(self, training=True):
        self.training = training
        self.encoders.train(training)
        self.critic.train(training)
        assert not self.critic_target.training

    def _encode(self, obs: dict[str, torch.Tensor], augment: bool) -> torch.Tensor:
        r"""Encode camera observations into a single feature tensor.

        Images may be stored in the replay buffer as uint8 to save GPU memory.
        In that case we convert them to float32 in [0, 1] before encoding.
        """
        feats = []
        for cam_idx, cam_name in enumerate(self.rl_cameras):
            data = obs[cam_name]
            if data.dtype == torch.uint8:
                data = data.float().div_(255.0)
            else:
                data = data.float()

            if augment:
                data = self.aug(data)

            feat_cam = self.encoders[cam_idx].forward(data, flatten=False)
            feats.append(feat_cam)

        feat_all = torch.cat(feats, dim=1)
        return feat_all  # noqa: RET504

    def _action_gradient_step(
        self,
        feat: torch.Tensor,
        state: torch.Tensor,
        a_init: torch.Tensor,
    ) -> torch.Tensor:
        """Improve *a_init* via gradient ascent on ``min_i Q_ψi_target(s, a)``.

        DIPO action-gradient update rule:
            a ← clip(a + η · ∇_a min_i Q_ψi_target(s, a), -1, 1)

        Uses ``torch.enable_grad()`` internally so that it is safe to call
        from within a ``torch.no_grad()`` context (as in update_critic).

        Parameters
        ----------
        feat   : [B, P, D]  pre-computed encoder features (detached).
        state  : [B, S]     proprioceptive state.
        a_init : [B, A]     starting action (base action from replay buffer).

        Returns
        -------
        a_improved : [B, A]  action after gradient steps, clamped to [-1, 1].
        """
        # FIX (Bug 2 & 5): torch.enable_grad() is required here because this
        # method is called inside torch.no_grad() in update_critic.
        # Without it, autograd.grad() raises a RuntimeError (no graph exists).
        # Additionally, requires_grad_(True) inside no_grad() does not
        # actually enable graph-building for subsequent ops, so the re-attach
        # at the end of each iteration must also live inside enable_grad().
        with torch.enable_grad():
            a = a_init.detach().clone().requires_grad_(True)
            eta = self.action_gradient_step_size

            for _ in range(self.action_gradient_steps):
                # Forward through the frozen target critic
                q_min = self.critic_target.q_value(feat, state, a).squeeze(-1)  # [B]

                # ∂Q/∂a
                grad = torch.autograd.grad(q_min.sum(), a)[0]  # [B, A]

                # Gradient-ascent step; detach before re-attaching so the
                # computation graph does not accumulate across steps.
                a = torch.clamp(a.detach() + eta * grad.detach(), -1.0, 1.0)
                a = a.requires_grad_(True)  # re-attach inside enable_grad()

        return a.detach()

    def update_critic(
        self,
        obs: dict[str, torch.Tensor],
        action: torch.Tensor,
        reward: torch.Tensor,
        discount: torch.Tensor,
        next_obs: dict[str, torch.Tensor],
        # FIX (Bug 3): `stddev` removed — it was only used for the actor,
        # which no longer exists in this class.
        importance_weights: torch.Tensor | None = None,
    ):
        with torch.no_grad():
            # FIX (Bug 1): removed `assert self.actor_target.training` —
            # actor_target does not exist in this class; the assert was
            # leftover from the original QAgent and would always crash.

            # DIPO: compute next action via action gradient on the target critic.
            # _action_gradient_step uses torch.enable_grad() internally, so
            # calling it inside torch.no_grad() here is safe.
            a_improved = self._action_gradient_step(
                feat=next_obs["feat"],
                state=next_obs["observation.state"],
                a_init=next_obs["observation.base_action"],
            )
            # a_improved is already clamped to [-1, 1] inside _action_gradient_step
            next_action = a_improved

            # Compute target Q using min over ensemble heads
            target_all = self.critic_target.q_value(next_obs["feat"], next_obs["observation.state"], next_action)
            target_q_min = target_all.squeeze(-1)  # [B]
            target_q = (reward + (discount * target_q_min)).detach()

        if self.cfg.clip_q_target_to_reward_range:
            target_q = torch.clamp(target_q, min=0, max=1)  # Sparse rewards are in {0, 1}

        td_errors = None

        if self.critic.loss_cfg.type == "hl_gauss":
            q_per_head, logits_per_head = self.critic(obs["feat"], obs["observation.state"], action, return_logits=True)
            K = logits_per_head.shape[0]
            losses = [self.critic.hl_loss(logits_per_head[i], target_q) for i in range(K)]
            critic_loss = torch.stack(losses).mean()
        elif self.critic.loss_cfg.type == "c51":
            q_per_head, logits_per_head = self.critic(obs["feat"], obs["observation.state"], action, return_logits=True)

            with torch.no_grad():
                _, next_logits = self.critic_target(
                    next_obs["feat"], next_obs["observation.state"], next_action, return_logits=True
                )
                num_heads = min(self.critic.cfg.min_q_heads, next_logits.shape[0])
                idx = torch.randperm(next_logits.shape[0], device=next_logits.device)[:num_heads]
                next_logits_min = torch.min(next_logits.index_select(0, idx), dim=0).values
                next_distribution = torch.softmax(next_logits_min, dim=-1)

                dones = (discount == 0.0).float()
                gamma = 0.99
                target_distribution = self.critic.c51_loss.project_distribution(next_distribution, reward, dones, gamma)

            K = logits_per_head.shape[0]
            losses = [self.critic.c51_loss(logits_per_head[i], target_distribution) for i in range(K)]
            critic_loss = torch.stack(losses).mean()
        else:  # mse (default)
            q_all = self.critic(obs["feat"], obs["observation.state"], action).squeeze(-1)  # [K, B]
            td_errors = torch.abs(q_all - target_q.unsqueeze(0)).mean(dim=0)  # [B]

            if importance_weights is not None:
                weighted_td_errors = td_errors**2 * importance_weights
                critic_loss = weighted_td_errors.mean()
            else:
                critic_loss = (td_errors**2).mean()

        metrics = {}
        metrics["train/critic_qt"] = target_q.mean().item()
        metrics["train/critic_loss"] = critic_loss.item()
        metrics["_target_q"] = target_q.detach().cpu()
        if td_errors is not None:
            metrics["_td_errors"] = td_errors.detach().cpu()
        if importance_weights is not None:
            metrics["train/importance_weights_mean"] = importance_weights.mean().item()
            metrics["train/importance_weights_std"] = importance_weights.std().item()
            metrics["train/importance_weights_min"] = importance_weights.min().item()
            metrics["train/importance_weights_max"] = importance_weights.max().item()

        # Zero gradients
        self.encoder_opt.zero_grad(set_to_none=True)
        self.critic_opt.zero_grad(set_to_none=True)

        critic_loss.backward(retain_graph=True)

        # Gradient clipping
        encoder_grad_norm = torch.nn.utils.clip_grad_norm_(self.encoders.parameters(), self.cfg.critic_grad_clip_norm)
        critic_grad_norm = torch.nn.utils.clip_grad_norm_(self.critic.parameters(), self.cfg.critic_grad_clip_norm)

        metrics["train/encoder_grad_norm"] = encoder_grad_norm.item()
        metrics["train/critic_grad_norm"] = critic_grad_norm.item()

        self.encoder_opt.step()
        self.critic_opt.step()

        return metrics

    def update(
        self,
        batch,
    ):
        obs: dict[str, torch.Tensor] = batch["obs"]
        action: torch.Tensor = batch["action"]
        reward: torch.Tensor = batch[("next", "reward")]
        discount: torch.Tensor = batch["gamma"]
        next_nonterminal: torch.Tensor = batch["nonterminal"]
        next_obs: dict[str, torch.Tensor] = batch[("next", "obs")]

        effective_discount = discount * next_nonterminal

        obs["feat"] = self._encode(obs, augment=True)

        with torch.no_grad():
            next_obs["feat"] = self._encode(next_obs, augment=True)

        metrics = {}
        metrics["data/batch_R"] = reward.mean().item()

        importance_weights = batch.get("_weight", None)

        critic_metric = self.update_critic(
            obs=obs,
            action=action,
            reward=reward,
            discount=effective_discount,
            next_obs=next_obs,
            # FIX (Bug 3): stddev no longer forwarded to update_critic
            importance_weights=importance_weights,
        )
        utils.soft_update_params(self.critic, self.critic_target, self.cfg.critic_target_tau)
        metrics.update(critic_metric)

        return metrics

    def step_lr_schedulers(self):
        """Step the learning rate schedulers for warmup."""
        if self.encoder_scheduler is not None:
            self.encoder_scheduler.step()
        if self.critic_scheduler is not None:
            self.critic_scheduler.step()
        # FIX (Bug 4): actor_scheduler removed — there is no actor in this class