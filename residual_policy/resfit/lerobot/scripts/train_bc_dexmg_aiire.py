#!/usr/bin/env python
"""Train a LeRobot policy on a dataset hosted on the Hugging Face Hub.

Training only - no evaluation rollouts or environment initialization.
"""

from __future__ import annotations

import argparse
import json
import logging
import multiprocessing as mp
import os
import re
import shutil
import time
from dataclasses import asdict, is_dataclass
from datetime import datetime
from pathlib import Path
from typing import Any

import numpy as np
import torch
from lerobot.common.datasets.factory import resolve_delta_timestamps
from lerobot.common.datasets.lerobot_dataset import LeRobotDataset, LeRobotDatasetMetadata
from lerobot.common.datasets.transforms import ImageTransforms, ImageTransformsConfig
from lerobot.common.datasets.utils import cycle
from lerobot.common.utils.random_utils import set_seed
from termcolor import colored

import wandb
from resfit.lerobot.policies.factory import make_policy, make_policy_config
from resfit.lerobot.utils.load_policy import load_checkpoint, save_checkpoint

# Set multiprocessing start method for CUDA compatibility
try:
    mp.set_start_method("spawn", force=True)
except RuntimeError:
    pass

# Caching configuration
_CACHE_ROOT = Path(os.environ.get("CACHE_DIR", ".")).expanduser().resolve()

parser = argparse.ArgumentParser(description="Offline training on a HF Hub dataset with LeRobot policies (training only)")

parser.add_argument("--no_cleanup", action="store_true", help="Do not delete the run directory after training", default = True)

# Required args
parser.add_argument(
    "--dataset", type=str, required=True, help="HF Hub dataset repo-id e.g. `ankile/franka-lift-dataset`"
)
parser.add_argument(
    "--policy",
    type=str,
    default="diffusion",
    choices=[
        "diffusion",
        "act",
        "latent_act",
        "pi0",
        "pi0fast",
        "tdmpc",
        "vqbet",
    ],
    help="Which policy architecture to train",
)

# Training hyper-parameters
parser.add_argument("--steps", type=int, default=100_000, help="Total optimization steps")
parser.add_argument("--batch_size", type=int, default=64)
parser.add_argument("--grad_clip_norm", type=float, default=10.0)
parser.add_argument("--num_workers", type=int, default=4)

# Reproducibility / device
parser.add_argument("--seed", type=int, default=None)
parser.add_argument("--device", type=str, default="cuda" if torch.cuda.is_available() else "cpu")

# Logging & checkpoints
parser.add_argument("--output_dir", type=str, default="outputs/train_hf")
parser.add_argument("--log_freq", type=int, default=100, help="How often to print & log to W&B (in steps)")
parser.add_argument("--save_freq", type=int, default=10_000, help="How often to save checkpoints (in steps)")

# WandB
parser.add_argument("--wandb_enable", action="store_true", help="Enable Weights & Biases logging")
parser.add_argument("--wandb_project", type=str, default=None, help="W&B project name (required when --wandb_enable)")
parser.add_argument("--wandb_entity", type=str, default=None)

# Resume
parser.add_argument("--resume_ckpt", type=str, default=None, help="Path to a local checkpoint directory to resume from")
parser.add_argument(
    "--resume_run_id",
    type=str,
    default=None,
    help="WandB run ID to resume from (grabs the 'latest' artifact to restore trainer state).",
)

# Policy configuration overrides
parser.add_argument(
    "--policy_kwargs",
    type=str,
    default=None,
    help=(
        "Overrides for the policy configuration. Accepts either: "
        '1) A JSON dictionary string, e.g. \'{"dim_model": 1024, "chunk_size": 100}\', or '
        "2) A compact 'key=value' list separated by commas or spaces, e.g. "
        "   'dim_model=1024,chunk_size=100 optimizer_lr=3e-4'. "
        "All pairs are forwarded directly to make_policy_config(...)."
    ),
)

# Camera selection
parser.add_argument(
    "--policy_cameras",
    type=str,
    nargs="*",
    default=None,
    help=(
        "List of camera names to use for the policy. If not specified, all cameras from the dataset will be used. "
        "Example: --policy_cameras agentview robot0_eye_in_hand"
    ),
)

# Proprioceptive observations
parser.add_argument(
    "--disable_proprioceptive_obs",
    action="store_true",
    help=(
        "Disable proprioceptive observations (observation.state) during training. "
        "Only visual observations will be used."
    ),
)

# Validation
parser.add_argument("--val_ratio", type=float, default=0.1, help="Fraction of data to use for validation (default: 0.1)")
parser.add_argument("--val_max_batches", type=int, default=10, help="Max batches to use per validation run (default: 10)")

args_cli = parser.parse_args()

# Setup logging
logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s | %(levelname)s | %(message)s",
    force=True,
)

logger = logging.getLogger(__name__)

def filter_dataset_features(
    ds_meta: LeRobotDatasetMetadata,
    keep_action_key: bool = True,
) -> LeRobotDatasetMetadata:
    """
    Filter dataset features to only include the specific observation keys used for ECU assembly tasks
    and optionally the action key.
    
    Hardcoded observation features for dual-arm Trossen ECU assembly setup:
    - 4 camera views (wrist + overhead for both arms)
    - proprioceptive state (end-effector pose)
    
    Args:
        ds_meta: The dataset metadata to filter
        keep_action_key: Whether to keep the "action" feature (default: True)
    
    Returns:
        Modified dataset metadata with filtered features
    """
    # Hardcoded observation features for this specific task
    used_obs_features = {
        "observation.images.wrist_video",
        "observation.images.video", 
        "observation.images.wrist_video_2",
        "observation.state",
    }
    
    filtered_features = {}
    
    # Add action feature if requested
    if keep_action_key and "action" in ds_meta.features:
        filtered_features["action"] = ds_meta.features["action"]
    
    # Add specified observation features
    for key in used_obs_features:
        if key in ds_meta.features and key.startswith("observation."):
            filtered_features[key] = ds_meta.features[key]
        elif key not in ds_meta.features:
            print("ds_meta.features : ", ds_meta.features)
            raise ValueError(
                f"Required observation feature '{key}' not found in dataset. "
                f"Available features: {list(ds_meta.features.keys())}"
            )
    
    # Replace features in metadata
    ds_meta.info["features"] = filtered_features
    
    return ds_meta


@torch.no_grad()
def run_validation(policy, val_loader, device, max_batches=10):
    """Run validation and return mean loss over up to max_batches batches."""
    policy.eval()
    losses = []

    for i, batch in enumerate(val_loader):
        if i >= max_batches:
            break

        for k, v in batch.items():
            if isinstance(v, torch.Tensor):
                batch[k] = v.to(device, non_blocking=True)

        loss, _ = policy.forward(batch)
        losses.append(loss.item())

    policy.train()
    return float(np.mean(losses))


def main(cfg: argparse.Namespace):
    # Device setup
    device = torch.device(cfg.device)
    logger.info(colored(f"Using device: {device}", "green"))

    # Create a timestamped folder in CACHE_DIR for all outputs
    timestamp = datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
    run_cache_dir = _CACHE_ROOT / f"bc_run_{timestamp}_{Path(cfg.dataset).name}_{cfg.policy}"
    run_cache_dir.mkdir(parents=True, exist_ok=True)

    if cfg.seed is not None:
        set_seed(cfg.seed)
        logger.info(colored(f"Random seed set to {cfg.seed}", "yellow"))

    # Dataset (metadata first, then actual dataset with resolved timestamps)
    logger.info("Fetching dataset metadata from the Hub…")
    ds_meta = LeRobotDatasetMetadata(cfg.dataset)
    ds_meta = filter_dataset_features(ds_meta, keep_action_key=True) # added function to work with aiire dataset

    print("ds_meta : ", ds_meta)

    # Build the policy configuration, applying any CLI-specified overrides
    def _infer_type(val: str):
        """Try to cast *val* to int, float or bool if possible, else return str."""
        if val.lower() in {"true", "false"}:
            return val.lower() == "true"
        try:
            if val.isdigit() or (val.startswith("-") and val[1:].isdigit()):
                return int(val)
            return float(val)
        except ValueError:
            return val

    if cfg.policy_kwargs is not None:
        policy_kwargs = {}
        try:
            policy_kwargs = json.loads(cfg.policy_kwargs)
            if not isinstance(policy_kwargs, dict):
                raise TypeError
        except Exception:
            text = cfg.policy_kwargs.strip()
            tokens = re.split(r"[ ,]+", text)
            for token in filter(None, tokens):
                if "=" not in token:
                    raise ValueError(f"Could not parse --policy_kwargs token '{token}'. Expected 'key=value'.")
                k, v = token.split("=", 1)
                policy_kwargs[k] = _infer_type(v)
    else:
        policy_kwargs = {}

    policy_cfg = make_policy_config(cfg.policy, **policy_kwargs)

    # Set the chunk size to 20 (env is at 20 fps)
    policy_cfg.chunk_size = 20
    policy_cfg.n_action_steps = 20

    if isinstance(cfg.device, str):
        policy_cfg.device = cfg.device.split(":", 1)[0]
    else:
        policy_cfg.device = cfg.device

    # Filter dataset features based on selected cameras if specified
    if cfg.policy_cameras is not None:
        logger.info(f"Filtering dataset to use only cameras: {cfg.policy_cameras}")

        filtered_features = {
            key: feature for key, feature in ds_meta.features.items() if not key.startswith("observation.images.")
        }

        available_cameras = []
        for key, feature in ds_meta.features.items():
            if key.startswith("observation.images."):
                camera_name = key.replace("observation.images.", "")
                available_cameras.append(camera_name)
                if camera_name in cfg.policy_cameras:
                    filtered_features[key] = feature

        missing_cameras = [cam for cam in cfg.policy_cameras if cam not in available_cameras]
        if missing_cameras:
            raise ValueError(
                f"Requested cameras not found in dataset: {missing_cameras}. Available cameras: {available_cameras}"
            )

        logger.info(f"Available cameras: {available_cameras}")
        logger.info(f"Selected cameras: {cfg.policy_cameras}")

        ds_meta.info["features"] = filtered_features

    # Filter dataset features to remove proprioceptive observations if specified
    if cfg.disable_proprioceptive_obs:
        logger.info("Filtering dataset to remove proprioceptive observations (observation.state)")

        filtered_features = {key: feature for key, feature in ds_meta.features.items() if key != "observation.state"}

        remaining_obs_keys = [key for key in filtered_features if key.startswith("observation")]
        if not remaining_obs_keys:
            raise ValueError(
                "Cannot disable proprioceptive observations: no other observation types found in dataset. "
                "Dataset must contain at least one image observation or environment state."
            )

        logger.info(f"Remaining observation keys after filtering: {remaining_obs_keys}")

        ds_meta.info["features"] = filtered_features

    # Determine delta-timestamps from policy indices & dataset fps
    delta_timestamps = resolve_delta_timestamps(policy_cfg, ds_meta)

    logger.info("Building LeRobotDataset with inferred delta-timestamps…")

    image_transforms_config = ImageTransformsConfig(enable=True)
    image_transforms = ImageTransforms(image_transforms_config)

    print("delta_timestamps : ", delta_timestamps)

    dataset = LeRobotDataset(
        cfg.dataset,
        delta_timestamps=delta_timestamps,
        download_videos=True,
        image_transforms=image_transforms,
    )

    # ------------------------------------------------------------------ #
    # Train / validation split                                             #
    # ------------------------------------------------------------------ #
    val_ratio = cfg.val_ratio
    n_total = len(dataset)
    n_val = int(val_ratio * n_total)
    n_train = n_total - n_val

    logger.info(
        colored(
            f"Splitting dataset: {n_train} train / {n_val} val "
            f"({val_ratio:.0%} val ratio, seed={cfg.seed or 0})",
            "yellow",
        )
    )

    train_dataset, val_dataset = torch.utils.data.random_split(
        dataset,
        [n_train, n_val],
        generator=torch.Generator().manual_seed(cfg.seed or 0),
    )

    # Dataloaders
    train_loader = torch.utils.data.DataLoader(
        train_dataset,
        batch_size=cfg.batch_size,
        shuffle=True,
        num_workers=cfg.num_workers,
        pin_memory=device.type != "cpu",
        drop_last=True,
        persistent_workers=cfg.num_workers > 0,
    )

    val_loader = torch.utils.data.DataLoader(
        val_dataset,
        batch_size=cfg.batch_size,
        shuffle=False,
        num_workers=cfg.num_workers,
        pin_memory=device.type != "cpu",
        drop_last=False,
        persistent_workers=cfg.num_workers > 0,
    )

    dl_iter = cycle(train_loader)

    # Policy + optimizer
    policy = make_policy(policy_cfg, ds_meta=ds_meta)
    policy.train()

    logger.info(colored(f"Policy config: {policy_cfg}", "cyan"))

    # Learning-rate & weight-decay fallbacks
    lr_default = getattr(policy_cfg, "optimizer_lr", 1e-4)
    wd_default = getattr(policy_cfg, "optimizer_weight_decay", 0.0)

    optimizer = torch.optim.AdamW(policy.get_optim_params(), lr=lr_default, weight_decay=wd_default)

    # Optional WandB
    if cfg.wandb_enable:
        if cfg.wandb_project is None:
            raise ValueError("--wandb_project is required when --wandb_enable is set")

        wandb_run_id = cfg.resume_run_id if cfg.resume_run_id else None

        extra_cfg: dict[str, Any] = {}
        extra_cfg.update(vars(cfg))

        try:
            extra_cfg["policy_config"] = asdict(policy_cfg) if is_dataclass(policy_cfg) else policy_cfg.__dict__
        except Exception:
            extra_cfg["policy_config"] = str(policy_cfg)

        extra_cfg["dataset_meta"] = {
            "repo_id": ds_meta.repo_id,
            "fps": ds_meta.fps,
            "robot_type": ds_meta.robot_type,
            "total_episodes": ds_meta.total_episodes,
            "total_frames": ds_meta.total_frames,
            "feature_keys": list(ds_meta.features.keys()),
        }

        extra_cfg["delta_timestamps"] = delta_timestamps
        extra_cfg["image_transforms"] = asdict(image_transforms_config)

        wandb.init(
            project=cfg.wandb_project,
            entity=cfg.wandb_entity,
            config=extra_cfg,
            name=f"{cfg.policy}_{Path(cfg.dataset).name}",
            id=wandb_run_id,
            resume="must" if wandb_run_id else None,
        )
        logger.info(colored("W&B logging enabled", "blue"))

    # Optionally resume from checkpoint
    start_step = 0
    if cfg.resume_run_id is not None:
        logger.info(colored(f"Resuming from WandB run {cfg.resume_run_id}", "cyan"))
        api = wandb.Api()
        artifact_path = (
            f"{(cfg.wandb_entity + '/' if cfg.wandb_entity else '')}"
            f"{cfg.wandb_project}/run_{cfg.resume_run_id}_latest:latest"
        )
        artifact = api.artifact(artifact_path)
        artifact_dir = Path(artifact.download())

        start_step, policy, optimizer = load_checkpoint(artifact_dir, policy, optimizer)
        policy.to(device)
    elif cfg.resume_ckpt is not None:
        logger.info(colored(f"Resuming from local checkpoint {cfg.resume_ckpt}", "cyan"))
        start_step, policy, optimizer = load_checkpoint(Path(cfg.resume_ckpt), policy, optimizer)
        policy.to(device)

    # Training loop
    output_dir = run_cache_dir
    output_dir.mkdir(parents=True, exist_ok=True)

    step = start_step

    while step < cfg.steps:
        # ------------------------------------------------------------------
        # Measure data loading time ----------------------------------------
        # ------------------------------------------------------------------
        iter_start_t = time.perf_counter()

        data_t0 = time.perf_counter()
        batch: dict[str, Any] = next(dl_iter)

        # Move tensors to device
        for key, val in batch.items():
            if isinstance(val, torch.Tensor):
                batch[key] = val.to(device, non_blocking=True)
        data_load_ms = (time.perf_counter() - data_t0) * 1000

        # ------------------------------------------------------------------
        # Measure policy update time ---------------------------------------
        # ------------------------------------------------------------------

        update_t0 = time.perf_counter()

        loss, _ = policy.forward(batch)
        print("step done : ", step)
        loss.backward()

        torch.nn.utils.clip_grad_norm_(policy.parameters(), cfg.grad_clip_norm)
        optimizer.step()
        optimizer.zero_grad(set_to_none=True)

        update_ms = (time.perf_counter() - update_t0) * 1000

        iter_ms = (time.perf_counter() - iter_start_t) * 1000

        # ------------------------------------------------------------------
        # Logging ----------------------------------------------------------
        # ------------------------------------------------------------------
        loss_val = loss.item()

        if step % cfg.log_freq == 0:
            val_loss = run_validation(policy, val_loader, device, max_batches=cfg.val_max_batches)

            msg = (
                f"[step {step:>6d}/{cfg.steps}]"
                f" train_loss: {loss_val:.4f}"
                f" val_loss: {val_loss:.4f}"
                f" | data: {data_load_ms:.1f} ms"
                f" | update: {update_ms:.1f} ms"
                f" | iter: {iter_ms:.1f} ms"
            )
            logger.info(msg)

            if wandb is not None:
                wandb.log(
                    {
                        "train/loss": loss_val,
                        "val/loss": val_loss,
                        "time/data_load_ms": data_load_ms,
                        "time/update_ms": update_ms,
                        "time/iter_ms": iter_ms,
                    },
                    step=step,
                )

        # Checkpointing ----------------------------------------------
        if (step % cfg.save_freq == 0 and step != start_step) or step + 1 == cfg.steps:
            val_loss = run_validation(policy, val_loader, device, max_batches=cfg.val_max_batches)
            val_tag = f"val{val_loss:.4f}"

            # 1) Save model-only weights (space-efficient, keeps history)
            #    └── policy_step_<n>_val<loss>
            model_dir = output_dir / f"policy_step_{step}_{val_tag}"
            model_dir.mkdir(parents=True, exist_ok=True)
            policy.save_pretrained(model_dir / "policy")

            # 2) Save full training state once under a constant "latest" directory
            latest_dir = output_dir / "latest"
            if latest_dir.exists():
                # Remove previous to avoid stale files
                shutil.rmtree(latest_dir)
            save_checkpoint(latest_dir, step, policy, optimizer)

            logger.info(
                colored(
                    f"Checkpoint saved (model-only @ {model_dir}, full state @ {latest_dir})",
                    "magenta",
                )
            )

        step += 1


    logger.info(colored("Training finished!", "green", attrs=["bold"]))
    if wandb is not None:
        wandb.finish()

    # Cleanup
    if not cfg.no_cleanup and run_cache_dir.exists():
        logger.info(f"Cleaning up run directory: {run_cache_dir}")
        shutil.rmtree(run_cache_dir)
        logger.info("Run directory cleaned up successfully.")
    elif run_cache_dir.exists():
        logger.info(f"Keeping run directory at: {run_cache_dir}")


if __name__ == "__main__":

    main(args_cli)