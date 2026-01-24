"""
Example usage of TrossenImageRunner for policy evaluation.

This script demonstrates how to integrate TrossenImageRunner into your
training pipeline to evaluate policies on Trossen AI tasks.
"""

import os
from pathlib import Path
from diffusion_policy.diffusion_policy.env_runner.simple_trossen_image_runner import TrossenImageRunner


def create_shape_meta(action_dim: int = 16) -> dict:
    """
    Create shape metadata for Trossen AI bimanual manipulation.
    
    Args:
        action_dim: Action dimension (16 for quat-based, 14 for aa-based)
        
    Returns:
        shape_meta dict
    """
    return {
        'action': {
            'shape': (action_dim,)
        },
        'obs': {
            'cam_high': {
                'shape': (128, 128, 3),
                'type': 'image'
            },
            'cam_low': {
                'shape': (128, 128, 3),
                'type': 'image'
            },
            'cam_left_wrist': {
                'shape': (128, 128, 3),
                'type': 'image'
            },
            'cam_right_wrist': {
                'shape': (128, 128, 3),
                'type': 'image'
            },
            'robot0_eef_pos': {
                'shape': (6,),
                'type': 'low_dim'
            },
            'robot0_eef_quat': {
                'shape': (8,),
                'type': 'low_dim'
            },
            'robot0_gripper_qpos': {
                'shape': (2,),
                'type': 'low_dim'
            },
        }
    }


def setup_runner(
    output_dir: str = "./outputs",
    n_train: int = 5,
    n_test: int = 10,
    n_train_vis: int = 3,
    n_test_vis: int = 5,
) -> TrossenImageRunner:
    """
    Create and configure a TrossenImageRunner.
    
    Args:
        output_dir: Directory to save videos and results
        n_train: Number of training eval conditions
        n_test: Number of test eval conditions
        n_train_vis: Number of training videos to save
        n_test_vis: Number of test videos to save
        
    Returns:
        Initialized TrossenImageRunner
    """
    # Create output directory
    Path(output_dir).mkdir(parents=True, exist_ok=True)
    
    # Create shape metadata
    shape_meta = create_shape_meta(action_dim=16)
    
    # Create runner
    runner = TrossenImageRunner(
        output_dir=output_dir,
        shape_meta=shape_meta,
        n_train=n_train,
        n_train_vis=n_train_vis,
        n_test=n_test,
        n_test_vis=n_test_vis,
        test_start_seed=10000,
        max_steps=400,
        n_obs_steps=2,
        n_action_steps=8,
        render_obs_key='cam_high',
        fps=10,
        crf=22,
        past_action=False,
        abs_action=False,
        tqdm_interval_sec=5.0,
        n_envs=None,  # Auto: n_train + n_test
        cam_list=["cam_high", "cam_low", "cam_left_wrist", "cam_right_wrist"],
        xml_path="trossen_ai_scene.xml",
    )
    
    return runner


def evaluate_policy(policy, runner: TrossenImageRunner) -> dict:
    """
    Run evaluation with a policy.
    
    Args:
        policy: The policy to evaluate (must implement BaseImagePolicy interface)
        runner: TrossenImageRunner instance
        
    Returns:
        log_data: Dict of metrics and videos for logging
    """
    log_data = runner.run(policy)
    return log_data


# Example integration in training loop
if __name__ == "__main__":
    # This is how you would use TrossenImageRunner in your training script:
    
    # 1. Setup runner (typically done once during trainer initialization)
    runner = setup_runner(
        output_dir="./outputs/eval",
        n_train=5,
        n_test=10,
    )
    print(f"Runner initialized with {len(runner.env_init_fn_dills)} eval conditions")
    
    # 2. In your training loop (e.g., every N epochs):
    # 
    # log_data = evaluate_policy(policy, runner)
    # 
    # # Log to wandb
    # import wandb
    # wandb.log(log_data)
    
    print("TrossenImageRunner setup successful!")
    print(f"Shape meta keys: {list(runner.env_meta.keys())}")
    print(f"Environment metadata: {runner.env_meta}")
