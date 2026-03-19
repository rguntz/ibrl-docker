# ResFiT

This repository contains the official release of code for paper "Residual Off-Policy RL for Finetuning Behavior Cloning Policies".

Website: https://residual-offpolicy-rl.github.io/

Paper: https://arxiv.org/abs/2509.19301

## Getting Started

### Environment Setup

#### 1. Create and Activate Conda Environment

Create a new conda environment with Python 3.10:

```bash
cd residual_policy
conda create -n residual python=3.10 -y
conda activate residual
```

#### 2. Install Core Dependencies

Install the RL finetuning dependencies:

```bash
./resfit/rl_finetuning/setup_rlpd_robosuite.sh
```

Install additional required packages:

```bash
pip install wandb
pip install draccus==0.10.0 torchrl==0.9.2
pip install hydra-core serial deepdiff matplotlib
```

#### 3. Logging into Hugging Face and wandb

Login to Hugging Face to access dataset and wandb for policy weights saving and loading:

```bash
hf auth login
wandb login
```

#### 4. Fix CUDA Support (if needed)

If you encounter CUDA-related issues, clean out CPU-only installs and reinstall CUDA-enabled packages:

```bash
# Remove CPU-only torchcodec
pip uninstall -y torchcodec

# Install CUDA-enabled wheel for CUDA 12.8
pip install --no-cache-dir torchcodec --index-url https://download.pytorch.org/whl/cu128
```

Verify CUDA is enabled:

```bash
python -c "import torch; print(torch.cuda.is_available())"
```

## Launch training

### BC policy training

First we need to train the base BC policy. Taking TwoArmCoffee as an example:

```
python resfit/lerobot/scripts/train_bc_dexmg.py \
    --dataset ankile/dexmg-two-arm-coffee \
    --policy diffusion \
    --steps 200000 \
    --batch_size 256 \
    --wandb_project dexmg-bc \
    --eval_env TwoArmCoffee \
    --rollout_freq 5000 \
    --eval_video_key observation.images.frontview \
    --eval_render_size 224 \
    --eval_num_envs 16 \
    --eval_num_episodes 100 \
    --wandb_enable
```

After training finished, put the `wandb_project_name/run_id` into the corresponding task config in [residual_td3.py](./resfit/rl_finetuning/config/residual_td3.py).

### Residual RL training

Next we can train our residual RL policy:

```
python resfit/rl_finetuning/scripts/train_residual_td3.py \
    --config-name=residual_td3_coffee_config \
    algo.prefetch_batches=4 \
    algo.n_step=5 \
    algo.gamma=0.995 \
    algo.learning_starts=10_000 \
    algo.critic_warmup_steps=10_000 \
    algo.num_updates_per_iteration=4 \
    algo.stddev_max=0.025 \
    algo.stddev_min=0.025 \
    algo.buffer_size=300_000 \
    agent.actor.action_scale=0.2 \
    agent.actor_lr=1e-6 \
    wandb.project=dexmg-coffee \
    wandb.name=resfit \
    wandb.group=resfit \
    debug=false
```

## AIIRE Control Residual Off Policy 

## Convert the dataset : 

- Download the AIIRE Dataset into the direction `residual_policy/task=entire_ecu_assembly`


```shell 
cd AIIRE
python convert_joint_to_ee_action.py
python convert_joint_to_ee_state.py
python convert_to_action_lerobot.py
python convert_to_delta_action.py
python convert_to_state_lerobot.py
python modify_episodes_stats.py
python modify_episodes_stats_keys.py
python modify_folders_name.py
python modify_parquet_key.py
```

Note that for the moment, the parameters of the files are hardcoded and you need to go through the files and check what you need to change. This needs to be adapted in the feature. 

For the info.json, currently, use the one provided in the folder `residual_policy/AIIRE/info.json` and just modify the values at the top of the json : 
-   "total_episodes": 2047,
-   "total_frames": 1500007,
-   "total_tasks": 1, 
-   "total_videos": 8412,
-   "total_chunks": 1,
- "chunks_size": 42000,

With you dataset values. 

### Debugging : 

Some mp4 are corrupted. To solve this issue, please use the following script : 
```shell 
python check_corrupted_mp4.py \
    --base-dir /home/user/project \
    --data-dir /home/user/project/data \
    --video-dir /home/user/project/videos \
    --meta-dir /home/user/project/meta \
    --task clean_corrupted_videos \
```

Some episodes have videos that are cut at the end. This happens when the timestamp of the videos don't match the timestamps of the parquet files.You need to find these videos and run the following script. 

```shell 
python check_corrupted_mp4.py \
    --base-dir /home/user/project \
    --data-dir /home/user/project/data \
    --video-dir /home/user/project/videos \
    --meta-dir /home/user/project/meta \
    --task clean_cut_videos \
    --bad-episodes 5 12 20 \
```

```shell 
python check_corrupted_mp4.py \
    --base-dir /home/user/project \
    --data-dir /home/user/project/data \
    --video-dir /home/user/project/videos \
    --meta-dir /home/user/project/meta \
    --task clean_missing_columns
```


## Train the Policy
```
python resfit/lerobot/scripts/train_bc_dexmg_aiire.py \
    --dataset path_to_aiire_dataset \
    --policy diffusion \
    --steps 200000 \
    --batch_size 64 \
    --wandb_project dexmg-bc \
    --wandb_enable
```

#### Note 

The diffusion policy is not performing very well and thus training the residual policy on top of it would not converge to any valid behavior. 


# DIPO: Diffusion Policy for Online Model-Free Reinforcement Learning

## Overview
This paper introduces **DIPO** (Diffusion Policy for Online Model-Free Reinforcement Learning), the first algorithm to successfully apply diffusion models to **online, model-free RL** tasks. While previous diffusion-based RL methods were limited to offline settings or trajectory planning, DIPO establishes a theoretical foundation for using diffusion processes as a policy representation in standard online learning frameworks.


### 3. The "Action Gradient" Method for Online Learning
A major challenge in online RL is **policy improvement**. Standard diffusion models only fit a data distribution; they do not inherently optimize for reward.
- **Innovation:** DIPO introduces a novel **Action Gradient** mechanism.
- **Mechanism:** Instead of traditional policy gradients, DIPO updates the actions in the replay buffer directly along the gradient field of the state-action value function ($Q$-function):
  $$a_t \leftarrow a_t + \eta \nabla_a Q^\pi(s_t, a_t)$$
- **Result:** This transforms existing state-action pairs in the dataset into "better" pairs (higher expected reward), which are then used to train the diffusion policy. This bridges the gap between generative modeling and reward maximization.

## Algorithm Framework (DIPO)
The training loop consists of four main steps:
1. **Data Collection:** Interact with the environment using the current diffusion policy.
2. **Critic Update:** Train the $Q$-network using standard Bellman residual minimization.
3. **Action Improvement (Action Gradient):** Modify actions in the replay buffer by ascending the $Q$-function gradient.
4. **Policy Update:** Train the diffusion model (score estimator) to match the distribution of the *improved* actions using denoising score matching.


## Key Contribution : 
The original paper uses the diffusion policy as a single-step predictor. In the new code, the diffusion policy predicts a chunk of actions, as in `Policy Agnostic RL: Offline RL and Online RL Fine-Tuning of Any Class and Backbone`. The code is adapted to the repertory of `Resfit` by training on the same task and enabling performance comparison.

## Run the code (Work in progress): 
First train the diffusion policy using the previous scripts. 

```shell
python resfit/rl_finetuning/scripts/train_residual_td3_agnostic.py     --config-name=residual_td3_coffee_config     algo.prefetch_batches=4     algo.n_step=5     algo.gamma=0.995     algo.learning_starts=10_000     algo.critic_warmup_steps=10_000     algo.num_updates_per_iteration=4     algo.stddev_max=0.025     algo.stddev_min=0.025     algo.buffer_size=300_000     agent.actor.action_scale=0.2     agent.actor_lr=1e-6     wandb.project=dexmg-coffee     wandb.name=resfit     wandb.group=resfit     debug=false
```

## Repository
- **Code:** https://github.com/BellmanTimeHut/DIPO
- **Paper:** arXiv:2305.13122
```
@article{mcallister2025flow,
  title={Flow Matching Policy Gradients},
  author={McAllister, David and Ge, Shun and Yi, Bowen and Kim, Chang Min and Weber, Erik and Choi, Hyeongseok and Kanazawa, Angjoo},
  journal={arXiv preprint arXiv:2507.21053},
  year={2025}
}
```



