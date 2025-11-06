 ## Getting Started

To clone this repository along with all submodules, run:

```bash
git clone --recursive https://github.com/rguntz/ibrl-docker.git

```

Download data and BC models


Download dataset and models from [Google Drive](https://drive.google.com/file/d/1F2yH84Iqv0qRPmfH8o-kSzgtfaoqMzWE/view?usp=sharing) and put the folders under `release` folder.
The release folder should contain `release/cfgs` (already shipped with the repo), `release/data` and `release/model` (the latter two are from the downloaded zip file).

unzip the file : 
```bash
unzip data_and_model.zip
```

Build the image :
docker build -t ibrl-gpu .

## Running the Container 
```bash
docker run --gpus all -it --rm \
    -v $(pwd):/app \
    ibrl-gpu
```

Compile the CPP files : 
```bash
cd common_utils
mkdir build
cd build
cmake ..
make -j
```

##  Create an API key for progress monitoring
Visit the site https://docs.wandb.ai/models/quickstart and create an account, and then an API key to monitor online the training pipeline. Then, run the commmand in inside the terminal : 
```bash
export WANDB_API_KEY="837e47625b8acbd29a5dbc4cc5e0dcc30fca5f73"
```



### Robomimic (pixel)

Train RL policy using the BC policy provided in `release` folder

#### IBRL

```shell
# can
python train_rl.py --config_path release/cfgs/robomimic_rl/can_ibrl.yaml

# square
python train_rl.py --config_path release/cfgs/robomimic_rl/square_ibrl.yaml
```

Use `--save_dir PATH` to specify where to store the logs and models.
Use `--use_wb 0` to disable logging to weight and bias.

Use the following commands to train a BC policy from scratch.
We find that IBRL is not sensitive to the exact performance of the BC policy.
```shell
# can
python train_bc.py --config_path release/cfgs/robomimic_bc/can.yaml

# square
python train_bc.py --config_path release/cfgs/robomimic_bc/square.yaml
```

#### RLPD

```shell
# can
python train_rl.py --config_path release/cfgs/robomimic_rl/can_rlpd.yaml

# square
python train_rl.py --config_path release/cfgs/robomimic_rl/square_rlpd.yaml
```

#### RFT (Regularized Fine-Tuning)

These commands run RFT from pretrained models in `release` folder.
```shell
# can rft
python train_rl.py --config_path release/cfgs/robomimic_rl/can_rft.yaml

# square rft
python train_rl.py --config_path release/cfgs/robomimic_rl/square_rft.yaml
```

To only perform pretraining:
```shell
# can, pretraining for 5 x 10,000 steps
python train_rl.py --config_path release/cfgs/robomimic_rl/can_rft.yaml --pretrain_only 1 --pretrain_num_epoch 5 --load_pretrained_agent None

# square, pretraining for 10 x 10,000 steps
python train_rl.py --config_path release/cfgs/robomimic_rl/square_rft.yaml --pretrain_only 1 --pretrain_num_epoch 10 --load_pretrained_agent None
```
---

### Robomimic (state)

#### IBRL

Train IBRL using the provided state BC policies:
```shell
# can state
python train_rl.py --config_path release/cfgs/robomimic_rl/can_state_ibrl.yaml

# square state
python train_rl.py --config_path release/cfgs/robomimic_rl/square_state_ibrl.yaml
```

To train a state BC policy from scratch:
```shell
# can
python train_bc.py --config_path release/cfgs/robomimic_bc/can_state.yaml

# square
python train_bc.py --config_path release/cfgs/robomimic_bc/square_state.yaml
```

#### RLPD

```shell
# can state
python train_rl.py --config_path release/cfgs/robomimic_rl/can_state_rlpd.yaml

# square state
python train_rl.py --config_path release/cfgs/robomimic_rl/square_state_rlpd.yaml
```

#### RFT

Since state policies are fast to train, we can just run pretrain and RL fine-tuning in one step.
```shell
# can
python train_rl.py --config_path release/cfgs/robomimic_rl/can_state_rft.yaml

# square
python train_rl.py --config_path release/cfgs/robomimic_rl/square_state_rft.yaml
```
---

### Metaworld

#### IBRL

Train RL policy using the BC policy provided in `release` folder
```shell
# assembly
python mw_main/train_rl_mw.py --config_path release/cfgs/metaworld/ibrl_basic.yaml --bc_policy assembly

# boxclose
python mw_main/train_rl_mw.py --config_path release/cfgs/metaworld/ibrl_basic.yaml --bc_policy boxclose

# coffeepush
python mw_main/train_rl_mw.py --config_path release/cfgs/metaworld/ibrl_basic.yaml --bc_policy coffeepush

# stickpull
python mw_main/train_rl_mw.py --config_path release/cfgs/metaworld/ibrl_basic.yaml --bc_policy stickpull
```

If you want to train BC policy from scratch
```shell
python mw_main/train_bc_mw.py --dataset.path Assembly --save_dir SAVE_DIR
```

#### RPLD

Note that we still specify `bc_policy` to specify the task name, but we don't use it in baselines.
This is special to `train_rl_mw.py`.

```shell
python mw_main/train_rl_mw.py --config_path release/cfgs/metaworld/rlpd.yaml --bc_policy assembly --use_wb 0
```

#### RFT

For simplicity, here this one command performs both pretraining and RL training.
```shell
python mw_main/train_rl_mw.py --config_path release/cfgs/metaworld/rft.yaml --bc_policy assembly --use_wb 0
```
---

## Citation

```
@misc{hu2023imitation,
    title={Imitation Bootstrapped Reinforcement Learning},
    author={Hengyuan Hu and Suvir Mirchandani and Dorsa Sadigh},
    year={2023},
    eprint={2311.02198},
    archivePrefix={arXiv},
    primaryClass={cs.LG}
}
```




