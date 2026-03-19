 ## Getting Started

To clone this repository along with all submodules, run:

```bash
git clone --recursive https://github.com/rguntz/ibrl-docker.git
git checkout bahavior_cloning_ee_control

```

## Install dependencies
First Install MuJoCo
```bash 
mkdir -p /root/.mujoco 
wget https://mujoco.org/download/mujoco210-linux-x86_64.tar.gz 
tar -xvzf mujoco210-linux-x86_64.tar.gz -C /root/.mujoco
rm mujoco210-linux-x86_64.tar.gz
```

Create the Conda env : 
```bash 
conda create --name ibrl python=3.10
```

Activate the env : 
```bash
source set_env.sh
```

Download torch : 
```bash 
pip install torch==2.7.0 torchvision==0.22.0 torchaudio==2.7.0 --index-url https://download.pytorch.org/whl/cu128
```

Install requirements : 
```bash 
pip install -r requirements.txt
```


Compile the CPP files : 
```bash
cd common_utils
mkdir build
cd build
cmake ..
make -j
```

####  Create an API key for monitoring the state of computer
Visit the site https://docs.wandb.ai/models/quickstart and create an account, and then an API key to monitor online the training pipeline. Then, run the commmand in inside the terminal : 
```bash
export WANDB_API_KEY="837e47625b8acbd29a5dbc4cc5e0dcc30fca5f73"
```


#### Install the packages of trossen mujoco : 
```bash 
cd trossen_arm_mujoco
pip install .
```

#### Install the Trossen package : 
```shell
pip install trossen==1.8.8
```
If this is not working, please refer to version of the controller. 


# REAL SETUP 


### DATA Collection : 
Refer to the `README.md`file inside `sim_recorder_ee_real/server`


### Train BC policy on the real harware : 
First, you need to update the yaml file :

```shell
release/cfgs/trossen_bc/transfer_cube_task_real.yaml
```
With your dataset paths and normalization constants path : 
```shell
dataset_tresholded_wr_shifted_delta_gripper_normed_cut_end_normalized.hdf5
delta_action_stats_scaled_bc.json
```

Run the training script : 

```shell
python train_bc_trossen_real.py --config_path release/cfgs/trossen_bc/transfer_cube_task.yaml  --save_dir /home/qtf5422/Desktop/AIRE/DATA/IBRL/run_ee_bc_real_cube_1
```

### Debugging : 
If the camera is not working, this means that the station ip adress must be switched : 
```shell 
Go to env.trossen_env/trossen_wrapper_real.py
``` 
Change for the needed station : 
```shell
STATION1 = True
STATION2 = False
```
Or : 
```shell
STATION1 = True
STATION2 = False
```

### Notes on the Trossen_env observation format

#### Observation

#### Variables
```python
qpos = self.ts.observation["qpos"]
qvel = self.ts.observation["qvel"]

robot0_eef_pos = self.ts.observation["robot0_eef_pos"]
robot0_eef_aa = self.ts.observation["robot0_eef_aa"]
robot0_eef_quat = self.ts.observation["robot0_eef_quat"]

robot0_gripper_qpos = self.ts.observation["robot0_gripper_qpos"]

```

- Quaternion format: [w, x, y, z]
- Bimanual data is concatenated: [left_robot, right_robot]
- Applies to: robot0_eef_pos, robot0_eef_aa, robot0_eef_quat, robot0_gripper_qpos

### Evaluation of the model : 

```shell
python train_bc_trossen_real.py --config_path release/cfgs/trossen_bc/transfer_cube_task.yaml --load_model "/home/qtf5422/Desktop/AIRE/DATA/IBRL/run_ee_bc_real_cube_1/checkpoint_step_18.pt" --training_type eval --save_dir /home/qtf5422/Desktop/AIRE/DATA/IBRL/run_ee_bc_real_cube_1_eval
```

### Train RL policy on the real harware : 

Change the yaml file in : 

```shell
release/cfgs/trossen_rl/transfer_cube_task_real.yaml
```

Add the part 
#### First time running the RL code (meaning hardware didn't crash before and we have saved at least 1 RL model) 

```shell 
python train_rl_trossen_real.py --config_path release/cfgs/trossen_rl/transfer_cube_task_real.yaml --save_dir /home/qtf5422/Desktop/AIRE/DATA/IBRL/run_ee_rl_real_cube_1
```

#### Second time running the code 
```shell 
python train_rl_trossen_real.py --config_path release/cfgs/trossen_rl/transfer_cube_task_real.yaml --load_pretrained_agent "/home/qtf5422/Desktop/AIRE/DATA/IBRL/run_ee_rl_real_cube_1/latest.pt"--load_policy_only 0 --save_dir /home/qtf5422/Desktop/AIRE/DATA/IBRL/run_ee_rl_real_cube_1
```
This will load the latest policy before any hardware crash happened. 

#### Commands During RL Training . 
- Success : Space
- Next episode : Enter
- Pause : Del

## Code Structure

### Entry Points
The scripts : 
- `train_rl_trossen_real.py`
- `train_bc_trossen_real.py`

Interact with the wrapper:
- `env/trossen_wrapper_real.py`

---

### Environment Hierarchy

- `trossen_wrapper_real.py`  
  - Provides a **reset/step interface**
  - Wraps the lower-level environment   **Trossen_env**

- `trossen_env.py`  
  - Also implements **reset/step**
  - Directly interacts with the **hardware**

---

### Important Parameters

When creating `trossen_wrapper_real`, two file paths are required:

- `denormalization_path_bc`  
  - Path to normalization statistics  
  - JSON file: `delta_action_stats_scaled.json`  
  - Contains dataset **denormalization constants**

- `initial_position_file`  
  - HDF5 dataset:  
    `dataset_tresholded_wr_shifted_delta_gripper_normed_cut_end_normalized.hdf5`  
  - Used to compute the **mean initial position** across demonstrations  
  - Ensures inference (BC or RL) starts from a state consistent with the training data distribution



### Action Structure

The action is represented as `[left_arm, right_arm]`, where each arm includes the following components:

- **dx, dy, dz** – Cartesian position increments (Δx, Δy, Δz) representing the desired change in end-effector position.  
- **drx, dry, drz** – rotational increments (Δroll, Δpitch, Δyaw) representing the desired change in end-effector orientation, typically expressed in radians.  
- **gripper** – an absolute value indicating the gripper state (e.g., open or closed).

All positional and rotational components are **delta values** relative to the current state, whereas the gripper is an **absolute command**.



# SIMULATION 

### DATA COLLECTION 

Refer to the README.md file inside the direction `sim_recorder_ee. `


### BC Training : 

- Modify the configuration file located at  
  `release/cfgs/trossen_bc/transfer_cube_task_sim.yaml`.

- `denormalization_path` specifies the file containing the **unnormalization factors** used to rescale the policy outputs before applying them in the environment.  
  This should point to the file `delta_action_stats.json`.

- `initial_position_file` defines the path to the **processed dataset**.  
  It is used to initialize each simulation run at the same starting positions as in the dataset demonstrations. This ensures that evaluations remain **within the training distribution**, leading to more consistent and meaningful results.

Run the training script : 
```shell
python train_bc_trossen.py --config_path release/cfgs/trossen_bc/transfer_cube_task_sim.yaml --save_dir /home/qtf5422/Desktop/AIRE/DATA/IBRL/EXPERIMENT_BC_EE
```

### RL Training : 
- Modify the configuration file located at  
  `release/cfgs/trossen_rl/transfer_cube_task_sim.yaml`.

Run the training script : 
```shell 
pyhton train_rl_trossen.py --config_path release/cfgs/trossen_rl/transfer_cube_task_sim.yaml --save_dir /home/qtf5422/Desktop/AIRE/DATA/IBRL/EXPERIMENT_RL_EE
```


# Residual Policy : 
Check the code for applying residual policy inside the direction `residual_policy`
And refer to the README of the residual policy folder. 

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




