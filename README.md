 ## Getting Started

To clone this repository along with all submodules, run:

```bash
git clone --recursive https://github.com/rguntz/ibrl-docker.git
git checkout bahavior_cloning_ee_control

```

# Install dependencies
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

##  Create an API key for monitoring the state of computer
Visit the site https://docs.wandb.ai/models/quickstart and create an account, and then an API key to monitor online the training pipeline. Then, run the commmand in inside the terminal : 
```bash
export WANDB_API_KEY="837e47625b8acbd29a5dbc4cc5e0dcc30fca5f73"
```


# Install the packages of trossen mujoco : 
```bash 
cd trossen_arm_mujoco
pip install .
```


#### Collect the data : 
Go inside the sim_recorder folder and refer to the README.md in this separate folder for instructions on how to record data. 

#### Process the data : 
these steps are temporary as I am still looking for the best way to process the data of demonstrations before using it into the behavior cloning pipeline. Do these steps after having recorded the dataset using the sim_recorder folder. 

```shell
copy the dataset.hdf5 file from the sim_recorder file into data/cube_picking_and_placing
cd data
python extend_gripper.py
python action_t_plus_k.py
python normalize_actions.py
```

#### Train BC policy : 
```shell
python python train_bc_trossen --config_path release/cfgs/trossen_bc/transfer_cube_task.yaml 
```

#### IBRL
```shell
python train_rl_trossen.py --config_path release/cfgs/trossen/transfer_cube_task.yaml
```





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




