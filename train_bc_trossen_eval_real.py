from dataclasses import dataclass, field
from typing import Optional
import os
import sys
import yaml
import pyrallis
import numpy as np
import torch

import common_utils
from bc.dataset_trossen import DatasetConfig, RobomimicDataset
from bc.bc_policy import StateBcPolicy, StateBcPolicyConfig
from bc.bc_policy import BcPolicy, BcPolicyConfig
from evaluate import run_eval_mp
#from env.robosuite_wrapper import PixelRobosuite
from env.trossen_wrapper import PixelTrossen

class SimulatedEnv:
    pass

@dataclass
class MainConfig(common_utils.RunConfig):
    dataset: DatasetConfig = field(default_factory=lambda: DatasetConfig())
    state_policy: StateBcPolicyConfig = field(default_factory=lambda: StateBcPolicyConfig())
    policy: BcPolicyConfig = field(default_factory=lambda: BcPolicyConfig())
    # training
    seed: int = 1
    load_model: str = "none"
    num_epoch: int = 20
    epoch_len: int = 1000
    batch_size: int = 256
    lr: float = 1e-4
    grad_clip: float = 5
    weight_decay: float = 0
    # eval
    num_eval_episode: int = 20
    # to be overwritten by run() to facilitate model loading
    task_name: str = "TransferCubeEETask"
    robots: list[str] = field(default_factory=lambda: [])
    image_size: int = -1
    rl_image_size: int = -1
    # log
    save_dir: str = "exps/bc/run_ee_real_1"
    use_wb: int = 0
    save_per: int = -1

    @property
    def prop_stack(self):
        return self.dataset.prop_stack


def run(cfg: MainConfig, policy):
    print("init the dataset : ")
    dataset = RobomimicDataset(cfg.dataset)
    print("finished init dataset")
    print("cfg task name : ", cfg.task_name)

    if not cfg.dataset.real_data:
        cfg.task_name = dataset.cfg.task_name
        cfg.robots = dataset.cfg.robot
        cfg.image_size = dataset.env_params["image_size"]
        cfg.rl_image_size = dataset.env_params["rl_image_size"]
    else : # NEWLY ADDED
        cfg.task_name = dataset.cfg.task_name
        cfg.robots = dataset.cfg.robot
        cfg.image_size = dataset.env_params["image_size"]
        cfg.rl_image_size = dataset.env_params["rl_image_size"]

    pyrallis.dump(cfg, open(cfg.cfg_path, "w"))  # type: ignore
    print(common_utils.wrap_ruler("config"))
    with open(cfg.cfg_path, "r") as f:
        print(f.read(), end="")
    cfg_dict = yaml.safe_load(open(cfg.cfg_path, "r"))

    if policy is None: # this is out case. 
        print("the policy is none") 
        if cfg.dataset.use_state: # Not our case
            policy = StateBcPolicy(dataset.obs_shape, dataset.action_dim, cfg.state_policy)
        else:
            policy = BcPolicy(
                dataset.obs_shape,
                dataset.prop_shape,
                dataset.action_dim,
                dataset.cfg.rl_cameras,
                cfg.policy,
            )

    policy = policy.to("cuda")
    print(common_utils.wrap_ruler("policy weights"))
    print(policy)

    common_utils.count_parameters(policy)

    stat = common_utils.MultiCounter(
        cfg.save_dir,
        bool(cfg.use_wb),
        wb_exp_name=cfg.wb_exp,
        wb_run_name=cfg.wb_run,
        wb_group_name=cfg.wb_group,
        config=cfg_dict,
    )

    stopwatch = common_utils.Stopwatch()
    best_score = 0

    epoch = 1

    with stopwatch.time("eval"):
        seed = epoch * cfg.num_eval_episode + 1
        scores = evaluate(policy, dataset, seed=seed, num_game=cfg.num_eval_episode)
        score = float(np.mean(scores))

    best_score = max(best_score, score)
    stat["score"].append(score)
    stat["score(best)"].append(best_score)

    # quit!
    assert False


def evaluate(policy, dataset: RobomimicDataset, seed, num_game):
    return run_eval_mp(
        dataset.env_params, policy, num_game=num_game, seed=seed, num_proc=1, verbose=False
    )


def _load_model(weight_file, env: PixelTrossen, device, cfg: Optional[MainConfig] = None):
    if cfg is None:
        cfg_path = os.path.join(os.path.dirname(weight_file), f"cfg.yaml")
        print("the config path is : ", cfg_path)
        cfg = pyrallis.load(MainConfig, open(cfg_path, "r"))  # type: ignore

    print("observation shape: ", env.observation_shape)
    print("env.prop_shape : ", env.prop_shape)
    print("env.action_dim : ", env.action_dim)
    print("env.rl_cameras : ", env.rl_cameras)
    print("--------------------------------------------------------------")
    if cfg.dataset.use_state:
        policy = StateBcPolicy(env.observation_shape, env.action_dim, cfg.state_policy)
    else: # here for us. 
        policy = BcPolicy(
            env.observation_shape, env.prop_shape, env.action_dim, env.rl_cameras, cfg.policy
        )
        print("cfg policy is : ", cfg.policy, "rest :", env.observation_shape, env.prop_shape, env.action_dim, env.rl_cameras)
    policy.load_state_dict(torch.load(weight_file))
    return policy.to(device)

# function to load bc models
def load_model(weight_file, device, *, verbose=True):
    run_folder = os.path.dirname(weight_file)
    cfg_path = os.path.join(run_folder, f"cfg.yaml")
    print("the config path is from trossen bc : ", cfg_path)
    if verbose:
        print(common_utils.wrap_ruler("config of loaded agent"))
        with open(cfg_path, "r") as f:
            print(f.read(), end="")
        print(common_utils.wrap_ruler(""))

    cfg = pyrallis.load(MainConfig, open(cfg_path, "r"))  # type: ignore

    print("before init trossen task name : ", cfg.task_name)

    # assert not cfg.dataset.real_data
    env_params = dict(
        env_name=cfg.task_name,
        robots=cfg.robots,
        episode_length=cfg.dataset.eval_episode_len,
        reward_shaping=False,
        image_size=cfg.image_size,  
        rl_image_size=cfg.rl_image_size,
        camera_names=cfg.dataset.rl_cameras,
        rl_cameras=cfg.dataset.rl_cameras,
        device=device,
        use_state=cfg.dataset.use_state,
        obs_stack=cfg.dataset.obs_stack,
        state_stack=cfg.dataset.state_stack,
        prop_stack=cfg.dataset.prop_stack,
    )
    # env = PixelTrossen(**env_params)  # type: ignore

    env  = SimulatedEnv()
    env.observation_shape = (3, 128, 128)
    env.prop_shape = (16,)
    env.action_dim = 14
    env.rl_cameras = ['cam_high']

    if cfg.dataset.use_state:
        print(" observation shape",  env.observation_shape)
    else:
        print("observation shape:", env.observation_shape)

    policy = _load_model(weight_file, env, device, cfg)
    return policy, env, env_params


if __name__ == "__main__":
    import rich.traceback

    # make logging more beautiful
    rich.traceback.install()
    torch.set_printoptions(linewidth=100)

    cfg = pyrallis.parse(config_class=MainConfig)  # type: ignore

    print("env name before run : ", cfg.task_name)

    common_utils.set_all_seeds(cfg.seed)
    log_path = os.path.join(cfg.save_dir, "train.log")
    sys.stdout = common_utils.Logger(log_path, print_to_stdout=True)

    if cfg.load_model is not None and cfg.load_model != "none":
        policy = load_model(cfg.load_model, "cuda")[0]
    else:
        policy = None

    run(cfg, policy=policy)
