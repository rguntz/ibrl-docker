import os
import sys
from dataclasses import dataclass, field
import yaml
import copy
from typing import Optional
import pyrallis
import torch
import numpy as np

import common_utils
from common_utils import ibrl_utils as utils
from evaluate import run_eval_mp
from env.trossen_wrapper import PixelTrossen
from rl.q_agent import QAgent, QAgentConfig
from rl import replay_trossen as replay
import train_bc_trossen as train_bc
from self_improving_robots.agents_discriminator import VICEAgent
import cv2

@dataclass
class MainConfig(common_utils.RunConfig):
    seed: int = 1
    # env
    task_name: str = "TransferCubeEETask"
    episode_length: int = 120
    end_on_success: int = 1
    # render image in higher resolution for recording or using pretrained models
    image_size: int = 224
    rl_image_size: int = 128
    rl_camera: str = "robot0_eye_in_hand"
    obs_stack: int = 1
    prop_stack: int = 1
    state_stack: int = 1
    # agent
    use_state: int = 0
    q_agent: QAgentConfig = field(default_factory=lambda: QAgentConfig())
    stddev_max: float = 1.0
    stddev_min: float = 0.1
    stddev_step: int = 500000
    nstep: int = 3
    discount: float = 0.99
    replay_buffer_size: int = 500
    batch_size: int = 256
    num_critic_update: int = 1
    update_freq: int = 2
    bc_policy: str = ""
    # rl with preload data
    mix_rl_rate: float = 1
    preload_num_data: int = 0 
    preload_datapath: str = ""
    freeze_bc_replay: int = 1
    # pretrain rl policy with bc and finetune
    pretrain_only: int = 1
    pretrain_num_epoch: int = 0
    pretrain_epoch_len: int = 10000
    load_pretrained_agent: str = ""
    load_policy_only: int = 1
    add_bc_loss: int = 0
    # others
    env_reward_scale: float = 1
    num_warm_up_episode: int = 50
    num_eval_episode: int = 10
    save_per_success: int = -1
    mp_eval: int = 0
    num_train_step: int = 200000
    log_per_step: int = 5000
    # log
    save_dir: str = "/home/qtf5422/Desktop/AIRE/DATA/IBRL/EXPERIMENT_RL_EE_DELTA_MEDAL"
    use_wb: int = 0
    denormalization_path : str = ""
    initial_position_file : str = ""
    # discriminator reward model config
    discrim_lr: float = 3e-4
    discrim_hidden_size: int = 128
    discrim_feature_dim: int = 50
    discrim_mixup: int = 1
    discrim_train_interval: int = 2000
    discrim_train_steps_per_iteration: int = 1
    preload_pos_data: str = ""

    def __post_init__(self):
        self.rl_cameras = self.rl_camera.split("+")
        print("rl camera from main config : ", self.rl_cameras)

        if self.bc_policy in ["none", "None"]:
            self.bc_policy = ""

        if self.bc_policy:
            print(f"Using BC policy {self.bc_policy}")
            os.path.exists(self.bc_policy)

        if self.pretrain_num_epoch > 0:
            assert self.preload_num_data > 0

        self.stddev_min = min(self.stddev_max, self.stddev_min)

        if self.preload_datapath:
            self.num_warm_up_episode += self.preload_num_data

        if self.task_name == "TwoArmTransport":
            self.robots: list[str] = ["Panda", "Panda"]
        else:
            self.robots: list[str] = ["Trossen"]

    @property
    def bc_cameras(self) -> list[str]:
        if not self.bc_policy: 
            return []
        
        bc_cfg_path = os.path.join(os.path.dirname(self.bc_policy), f"cfg.yaml")
        bc_cfg = pyrallis.load(train_bc.MainConfig, open(bc_cfg_path, "r"))
        return bc_cfg.dataset.rl_cameras

    @property
    def stddev_schedule(self):
        return f"linear({self.stddev_max},{self.stddev_min},{self.stddev_step})"


class Workspace:
    def __init__(self, cfg: MainConfig, from_main=True):
        self.work_dir = cfg.save_dir
        print(f"workspace: {self.work_dir}")

        if from_main:
            common_utils.set_all_seeds(cfg.seed)
            sys.stdout = common_utils.Logger(cfg.log_path, print_to_stdout=True)

            pyrallis.dump(cfg, open(cfg.cfg_path, "w"))
            print(common_utils.wrap_ruler("config"))
            with open(cfg.cfg_path, "r") as f:
                print(f.read(), end="")
            print(common_utils.wrap_ruler(""))

        self.cfg = cfg
        self.cfg_dict = yaml.safe_load(open(cfg.cfg_path, "r"))

        self.global_step = 0
        self.global_episode = 0
        self.train_step = 0
        self._setup_env()

        print("train_env.observation_shape", self.train_env.observation_shape, "use state : ", self.cfg.use_state)
        print("action dim shape : ", self.train_env.action_dim, "prop dim shape : ", self.train_env.prop_shape)
        self.agent = QAgent(
            self.cfg.use_state,
            self.train_env.observation_shape,
            self.train_env.prop_shape,
            self.train_env.action_dim,
            self.cfg.rl_camera,
            cfg.q_agent,
        )

        if not from_main:
            return

        if cfg.load_pretrained_agent and cfg.load_pretrained_agent != "None":
            print(f"loading loading pretrained agent from {cfg.load_pretrained_agent}")
            critic_states = copy.deepcopy(self.agent.critic.state_dict())
            self.agent.load_state_dict(torch.load(cfg.load_pretrained_agent))
            if cfg.load_policy_only:
                self.agent.critic.load_state_dict(critic_states)
                self.agent.critic_target.load_state_dict(critic_states)

        self.ref_agent = copy.deepcopy(self.agent)
        self.ref_agent.cfg.act_method = "rl"

        self.bc_policy: Optional[torch.nn.Module] = None
        if cfg.bc_policy:
            bc_policy, _, bc_env_params = train_bc.load_model(cfg.bc_policy, "cuda")
            assert bc_env_params["obs_stack"] == self.eval_env_params["obs_stack"]

            self.agent.add_bc_policy(copy.deepcopy(bc_policy))
            self.bc_policy = bc_policy

        self._setup_replay()
        self._setup_discriminator_replay()
        self._setup_discriminator()


        self.stat = common_utils.MultiCounter(
            self.work_dir,
            bool(self.cfg.use_wb),
            wb_exp_name=self.cfg.wb_exp,
            wb_run_name=self.cfg.wb_run,
            wb_group_name=self.cfg.wb_group,
            config=self.cfg_dict,
        )

    def _setup_env(self):
        self.rl_cameras: list[str] = list(set(self.cfg.rl_cameras + self.cfg.bc_cameras))
        if self.cfg.use_state:
            self.rl_cameras = []
        print(f"rl_cameras: {self.rl_cameras}")

        if self.cfg.save_per_success > 0:
            for cam in ["agentview", "robot0_eye_in_hand"]:
                if cam not in self.rl_cameras:
                    print(f"Adding {cam} to recording camera because {self.cfg.save_per_success=}")
                    self.rl_cameras.append(cam)

        self.obs_stack = self.cfg.obs_stack
        self.prop_stack = self.cfg.prop_stack

        print("episode length cfg trossen", self.cfg.episode_length)

        self.train_env = PixelTrossen(
            env_name=self.cfg.task_name,
            robots=self.cfg.robots,
            episode_length=120,
            reward_shaping=False,
            image_size=self.cfg.image_size,
            rl_image_size=self.cfg.rl_image_size,
            camera_names=self.rl_cameras,
            rl_cameras=self.rl_cameras,
            env_reward_scale=self.cfg.env_reward_scale,
            end_on_success=bool(self.cfg.end_on_success),
            use_state=bool(self.cfg.use_state),
            obs_stack=self.obs_stack,
            state_stack=self.cfg.state_stack,
            prop_stack=self.prop_stack,
            record_sim_state=bool(self.cfg.save_per_success > 0),
            denormalization_path = self.cfg.denormalization_path, 
            initial_position_file = self.cfg.initial_position_file
        )
        self.eval_env_params = dict(
            env_name=self.cfg.task_name,
            robots=self.cfg.robots,
            episode_length=120,
            reward_shaping=False,
            image_size=self.cfg.image_size,
            rl_image_size=self.cfg.rl_image_size,
            camera_names=self.rl_cameras,
            rl_cameras=self.rl_cameras,
            use_state=self.cfg.use_state,
            obs_stack=self.obs_stack,
            state_stack=self.cfg.state_stack,
            prop_stack=self.prop_stack,
        )
        self.eval_env = PixelTrossen(**self.eval_env_params, denormalization_path = self.cfg.denormalization_path, initial_position_file = self.cfg.initial_position_file)

    def _setup_replay(self): 
        use_bc = False
        if self.cfg.mix_rl_rate < 1:
            use_bc = True 
        if self.cfg.save_per_success > 0:
            use_bc = True
        if self.cfg.pretrain_num_epoch > 0 or self.cfg.add_bc_loss:
            assert self.cfg.preload_num_data
            use_bc = True

        self.replay = replay.ReplayBuffer(
            self.cfg.nstep,
            self.cfg.discount,
            frame_stack=1,
            max_episode_length=self.cfg.episode_length,
            replay_size=self.cfg.replay_buffer_size,
            use_bc=use_bc,
            save_per_success=self.cfg.save_per_success,
            save_dir=self.cfg.save_dir,
        )

        if self.cfg.preload_num_data:
            print("setting up the replay buffer : ")
            replay.add_demos_to_replay(
                self.replay,
                self.cfg.preload_datapath,
                num_data=self.cfg.preload_num_data,
                rl_cameras=self.rl_cameras,
                use_state=self.cfg.use_state,
                obs_stack=self.obs_stack,
                state_stack=self.cfg.state_stack,
                prop_stack=self.prop_stack,
                reward_scale=self.cfg.env_reward_scale,
                record_sim_state=bool(self.cfg.save_per_success > 0),
            )
        if self.cfg.freeze_bc_replay:
            assert self.cfg.save_per_success <= 0, "cannot save a non-growing replay"
            self.replay.freeze_bc_replay = True

    def _setup_discriminator_replay(self):
        """Create a simple list-based replay buffer for discriminator training."""
        self.discrim_replay_buffer = []  # Simple list to store observations
        self.discrim_max_size = int(1e6)  # Maximum buffer size

    def _create_discrim_replay_iter(self):
        """Create an iterator that yields batches from the discriminator replay buffer."""
        class SimpleBatchIterator:
            def __init__(self, buffer, batch_size):
                self.buffer = buffer
                self.batch_size = batch_size
            
            def __iter__(self):
                return self
            
            def __next__(self):
                if len(self.buffer) < self.batch_size:
                    raise StopIteration
                
                # Randomly sample batch_size observations from buffer
                indices = np.random.choice(len(self.buffer), self.batch_size, replace=False)
                batch_obs = np.array([self.buffer[i] for i in indices])
                
                # Return as numpy array (update_discriminator expects this format)
                return batch_obs
        
        return SimpleBatchIterator(self.discrim_replay_buffer, self.cfg.batch_size)

    @property
    def discrim_replay_iter_forward(self):
        """Property that returns an iterator yielding one batch."""
        if len(self.discrim_replay_buffer) < self.cfg.batch_size:
            # Not enough data yet, return empty iterator
            return iter([])
        
        # Create iterator and get one batch
        iterator = self._create_discrim_replay_iter()
        return iterator

    def _setup_discriminator(self):
        """Initialize the discriminator reward model."""
        self.forward_demos = dict(np.load('/home/qtf5422/Desktop/AIRE/ibrl-docker/self_improving_robots/vision_demos/trossen/forward.npz'))

        if 'rewards' in self.forward_demos.keys():
            goal_keys = np.where(self.forward_demos['rewards'] == 1.)[0]
        else:
            goal_width = 10
            goal_keys_end = np.where(self.forward_demos['terminals'])[0]
            goal_keys_start = goal_keys_end - goal_width
            goal_keys = np.concatenate([np.arange(start, end) for start, end in zip(goal_keys_start, goal_keys_end)], axis=0)

        self.forward_success_states = self.forward_demos['observations'][goal_keys].copy()
        self.forward_success_states = self.forward_success_states.transpose(0, 3, 1, 2)
        self.forward_success_states = torch.from_numpy(self.forward_success_states)

        pos_dataset=self.forward_success_states
        
        self.discriminator = VICEAgent(obs_shape=(3, 84, 84),
                     action_shape=self.train_env.action_dim,
                     device='cuda',
                     lr=3e-4,
                     feature_dim=50,
                     hidden_dim=256,
                     critic_target_tau=0.005, 
                     reward_scale_factor=1.0,
                     bc_reg_lambda=0.0,
                     use_tb= True,
                     num_Q=10,
                     utd_ratio=2,
                     train_alpha=False,
                     target_entropy='default',
                     share_encoder=0,
                     use_trunk=True,
                     mixup=True,
                     reward_type='logd',
                     spectral_norm=True,
                     gaussian_noise_coef=0.0,
                     pos_dataset=pos_dataset,)

        
        self.ondata = []

    def eval(self, seed, policy) -> float:
        random_state = np.random.get_state()

        if self.cfg.mp_eval:
            scores: list[float] = run_eval_mp(
                env_params=self.eval_env_params,
                agent=policy,
                num_proc=10,
                num_game=self.cfg.num_eval_episode,
                seed=seed,
                verbose=False,
                env_type = "sim", 
                denormalization_path = self.cfg.denormalization_path, 
                initial_position_file = self.cfg.initial_position_file, 
            )
        else:
            scores: list[float] = run_eval(
                env_params=self.eval_env_params,
                agent=policy,
                num_game=self.cfg.num_eval_episode,
                seed=seed,
                record_dir=None,
                verbose=False,
            )

        np.random.set_state(random_state)
        return float(np.mean(scores))

    def get_online_discriminator_metrics(self, ondata, disc_out_callable):
        """Compute metrics on online discriminator predictions."""
        metrics = dict()

        ondata = torch.from_numpy(np.stack(ondata)).to("cuda:0").type(torch.cuda.FloatTensor)
        with torch.no_grad():
            _, output = disc_out_callable(ondata, return_sig=True)
        on_labels = torch.zeros(ondata.shape[0], 1).to("cuda:0")

        metrics['online_disc_bce'] = torch.nn.BCELoss()(output, on_labels).item()
        metrics['online_disc_acc'] = ((output > 0.5) == on_labels).type(torch.float).mean().item()
        metrics['online_disc_prob'] = output.mean().item()

        return metrics

    def warm_up(self):
        obs, _ = self.train_env.reset() 
        self.replay.new_episode(obs)
        total_reward = 0
        num_episode = 0
        counter = 0
        while True:
            if self.bc_policy is not None:
                with torch.no_grad(), utils.eval_mode(self.bc_policy):
                    action = self.bc_policy.act(obs, eval_mode=True)
            elif self.cfg.load_pretrained_agent or self.cfg.pretrain_num_epoch > 0:
                with torch.no_grad(), utils.eval_mode(self.agent):
                    action = self.agent.act(obs, eval_mode=True)
            else:
                action = torch.zeros(self.train_env.action_dim)
                action = action.uniform_(-1.0, 1.0)

            obs, reward, terminal, success, image_obs = self.train_env.step(action, check_od_movement=False)
            counter += 1

            reply = {"action": action}
            self.replay.add(obs, reply, reward, terminal, success, image_obs)
            
            self.update_batch_discriminator(image_obs, terminal, action, reward, obs)

            ### discriminator training ###
            if counter % self.cfg.discrim_train_interval == 0 and len(self.ondata) > 0:
                if len(self.discrim_replay_buffer) >= self.cfg.batch_size:
                    self.train_discriminator(self.stat)

            reward_model = self.get_reward(image_obs)
            # overwrite the reward using the model's reward : 
            reward = reward_model

            if terminal:
                num_episode += 1
                total_reward += self.train_env.episode_reward
                if self.replay.size() < self.cfg.num_warm_up_episode:
                    self.replay.new_episode(obs)
                    obs, _ = self.train_env.reset()
                else:
                    break

        print(f"Warm up done. #episode: {self.replay.size()}")
        print(f"#episode from warmup: {num_episode}, #reward: {total_reward}")

    def capture_cameras(self, image):
        """Capture all camera images from self.ts and resize to server resolution"""
        if isinstance(image, torch.Tensor):
            image = image.detach().cpu().numpy()
        
        if image.shape[0] in [1, 3] and len(image.shape) == 3:
            image = image.transpose(1, 2, 0)

        resized = cv2.resize(image, (84, 84), interpolation=cv2.INTER_AREA)
        return resized.copy()

    def update_batch_discriminator(self, image_obs, terminal, action, reward, obs):
        """Simplified version that just stores observations in a list."""
        if image_obs is not None:
            image_discriminator = image_obs["cam_right_wrist"]
            image_discriminator = self.capture_cameras(image_discriminator)

            # Fix channel order to match spec: CHW
            if image_discriminator.ndim == 3 and image_discriminator.shape[2] == 3:
                image_discriminator = np.transpose(image_discriminator, (2, 0, 1))

            assert isinstance(image_discriminator, np.ndarray), f"Expected ndarray, got {type(image_discriminator)}"
            assert image_discriminator.shape == (3, 84, 84), f"Wrong shape: {image_discriminator.shape}"
            
            # Add to ondata for metrics
            self.ondata.append(image_discriminator)
            
            # Add to simple replay buffer
            self.discrim_replay_buffer.append(image_discriminator)
            
            # Maintain max size by removing oldest entries
            if len(self.discrim_replay_buffer) > self.discrim_max_size:
                self.discrim_replay_buffer.pop(0)

    def get_reward(self, image_obs):
        """Simplified version that just stores observations in a list."""
        if image_obs is not None:
            image_discriminator = image_obs["cam_right_wrist"]
            image_discriminator = self.capture_cameras(image_discriminator)

            # Fix channel order to match spec: CHW
            if image_discriminator.ndim == 3 and image_discriminator.shape[2] == 3:
                image_discriminator = np.transpose(image_discriminator, (2, 0, 1))

            assert isinstance(image_discriminator, np.ndarray), f"Expected ndarray, got {type(image_discriminator)}"
            assert image_discriminator.shape == (3, 84, 84), f"Wrong shape: {image_discriminator.shape}"

        # Use the same function as the get online get_online_discriminator_metrics 
        ondata = (
            torch.from_numpy(image_discriminator)
            .unsqueeze(0)          # add batch dimension
            .to("cuda:0")
            .type(torch.cuda.FloatTensor)
        )
        with torch.no_grad():
            _, output = self.discriminator.compute_reward(ondata, return_sig=True)

        # write to disk
        reward = output.squeeze().item()  # Python float
        with open(f"{self.work_dir}/rewards.bin", "wb") as f:
            f.write(np.float32(reward).tobytes())

        return reward


    def train(self):
        stat = self.stat
        self.agent.set_stats(self.stat)
        saver = common_utils.TopkSaver(save_dir=self.work_dir, topk=1)

        if self.replay.num_episode < self.cfg.num_warm_up_episode:
            print("doing the warmup")
            self.warm_up()
            print("finished the warmup")

        stopwatch = common_utils.Stopwatch()
        obs, _ = self.train_env.reset()
        self.replay.new_episode(obs)
        while self.global_step < self.cfg.num_train_step:

            ### act ###
            with stopwatch.time("act"), torch.no_grad(), utils.eval_mode(self.agent):
                stddev = utils.schedule(self.cfg.stddev_schedule, self.global_step)
                action = self.agent.act(obs, eval_mode=False, stddev=stddev)
                self.stat["data/stddev"].append(stddev)

            ### env.step ###
            with stopwatch.time("env step"):
                obs, reward, terminal, success, image_obs = self.train_env.step(action, check_od_movement=True)
                self.update_batch_discriminator(image_obs, terminal, action, reward, obs)
                reward_model = self.get_reward(image_obs)

            with stopwatch.time("add"):
                assert isinstance(terminal, bool)
                reply = {"action": action}
                self.replay.add(obs, reply, reward, terminal, success, image_obs)
                self.global_step += 1

            if terminal:
                with stopwatch.time("reset"):
                    self.global_episode += 1
                    self.stat["score/train_score"].append(float(success))
                    self.stat["data/episode_len"].append(self.train_env.time_step)

                    obs, _ = self.train_env.reset()
                    self.replay.new_episode(obs)

            ### logging ###
            if self.global_step % self.cfg.log_per_step == 0:
                self.log_and_save(stopwatch, self.stat, saver)

            ### train ###
            if self.global_step % self.cfg.update_freq == 0:
                with stopwatch.time("train"):
                    self.rl_train(self.stat)
                    self.train_step += 1
                    
            ### discriminator training ###
            if self.global_step % self.cfg.discrim_train_interval == 0 and len(self.ondata) > 0:
                if len(self.discrim_replay_buffer) >= self.cfg.batch_size:
                    with stopwatch.time("discriminator_train"):
                        self.train_discriminator(self.stat)

    def log_and_save(
        self,
        stopwatch: common_utils.Stopwatch,
        stat: common_utils.MultiCounter,
        saver: common_utils.TopkSaver,
    ):
        elapsed_time = stopwatch.elapsed_time_since_reset
        stat["other/speed"].append(self.cfg.log_per_step / elapsed_time)
        stat["other/elapsed_time"].append(elapsed_time)
        stat["other/episode"].append(self.global_episode)
        stat["other/step"].append(self.global_step)
        stat["other/train_step"].append(self.train_step)
        stat["other/replay"].append(self.replay.size())
        stat["score/num_success"].append(self.replay.num_success)

        if self.replay.bc_replay is not None:
            stat["data/bc_replay_size"].append(self.replay.size(bc=True))

        with stopwatch.time("eval"):
            eval_seed = (self.global_step // self.cfg.log_per_step) * self.cfg.num_eval_episode
            stat["eval/seed"].append(eval_seed)
            eval_score = self.eval(seed=eval_seed, policy=self.agent)
            stat["score/score"].append(eval_score)

            original_act_method = self.agent.cfg.act_method

            if self.agent.cfg.act_method == "ibrl_soft":
                with self.agent.override_act_method("ibrl"):
                    greedy_score = self.eval(seed=eval_seed, policy=self.agent)
                    stat["score/greedy_score"].append(greedy_score)
                    stat["score_diff/greedy-soft"].append(greedy_score - eval_score)
            assert self.agent.cfg.act_method == original_act_method

        saved = saver.save(self.agent.state_dict(), eval_score, save_latest=True)
        stat.summary(self.global_step, reset=True)
        print(f"saved?: {saved}")
        stopwatch.summary(reset=True)
        print("total time:", common_utils.sec2str(stopwatch.total_time))
        print(common_utils.get_mem_usage())

    def rl_train(self, stat: common_utils.MultiCounter):
        stddev = utils.schedule(self.cfg.stddev_schedule, self.global_step)
        for i in range(self.cfg.num_critic_update):
            if self.cfg.mix_rl_rate < 1:
                rl_bsize = int(self.cfg.batch_size * self.cfg.mix_rl_rate)
                bc_bsize = self.cfg.batch_size - rl_bsize
                batch = self.replay.sample_rl_bc(rl_bsize, bc_bsize, "cuda:0")
            else:
                batch = self.replay.sample(self.cfg.batch_size, "cuda:0")

            update_actor = i == self.cfg.num_critic_update - 1

            bc_batch = None
            if update_actor and self.cfg.add_bc_loss:
                bc_batch = self.replay.sample_bc(self.cfg.batch_size, "cuda:0")
            metrics = self.agent.update(batch, stddev, update_actor, bc_batch, self.ref_agent)
            stat.append(metrics)
            stat["data/discount"].append(batch.bootstrap.mean().item())

    def train_discriminator(self, stat: common_utils.MultiCounter):
        """Train the discriminator reward model with online exploration observations."""
        if len(self.ondata) > 0:
            metrics = self.get_online_discriminator_metrics(self.ondata, self.discriminator.compute_reward)
            for k, v in metrics.items():
                stat[f"discriminator/{k}"].append(v)
            self.ondata = []

        for _ in range(self.cfg.discrim_train_steps_per_iteration):
            metrics = self.discriminator.update_discriminator(self.discrim_replay_iter_forward)
            for k, v in metrics.items():
                stat[f"discriminator/{k}"].append(v)

    def pretrain_policy(self):
        stat = common_utils.MultiCounter(
            self.work_dir,
            bool(self.cfg.use_wb),
            wb_exp_name=self.cfg.wb_exp,
            wb_run_name=self.cfg.wb_run,
            wb_group_name=self.cfg.wb_group,
            config=self.cfg_dict,
        )
        saver = common_utils.TopkSaver(save_dir=self.work_dir, topk=1)

        for epoch in range(self.cfg.pretrain_num_epoch):
            for _ in range(self.cfg.pretrain_epoch_len):
                batch = self.replay.sample_bc(self.cfg.batch_size, "cuda")
                metrics = self.agent.pretrain_actor_with_bc(batch)

                for k, v in metrics.items():
                    stat[k].append(v)

            eval_seed = epoch * self.cfg.pretrain_epoch_len
            score = self.eval(eval_seed, policy=self.agent)
            stat["pretrain/score"].append(score)

            stat.summary(epoch, reset=True)
            saved = saver.save(self.agent.state_dict(), score, save_latest=True)
            print(f"saved?: {saved}")
            print(common_utils.get_mem_usage())


def load_model(weight_file, device):
    cfg_path = os.path.join(os.path.dirname(weight_file), f"cfg.yaml")
    print(common_utils.wrap_ruler("config of loaded agent"))
    with open(cfg_path, "r") as f:
        print(f.read(), end="")
    print(common_utils.wrap_ruler(""))

    cfg = pyrallis.load(MainConfig, open(cfg_path, "r"))
    cfg.preload_num_data = 0
    workplace = Workspace(cfg, from_main=False)

    eval_env = workplace.eval_env
    eval_env_params = workplace.eval_env_params
    agent = workplace.agent
    state_dict = torch.load(weight_file)
    agent.load_state_dict(state_dict)

    if cfg.bc_policy:
        bc_policy = train_bc._load_model(cfg.bc_policy, eval_env, device)
        agent.add_bc_policy(bc_policy)

    agent = agent.to(device)
    return agent, eval_env, eval_env_params


def main():
    cfg = pyrallis.parse(config_class=MainConfig)
    workspace = Workspace(cfg) 
    if cfg.pretrain_num_epoch > 0:
        print("Pretraining")
        workspace.pretrain_policy()
        if not cfg.pretrain_only:
            print("RL finetuning")
            workspace.train()
    else:
        print("no pretraining")
        workspace.train()

    if cfg.use_wb:
        wandb.finish()

    assert False


if __name__ == "__main__":
    import wandb
    from rich.traceback import install 

    install()
    os.environ["MUJOCO_GL"] = "egl"
    torch.backends.cudnn.allow_tf32 = True
    torch.backends.cudnn.benchmark = True
    main()