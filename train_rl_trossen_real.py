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
#from env.robosuite_wrapper import PixelRobosuite
from env.trossen_wrapper_real import PixelTrossen
from rl.q_agent import QAgent, QAgentConfig
from rl import replay_trossen as replay
from rl.replay_persistence import ReplayPersistence
import train_bc_trossen_real as train_bc

import os
import pickle
import torch
from pathlib import Path
import time 

class SimulatedEnv:
    pass

@dataclass
class MainConfig(common_utils.RunConfig):
    seed: int = 1
    # env
    task_name: str = "TransferCubeEETask"
    episode_length: int = 120 # this is a dummy number the real one is specified inside the yaml file. 
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
    nstep: int = 3 # steps of reward calculation. 
    discount: float = 0.99
    replay_buffer_size: int = 500
    batch_size: int = 256
    num_critic_update: int = 1
    update_freq: int = 2
    bc_policy: str = ""
    # rl with preload data
    mix_rl_rate: float = 1  # 1: only use rl, <1, mix in some bc data
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
    mp_eval: int = 0  # eval with multiprocess
    num_train_step: int = 200000
    log_per_step: int = 200
    # log
    save_dir: str = "/home/qtf5422/Desktop/AIRE/DATA/IBRL/run_ee_rl_real_cube_1"
    use_wb: int = 0
    denormalization_path_bc : str = ""
    initial_position_file : str = ""
    denormalization_path_rl : str = ""

    def __post_init__(self): # gets automatically called when initializing the class. 
        self.rl_cameras = self.rl_camera.split("+")
        print("rl camera from main config : ", self.rl_cameras)

        if self.bc_policy in ["none", "None"]: #  self.bc_policy is directly initilized from the definition of the class. 
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
        else: # only one robot. 
            self.robots: list[str] = ["Trossen"] # initialize the name of the robot. 

    @property
    def bc_cameras(self) -> list[str]:
        if not self.bc_policy: 
            return []
        
        # we are doing this below : 
        bc_cfg_path = os.path.join(os.path.dirname(self.bc_policy), f"cfg.yaml")
        bc_cfg = pyrallis.load(train_bc.MainConfig, open(bc_cfg_path, "r"))  # type: ignore
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

            pyrallis.dump(cfg, open(cfg.cfg_path, "w"))  # type: ignore
            print(common_utils.wrap_ruler("config"))
            with open(cfg.cfg_path, "r") as f:
                print(f.read(), end="")
            print(common_utils.wrap_ruler(""))

        self.cfg = cfg
        self.cfg_dict = yaml.safe_load(open(cfg.cfg_path, "r"))

        self.global_step = 0
        self.global_episode = 0
        self.train_step = 0
        
        # Initialize replay persistence
        replay_persist_dir = os.path.join(cfg.save_dir, "replay_persist")
        self.replay_persistence = ReplayPersistence(replay_persist_dir)
        
        # Tracking for episode data
        self.current_episode_obs = []
        self.current_episode_actions = []
        self.current_episode_rewards = []
        self.current_episode_terminals = []
        self.current_episode_success = False
        
        self._setup_env()

        print("train_env.observation_shape", self.train_env.observation_shape, "use state : ", self.cfg.use_state) # (3, 96, 96) I guess its for 96 pixels by 96 pixels and 3 for the rgb. 
        print("action dim shape : ", self.train_env.action_dim, "prop dim shape : ", self.train_env.prop_shape)
        self.agent = QAgent( # init the agent. 
            self.cfg.use_state, # in our case its 0 => false. 
            self.train_env.observation_shape,
            self.train_env.prop_shape,
            self.train_env.action_dim,
            self.cfg.rl_camera,
            cfg.q_agent,
        )

        if not from_main:
            return

        if cfg.load_pretrained_agent and cfg.load_pretrained_agent != "None": #  if we have specified a pre-trained agent. 
            print(f"loading loading pretrained agent from {cfg.load_pretrained_agent}")
            critic_states = copy.deepcopy(self.agent.critic.state_dict())
            self.agent.load_state_dict(torch.load(cfg.load_pretrained_agent))
            if cfg.load_policy_only:
                # avoid overwriting critic
                self.agent.critic.load_state_dict(critic_states)
                self.agent.critic_target.load_state_dict(critic_states)

        self.ref_agent = copy.deepcopy(self.agent)
        # override to always use RL even when self.agent is ibrl
        self.ref_agent.cfg.act_method = "rl"

        # set up bc related stuff
        self.bc_policy: Optional[torch.nn.Module] = None
        if cfg.bc_policy:
            bc_policy, _, bc_env_params = train_bc.load_model(cfg.bc_policy, "cuda") # load the imitation learning policy. 
            assert bc_env_params["obs_stack"] == self.eval_env_params["obs_stack"]

            self.agent.add_bc_policy(copy.deepcopy(bc_policy))
            self.bc_policy = bc_policy

        self._setup_replay()


    def _setup_env(self):
        self.rl_cameras: list[str] = list(set(self.cfg.rl_cameras + self.cfg.bc_cameras)) # the bc camera is the same as the rl camera. 
        #  If RL and BC cameras are the same, then this just returns that single list.
        if self.cfg.use_state: # not our case 
            self.rl_cameras = []
        print(f"rl_cameras: {self.rl_cameras}")

        if self.cfg.save_per_success > 0: # not entering this at the begining. 
            for cam in ["agentview", "robot0_eye_in_hand"]:
                if cam not in self.rl_cameras:
                    print(f"Adding {cam} to recording camera because {self.cfg.save_per_success=}")
                    self.rl_cameras.append(cam)

        self.obs_stack = self.cfg.obs_stack # default value is obs_stack
        self.prop_stack = self.cfg.prop_stack # number of proprioceptive inputs (robot joint states, gripper info) to stack.

        print("episode length cfg trossen", self.cfg.episode_length)
        # initialize at 1. 

        self.train_env = PixelTrossen( # Create the training environment
            env_name=self.cfg.task_name,
            robots=self.cfg.robots,
            episode_length=self.cfg.episode_length,
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
            denormalization_path_bc = self.cfg.denormalization_path_bc,
            initial_position_file = self.cfg.initial_position_file, 
            denormalization_path_rl = self.cfg.denormalization_path_rl
        )
        self.eval_env_params = dict(
            env_name=self.cfg.task_name,
            robots=self.cfg.robots,
            episode_length=self.cfg.episode_length,
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
        self.eval_env = SimulatedEnv()
        self.eval_env.observation_shape = (3, 128, 128)
        self.eval_env.prop_shape = (16,)
        self.eval_env.action_dim = 14
        self.eval_env.rl_cameras = ['cam_high']

    def _setup_replay(self): 
        use_bc = False
        if self.cfg.mix_rl_rate < 1: # If mix_rl_rate < 1: you're mixing RL data with some BC data.
            use_bc = True 
        if self.cfg.save_per_success > 0: # If save_per_success > 0: you plan to save successful episodes (often for imitation), so BC is needed.
            use_bc = True
        if self.cfg.pretrain_num_epoch > 0 or self.cfg.add_bc_loss: # If you’re doing pretraining (pretrain_num_epoch > 0) or adding a BC loss (add_bc_loss), then you must load BC demo data 
            assert self.cfg.preload_num_data
            use_bc = True

        self.replay = replay.ReplayBuffer(  # initialize the init buffer. 
            self.cfg.nstep,
            self.cfg.discount,
            frame_stack=1,
            max_episode_length=self.cfg.episode_length,
            replay_size=self.cfg.replay_buffer_size,
            use_bc=use_bc,
            save_per_success=self.cfg.save_per_success,
            save_dir=self.cfg.save_dir,
        )

        if self.cfg.preload_num_data: # Optionally load expert demonstration data
            #  it is initialize to 10 so 10 demos. 
            print("setting up the replay buffer : ")
            replay.add_demos_to_replay(
                self.replay, # use the instance of the replay buffer just create above. 
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
            # Save preloaded demonstrations to disk for crash recovery
            self._save_preload_data_to_disk()
        if self.cfg.freeze_bc_replay:
            assert self.cfg.save_per_success <= 0, "cannot save a non-growing replay"
            self.replay.freeze_bc_replay = True
        
        # Attempt to recover saved episodes from previous runs
        self._recover_replay_from_disk()

    def _recover_replay_from_disk(self):
        """
        Recover replay buffer episodes from disk after a crash.
        This loads all saved episodes and adds them back to the replay buffer.
        """
        from rl.replay_persistence import load_episode_from_file
        
        print("Attempting to recover replay buffer from disk...")
        
        # Recover BC episodes first (these are from preloading)
        bc_episodes = self.replay_persistence.get_recovery_episodes("bc")
        if bc_episodes and self.replay.bc_replay is not None:
            print(f"Found {len(bc_episodes)} saved BC episodes, recovering...")
            for episode_file in bc_episodes:
                try:
                    episode_data = load_episode_from_file(episode_file)
                    self._add_episode_to_replay(
                        episode_data,
                        replay_type="bc",
                    )
                except Exception as e:
                    print(f"Error recovering BC episode {episode_file}: {e}")
        
        # Recover warmup episodes
        warmup_episodes = self.replay_persistence.get_recovery_episodes("warmup")
        if warmup_episodes:
            print(f"Found {len(warmup_episodes)} saved warmup episodes, recovering...")
            for episode_file in warmup_episodes:
                try:
                    episode_data = load_episode_from_file(episode_file)
                    self._add_episode_to_replay(
                        episode_data,
                        replay_type="rl",
                    )
                except Exception as e:
                    print(f"Error recovering warmup episode {episode_file}: {e}")
        
        # Recover RL training episodes
        rl_episodes = self.replay_persistence.get_recovery_episodes("rl")
        if rl_episodes:
            print(f"Found {len(rl_episodes)} saved RL training episodes, recovering...")
            for episode_file in rl_episodes:
                try:
                    episode_data = load_episode_from_file(episode_file)
                    self._add_episode_to_replay(
                        episode_data,
                        replay_type="rl",
                    )
                except Exception as e:
                    print(f"Error recovering RL episode {episode_file}: {e}")
        
        total_recovered = len(bc_episodes) + len(warmup_episodes) + len(rl_episodes)
        if total_recovered > 0:
            print(f"Successfully recovered {total_recovered} episodes from disk")
            print(f"Replay buffer size: {self.replay.size()}")
            if self.replay.bc_replay is not None:
                print(f"BC replay buffer size: {self.replay.bc_replay.size()}")

    def _add_episode_to_replay(self, episode_data: dict, replay_type: str = "rl"):
        """
        Add a recovered episode to the replay buffer.
        
        Args:
            episode_data: Dictionary containing obs, actions, rewards, terminals
            replay_type: Type of replay ("rl" or "bc")
        """
        obs_dict = episode_data.get('obs', {})
        actions = episode_data.get('actions')
        rewards = episode_data.get('rewards')
        terminals = episode_data.get('terminals')
        success = episode_data.get('success', False)
        
        if actions is None or rewards is None:
            print("Skipping episode: missing actions or rewards")
            return
        
        # Convert to proper types
        if isinstance(actions, np.ndarray):
            actions = torch.from_numpy(actions).float()
        if isinstance(rewards, np.ndarray):
            rewards = torch.from_numpy(rewards).float()
        if isinstance(terminals, np.ndarray):
            terminals = torch.from_numpy(terminals).float()
        
        episode_len = actions.shape[0]
        
        # Convert observations to tensors
        for key in obs_dict:
            if isinstance(obs_dict[key], np.ndarray):
                obs_dict[key] = torch.from_numpy(obs_dict[key])
        
        # Initialize episode
        # Get first observation
        first_obs = {}
        for key, val in obs_dict.items():
            obs_item = val[0]
            if isinstance(obs_item, np.ndarray):
                obs_item = torch.from_numpy(obs_item)
            first_obs[key] = obs_item
        
        self.replay.new_episode(first_obs)
        
        # Add each step to the replay
        for step in range(episode_len):
            # Get observation at this step
            obs = {}
            for key, val in obs_dict.items():
                obs_item = val[step]
                if isinstance(obs_item, np.ndarray):
                    obs_item = torch.from_numpy(obs_item)
                obs[key] = obs_item
            
            # Get action, reward, terminal at this step
            action_t = actions[step]
            if isinstance(action_t, np.ndarray):
                action_t = torch.from_numpy(action_t)
            reward_t = float(rewards[step])
            terminal_t = bool(terminals[step])
            
            # Add to replay
            reply = {"action": action_t}
            self.replay.add(obs, reply, reward_t, terminal_t, success, image_obs={})

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
        return float(np.mean(scores))  # type: ignore

    def _save_preload_data_to_disk(self):
        """Save preloaded BC demonstrations to disk for crash recovery."""
        print("Saving preloaded BC demonstrations to disk...")
        if self.replay.bc_replay is None:
            return
        
        try:
            size = self.replay.bc_replay.size()
            if size > 0:
                episodes = self.replay.bc_replay.get_range(0, size, "cpu")
                self._save_episodes_to_disk(
                    episodes,
                    episode_type="bc",
                    start_id=0,
                )
            print(f"Successfully saved {size} preloaded BC episodes")
        except Exception as e:
            print(f"Error saving preloaded data: {e}")

    def _reset_episode_tracking(self):
        """Reset tracking variables for a new episode."""
        self.current_episode_obs = []
        self.current_episode_actions = []
        self.current_episode_rewards = []
        self.current_episode_terminals = []
        self.current_episode_success = False

    def _track_episode_step(
        self,
        obs: dict,
        action: torch.Tensor,
        reward: float,
        terminal: bool,
        success: bool,
    ):
        """Track a single step of the current episode."""
        # Store observations (convert to numpy if needed)
        obs_numpy = {}
        for key, val in obs.items():
            if isinstance(val, torch.Tensor):
                obs_numpy[key] = val.cpu().numpy()
            else:
                obs_numpy[key] = np.array(val)
        
        self.current_episode_obs.append(obs_numpy)
        
        # Store action
        if isinstance(action, torch.Tensor):
            self.current_episode_actions.append(action.cpu().numpy())
        else:
            self.current_episode_actions.append(np.array(action))
        
        # Store reward and terminal
        self.current_episode_rewards.append(float(reward))
        self.current_episode_terminals.append(float(terminal))
        
        # Update success status
        if success:
            self.current_episode_success = True

    def _save_current_episode_to_disk(
        self,
        episode_type: str = "rl",
    ) -> Optional[str]:
        """Save the current episode trajectory to disk."""
        if not self.current_episode_actions:
            return None
        
        try:
            # Prepare episode data
            episode_data = {
                'obs': self._stack_obs_list(self.current_episode_obs),
                'actions': np.array(self.current_episode_actions),
                'rewards': np.array(self.current_episode_rewards),
                'terminals': np.array(self.current_episode_terminals),
                'episode_type': episode_type,
                'success': self.current_episode_success,
            }
            
            # Save to disk
            filepath = self.replay_persistence.save_episode(
                episode_data,
                episode_type=episode_type,
                success=self.current_episode_success,
            )
            return filepath
        except Exception as e:
            print(f"Error saving episode to disk: {e}")
            return None
        finally:
            self._reset_episode_tracking()

    def _stack_obs_list(self, obs_list: list[dict]) -> dict:
        """Stack a list of observation dicts into single arrays."""
        stacked_obs = {}
        if not obs_list:
            return stacked_obs
        
        # Get keys from first observation
        keys = obs_list[0].keys()
        
        for key in keys:
            obs_values = [obs[key] for obs in obs_list]
            # Stack along time dimension
            stacked_obs[key] = np.stack(obs_values, axis=0)
        
        return stacked_obs

    def _save_episodes_to_disk(
        self,
        episodes,
        episode_type: str = "rl",
        start_id: int = 0,
    ):
        """Save multiple episodes from replay buffer to disk."""
        from env.trossen_wrapper import DEFAULT_STATE_KEYS
        
        size = int(episodes.seq_len.size(0))
        for i in range(size):
            try:
                episode_len = int(episodes.seq_len[i].item())
                
                # Extract observation data
                obs_data = {}
                for k, v in episodes.obs.items():
                    obs_data[k] = v[:episode_len, i].numpy()
                
                # Extract actions and rewards
                actions = episodes.action["action"][:episode_len, i].numpy()
                rewards = episodes.reward[:episode_len, i].numpy()
                
                # Extract terminals (assume last step is always terminal for saved episodes)
                terminals = np.zeros(episode_len)
                terminals[-1] = 1.0
                
                # Determine success
                success = bool(rewards[-1] == 1.0)
                
                episode_data = {
                    'obs': obs_data,
                    'actions': actions,
                    'rewards': rewards,
                    'terminals': terminals,
                    'episode_type': episode_type,
                    'success': success,
                }
                
                self.replay_persistence.save_episode(
                    episode_data,
                    episode_type=episode_type,
                    success=success,
                    episode_id=start_id + i,
                )
            except Exception as e:
                print(f"Error saving episode {i}: {e}")

    def warm_up(self):
        # warm up stage, fill the replay with some episodes
        # it can either be human demos, or generated by the bc, or purely random
        obs, _ = self.train_env.reset() 
        self.replay.new_episode(obs)
        self._reset_episode_tracking()
        total_reward = 0
        num_episode = 0
        counter = 0
        while True:
            if self.bc_policy is not None: # if we have a bc policy then we use it to fill the replay buffer. 
                # we have a BC policy
                with torch.no_grad(), utils.eval_mode(self.bc_policy):
                    start_time = time.time()
                    action = self.bc_policy.act(obs, eval_mode=True)
                    elapsed_time = time.time() - start_time
            elif self.cfg.load_pretrained_agent or self.cfg.pretrain_num_epoch > 0: # if we jave a pre-trained RL agent. 
                # the policy has been pretrained/initialized
                with torch.no_grad(), utils.eval_mode(self.agent):
                    action = self.agent.act(obs, eval_mode=True)
            else: # random actions => used at the begining.
                action = torch.zeros(self.train_env.action_dim)
                action = action.uniform_(-1.0, 1.0)

            obs, reward, terminal, success, image_obs = self.train_env.step(action, check_od_movement=False)
            counter += 1

            # Track episode step
            self._track_episode_step(obs, action, reward, terminal, success)

            #self.train_env.env.render()
            reply = {"action": action}
            self.replay.add(obs, reply, reward, terminal, success, image_obs)

            if terminal:
                num_episode += 1
                total_reward += self.train_env.episode_reward
                
                # Save episode to disk
                self._save_current_episode_to_disk(episode_type="warmup")
                
                if self.replay.size() < self.cfg.num_warm_up_episode:
                    self.replay.new_episode(obs)
                    obs, _ = self.train_env.reset()
                    self._reset_episode_tracking()
                else:
                    break

        print(f"Warm up done. #episode: {self.replay.size()}")
        print(f"#episode from warmup: {num_episode}, #reward: {total_reward}")

    def train(self):
        stat = common_utils.MultiCounter(
            self.work_dir,
            bool(self.cfg.use_wb),
            wb_exp_name=self.cfg.wb_exp,
            wb_run_name=self.cfg.wb_run,
            wb_group_name=self.cfg.wb_group,
            config=self.cfg_dict,
        )
        self.agent.set_stats(stat)
        saver = common_utils.TopkSaver(save_dir=self.work_dir, topk=1)

        if self.replay.num_episode < self.cfg.num_warm_up_episode:
            print("doing the warmup")
            self.warm_up() # fill the replay buffer with demo data or the behavior cloning policy
            #self.warm_up_with_checkpointing()
            print("finished the warmup")

        stopwatch = common_utils.Stopwatch()
        obs, _ = self.train_env.reset() # reset the env 
        self.replay.new_episode(obs)
        self._reset_episode_tracking()
        while self.global_step < self.cfg.num_train_step: # run until we attain max steps. 

            print("POLICY TRAINING")
            print("size the replay buffer : ", self.replay.size())

            ### act ###
            start_time = time.time()  # start timing
            with stopwatch.time("act"), torch.no_grad(), utils.eval_mode(self.agent):
                stddev = utils.schedule(self.cfg.stddev_schedule, self.global_step) # this is noise added to the observation. 
                action = self.agent.act(obs, eval_mode=False, stddev=stddev)
                stat["data/stddev"].append(stddev)
            
            elapsed_time = time.time() - start_time

            ### env.step ###
            with stopwatch.time("env step"): # Send the action to the simulator.
                obs, reward, terminal, success, image_obs = self.train_env.step(action, check_od_movement=True, warmup_bool = False)
                #self.train_env.env.render()

            with stopwatch.time("add"): # Save the transition into the replay buffer.
                assert isinstance(terminal, bool)
                reply = {"action": action}
                self.replay.add(obs, reply, reward, terminal, success, image_obs)
                self.global_step += 1

                print("self.global_ste : ", self.global_step)
                
                # Track episode step
                self._track_episode_step(obs, action, reward, terminal, success)

            if terminal: # if the episode has ended. 
                with stopwatch.time("reset"):
                    self.global_episode += 1
                    stat["score/train_score"].append(float(success))
                    stat["data/episode_len"].append(self.train_env.time_step)

                    # Save episode to disk for crash recovery
                    self._save_current_episode_to_disk(episode_type="rl")
                    
                    # reset env
                    obs, _ = self.train_env.reset()
                    self.replay.new_episode(obs)
                    self._reset_episode_tracking()

            ### logging ###
            if self.global_step % self.cfg.log_per_step == 0:
                print("==========================")
                print("saving the model<")
                print("==========================")
                self.log_and_save(stopwatch, stat, saver)

            ### train ### => update the agent every 2 steps for example. 
            if self.global_step % self.cfg.update_freq == 0:
                with stopwatch.time("train"):
                    self.rl_train(stat)
                    self.train_step += 1

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

        # --------------------------------------------------
        # Unconditional save with increasing name
        # --------------------------------------------------
        # save_name = f"checkpoint_step_{self.global_step:08d}"

        import re

        # Get all checkpoint files in the workspace folder
        checkpoint_files = [f for f in os.listdir(self.work_dir) if f.startswith("checkpoint_step_") and f.endswith(".pt")]

        # Extract step numbers using regex
        step_numbers = []
        for f in checkpoint_files:
            match = re.search(r"checkpoint_step_(\d+).pt", f)
            if match:
                step_numbers.append(int(match.group(1)))

        # Find the maximum step number (0 if no checkpoints exist)
        max_existing_step = max(step_numbers) if step_numbers else 0

        # New checkpoint name: max existing + current global_step
        save_name = f"checkpoint_step_{max_existing_step + self.global_step:08d}"
        print("saved file model : ", save_name)


        saver.save(
            state_dict=self.agent.state_dict(),
            perf=None,                     # disables top-k logic
            force_save_name=save_name,     # unique filename
            save_latest=True,              # optional: also overwrite latest.pt
            config=self.cfg,               # optional
        )

        stat.summary(self.global_step, reset=True)
        print(f"Saved checkpoint: {save_name}.pt")
        stopwatch.summary(reset=True)
        print("total time:", common_utils.sec2str(stopwatch.total_time))
        print(common_utils.get_mem_usage())


    def rl_train(self, stat: common_utils.MultiCounter): # the actual learner. 
        stddev = utils.schedule(self.cfg.stddev_schedule, self.global_step)
        for i in range(self.cfg.num_critic_update): # update the critic multiple times per actor update. 
            if self.cfg.mix_rl_rate < 1: # if we mix RL and BC data. 
                rl_bsize = int(self.cfg.batch_size * self.cfg.mix_rl_rate)
                bc_bsize = self.cfg.batch_size - rl_bsize
                batch = self.replay.sample_rl_bc(rl_bsize, bc_bsize, "cuda:0")
            else:
                batch = self.replay.sample(self.cfg.batch_size, "cuda:0")

            # in RED-Q, only update actor once
            update_actor = i == self.cfg.num_critic_update - 1

            bc_batch = None
            if update_actor and self.cfg.add_bc_loss:
                bc_batch = self.replay.sample_bc(self.cfg.batch_size, "cuda:0")
            metrics = self.agent.update(batch, stddev, update_actor, bc_batch, self.ref_agent)
            stat.append(metrics)
            stat["data/discount"].append(batch.bootstrap.mean().item())

    def pretrain_policy(self): # pre-train the behavioral cloning. 
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

    cfg = pyrallis.load(MainConfig, open(cfg_path, "r"))  # type: ignore
    cfg.preload_num_data = 0  # override this to avoid loading data
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
    cfg = pyrallis.parse(config_class=MainConfig)  # this will read the file "can_ibrl.yaml" which has the variables used for the simulation (model path and so on)
    workspace = Workspace(cfg) 
    if cfg.pretrain_num_epoch > 0:
        print("Pretraining")
        workspace.pretrain_policy()
        if not cfg.pretrain_only:
            # not our case
            print("RL finetuning")
            workspace.train()
    else:
        print("no pretraining") # => we are in this case. 
        workspace.train()

    if cfg.use_wb:
        wandb.finish()

    assert False


if __name__ == "__main__":
    import wandb
    from rich.traceback import install 

    install()
    os.environ["MUJOCO_GL"] = "egl"
    torch.backends.cudnn.allow_tf32 = True  # type: ignore  
    torch.backends.cudnn.benchmark = True  # type: ignore
    main()