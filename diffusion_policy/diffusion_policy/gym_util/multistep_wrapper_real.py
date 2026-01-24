import gym
from gym import spaces
import numpy as np
from collections import defaultdict, deque
import dill

from trossen_arm_mujoco.ee_sim_env import plot_observation_images, set_observation_images


def stack_repeated(x, n):
    return np.repeat(np.expand_dims(x,axis=0),n,axis=0)

def repeated_box(box_space, n):
    return spaces.Box(
        low=stack_repeated(box_space.low, n),
        high=stack_repeated(box_space.high, n),
        shape=(n,) + box_space.shape,
        dtype=box_space.dtype
    )

def repeated_space(space, n):
    if isinstance(space, spaces.Box):
        return repeated_box(space, n)
    elif isinstance(space, spaces.Dict):
        result_space = spaces.Dict()
        for key, value in space.items():
            result_space[key] = repeated_space(value, n)
        return result_space
    else:
        raise RuntimeError(f'Unsupported space type {type(space)}')

def take_last_n(x, n):
    x = list(x)
    n = min(len(x), n)
    return np.array(x[-n:])

def dict_take_last_n(x, n):
    result = dict()
    for key, value in x.items():
        result[key] = take_last_n(value, n)
    return result

def aggregate(data, method='max'):
    if method == 'max':
        # equivalent to any
        return np.max(data)
    elif method == 'min':
        # equivalent to all
        return np.min(data)
    elif method == 'mean':
        return np.mean(data)
    elif method == 'sum':
        return np.sum(data)
    else:
        raise NotImplementedError()

def stack_last_n_obs(all_obs, n_steps):
    assert(len(all_obs) > 0)
    all_obs = list(all_obs)
    result = np.zeros((n_steps,) + all_obs[-1].shape, 
        dtype=all_obs[-1].dtype)
    start_idx = -min(n_steps, len(all_obs))
    result[start_idx:] = np.array(all_obs[start_idx:])
    if n_steps > len(all_obs):
        # pad
        result[:start_idx] = result[start_idx]
    return result


class MultiStepWrapper(gym.Wrapper):
    def __init__(self, 
            env, 
            n_obs_steps, 
            n_action_steps, 
            max_episode_steps=None,
            reward_agg_method='max'
        ):
        super().__init__(env)
        self._action_space = repeated_space(env.action_space, n_action_steps)
        self._observation_space = repeated_space(env.observation_space, n_obs_steps)
        self.max_episode_steps = max_episode_steps
        self.n_obs_steps = n_obs_steps
        self.n_action_steps = n_action_steps
        self.reward_agg_method = reward_agg_method
        self.n_obs_steps = n_obs_steps

        self.obs = deque(maxlen=n_obs_steps+1)
        self.reward = list()
        self.done = list()
        self.info = defaultdict(lambda : deque(maxlen=n_obs_steps+1))
        
        # Initialize counter for tracking cube outside zone
        self.outside_counter = 0
        self.OUTSIDE_THRESHOLD = 5  # Number of consecutive checks before terminating episode
    
    def _get_physics(self):
        """
        Navigate through wrapper chain to access physics object.
        
        The wrapper structure is:
        - self: MultiStepWrapper
        - self.env: VideoRecordingWrapper
        - self.env.env: TrossenImageWrapper
        - self.env.env.env: dm_control environment (has .physics)
        
        Returns:
            Physics object from dm_control
        """
        # return self.env.env.env.physics
        return self.env.env.physics
        
    def cube_outside_initial_box(self):
        terminal = False

        # Get physics through wrapper chain
        physics = self._get_physics()

        # Define all collision geom names that belong to the red cube
        RED_CUBE_GEOMS = {"subcube1", "subcube2", "subcube3", "subcube4"}
        GRIPPER_GEOM = "right/gripper_follower_left"

        # Build set of contact pairs
        contact_pairs = set()
        for i in range(physics.data.ncon):
            g1_id = physics.data.contact[i].geom1
            g2_id = physics.data.contact[i].geom2
            g1_name = physics.model.id2name(g1_id, "geom")
            g2_name = physics.model.id2name(g2_id, "geom")
            if g1_name and g2_name:
                contact_pairs.add((g1_name, g2_name))
                contact_pairs.add((g2_name, g1_name))

        # Check if ANY red cube geom touches the right gripper
        touch_right_gripper = any(
            (cube_geom, GRIPPER_GEOM) in contact_pairs
            for cube_geom in RED_CUBE_GEOMS
        )

        # obs = self.env.env.env.task.get_observation(physics)
        obs = self.env.env.task.get_observation(physics)
        env_state = obs["env_state"]
        cube_x_y = env_state[:3]

        x_range = [-0.1, 0.2]
        y_range = [-0.15, 0.025]

        # Check if cube is outside allowed zone and not touching gripper
        outside_zone = (
            (cube_x_y[0] < x_range[0] or cube_x_y[0] > x_range[1]) or
            (cube_x_y[1] < y_range[0] or cube_x_y[1] > y_range[1])
        ) and not touch_right_gripper and cube_x_y[2] <= 0.15

        if outside_zone:
            self.outside_counter += 1
        else:
            # Reset counter if cube comes back inside
            self.outside_counter = 0

        # Trigger terminal only if outside for 5 consecutive checks
        if self.outside_counter >= self.OUTSIDE_THRESHOLD:
            terminal = True
            self.outside_counter = 0  # reset counter after triggering

        return terminal
    
    def reset(self):
        """Resets the environment using kwargs."""
        obs = super().reset()

        self.obs = deque([obs], maxlen=self.n_obs_steps+1)
        self.reward = list()
        self.done = list()
        self.info = defaultdict(lambda : deque(maxlen=self.n_obs_steps+1))

        obs = self._get_obs(self.n_obs_steps)

        return obs

    def step(self, action):
        """
        actions: (n_action_steps,) + action_shape
        """
        # action = action.squeeze(0)

        for act in action:
            if len(self.done) > 0 and self.done[-1]:
                # termination
                break
            observation, reward, done, info = super().step(act)

            if reward == 1 : 
                done = True

            self.obs.append(observation)
            self.reward.append(reward)
            if (self.max_episode_steps is not None) \
                and (len(self.reward) >= self.max_episode_steps):
                # truncation
                done = True
            self.done.append(done)
            self._add_info(info)

            if done : 
                # self.env.env.close_plot()
                self.env.close_plot()

        observation = self._get_obs(self.n_obs_steps)
        reward = aggregate(self.reward, self.reward_agg_method)
        done = aggregate(self.done, 'max')
        info = dict_take_last_n(self.info, self.n_obs_steps)
        return observation, reward, done, info

    def _get_obs(self, n_steps=1):
        """
        Output (n_steps,) + obs_shape
        """
        assert(len(self.obs) > 0)
        if isinstance(self.observation_space, spaces.Box):
            return stack_last_n_obs(self.obs, n_steps)
        elif isinstance(self.observation_space, spaces.Dict):
            result = dict()
            for key in self.observation_space.keys():
                result[key] = stack_last_n_obs(
                    [obs[key] for obs in self.obs],
                    n_steps
                )
            return result
        else:
            raise RuntimeError('Unsupported space type')

    def _add_info(self, info):
        for key, value in info.items():
            self.info[key].append(value)
    
    def get_rewards(self):
        return self.reward
    
    def get_attr(self, name):
        return getattr(self, name)

    def run_dill_function(self, dill_fn):
        fn = dill.loads(dill_fn)
        return fn(self)
    
    def get_infos(self):
        result = dict()
        for k, v in self.info.items():
            result[k] = list(v)
        return result
