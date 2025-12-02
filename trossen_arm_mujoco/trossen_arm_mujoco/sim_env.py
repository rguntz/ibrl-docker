# Copyright 2025 Trossen Robotics
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
#    * Redistributions of source code must retain the above copyright
#      notice, this list of conditions and the following disclaimer.
#
#    * Redistributions in binary form must reproduce the above copyright
#      notice, this list of conditions and the following disclaimer in the
#      documentation and/or other materials provided with the distribution.
#
#    * Neither the name of the copyright holder nor the names of its
#      contributors may be used to endorse or promote products derived from
#      this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

import collections

from dm_control.mujoco.engine import Physics
from dm_control.suite import base
import matplotlib.pyplot as plt
import numpy as np
import time


from trossen_arm_mujoco.constants import BOX_POSE, START_ARM_POSE
from trossen_arm_mujoco.utils import (
    get_observation_base,
    make_sim_env,
    plot_observation_images,
    sample_box_pose, 
)

import matplotlib.pyplot as plt
import numpy as np
import h5py
import numpy as np


class TrossenAIStationaryTask(base.Task):
    """
    A base task for bimanual manipulation with Trossen AI robotic arms in the Trossen AI Stationary Kit form factor.

    :param random: Random seed for environment variability, defaults to ``None``.
    :param onscreen_render: Whether to enable real-time rendering, defaults to ``False``.
    :param cam_list: List of cameras to capture observations, defaults to ``[]``.
    """

    def __init__(
        self,
        random: int | None = None,
        onscreen_render: bool = False,
        cam_list: list[str] = [],
    ):
        super().__init__(random=random)
        self.cam_list = cam_list
        if self.cam_list == []:
            self.cam_list = ["cam_high", "cam_low", "cam_left_wrist", "cam_right_wrist"]

    def before_step(self, action: np.ndarray, physics: Physics) -> None:
        """
        Processes the action before passing it to the simulation.

        :param action: The action array containing arm and gripper controls.
        :param physics: The MuJoCo physics simulation instance.
        """
        left_arm_action = action[:6]
        right_arm_action = action[8 : 8 + 6]
        normalized_left_gripper_action = action[6]
        normalized_right_gripper_action = action[8 + 6]

        # Assign the processed gripper actions
        left_gripper_action = normalized_left_gripper_action
        right_gripper_action = normalized_right_gripper_action

        # Ensure both gripper fingers act oppositely
        full_left_gripper_action = [left_gripper_action, left_gripper_action]
        full_right_gripper_action = [right_gripper_action, right_gripper_action]

        # Concatenate the final action array
        env_action = np.concatenate(
            [
                left_arm_action,
                full_left_gripper_action,
                right_arm_action,
                full_right_gripper_action,
            ]
        )
        self.counter += 1
        super().before_step(env_action, physics)

    def initialize_episode(self, physics: Physics) -> None:
        self.counter = 0
        """
        Sets the state of the environment at the start of each episode.

        :param physics: The MuJoCo physics simulation instance.
        """
        super().initialize_episode(physics)

    @staticmethod
    def get_env_state(physics: Physics) -> np.ndarray:
        """
        Retrieves the current state of the environment.

        :param physics: The MuJoCo physics simulation instance.
        :return: The environment state.
        """
        env_state = physics.data.qpos.copy()
        return env_state

    def get_position(self, physics: Physics) -> np.ndarray:
        """
        Retrieves the current joint positions.

        :param physics: The MuJoCo physics simulation instance.
        :return: The joint positions.
        """
        position = physics.data.qpos.copy()
        return position[:16]

    def get_velocity(self, physics: Physics) -> np.ndarray:
        """
        Retrieves the current joint velocities.

        :param physics: The MuJoCo physics simulation instance.
        :return: The joint velocities.
        """
        velocity = physics.data.qvel.copy()
        return velocity[:16]

    def get_observation(self, physics: Physics) -> collections.OrderedDict:
        """
        Collects the current observation from the environment.

        :param physics: The MuJoCo physics simulation instance.
        :return: An ordered dictionary containing joint positions, velocities, and environment state.
        """
        obs = get_observation_base(physics, self.cam_list)
        obs["qpos"] = self.get_position(physics)
        obs["qvel"] = self.get_velocity(physics)
        obs["env_state"] = self.get_env_state(physics)
        return obs

    def get_reward(self, physics: Physics) -> int:
        """
        Computes the reward for the current timestep.

        :param physics: The MuJoCo physics simulation instance.
        :raises NotImplementedError: This method must be implemented in subclasses.
        """
        # return whether left gripper is holding the box
        raise NotImplementedError


class TransferCubeTask(TrossenAIStationaryTask):
    """
    A task where a cube must be transferred between two robotic arms.

    :param random: Random seed for environment variability, defaults to ``None``.
    :param onscreen_render: Whether to enable real-time rendering, defaults to ``False``.
    :param cam_list: List of cameras to capture observations, defaults to ``[]``.
    """

    def __init__(
        self,
        random: int | None = None,
        onscreen_render: bool = False,
        cam_list: list[str] = [],
    ):
        super().__init__(
            random=random,
            onscreen_render=onscreen_render,
            cam_list=cam_list,
        )
        self.max_reward = 1

    def initialize_episode(self, physics: Physics) -> None:
        """
        Initializes the episode, resetting the robot's pose and cube position.

        :param physics: The MuJoCo physics simulation instance.
        """
        # TODO Notice: this function does not randomize the env configuration. Instead, set
        # BOX_POSE from outside reset qpos, control and box position
        with physics.reset_context():
            # Reset the arm pose
            physics.named.data.qpos[:16] = START_ARM_POSE

            # Randomize or sample the cube’s initial position
            cube_pose = sample_box_pose()
            box_start_idx = physics.model.name2id("red_box_joint", "joint")
            np.copyto(physics.data.qpos[box_start_idx : box_start_idx + 7], cube_pose)


        super().initialize_episode(physics)

    @staticmethod
    def get_env_state(physics: Physics) -> np.ndarray:
        """
        Retrieves the environment state related to the cube position.

        :param physics: The MuJoCo physics simulation instance.
        :return: The environment state.
        """
        env_state = physics.data.qpos.copy()[16:]
        return env_state

    def get_reward(self, physics: Physics) -> int:
        """
        Computes the reward based on whether the cube has been transferred successfully.

        :param physics: The MuJoCo physics simulation instance.
        :return: The computed reward which is whether left gripper is holding the box
        """
        all_contact_pairs = []
        for i_contact in range(physics.data.ncon):
            id_geom_1 = physics.data.contact[i_contact].geom1
            id_geom_2 = physics.data.contact[i_contact].geom2
            name_geom_1 = physics.model.id2name(id_geom_1, "geom")
            name_geom_2 = physics.model.id2name(id_geom_2, "geom")
            contact_pair = (name_geom_1, name_geom_2)
            all_contact_pairs.append(contact_pair)

        touch_right_gripper = (
            "red_box",
            "right/gripper_follower_left",
        ) in all_contact_pairs
        touch_blue_table = (
            "red_box",
            "table_box",
        ) in all_contact_pairs
        touch_table = ("red_box", "table") in all_contact_pairs

        reward = 0
        #if touch_right_gripper:
        #    reward = 1
        # lifted
        #if touch_right_gripper and not touch_table:
        #    reward = 2
        # attempted transfer
        #if touch_right_gripper and touch_blue_table: 
        #    return 3
        if touch_blue_table and not touch_right_gripper: 
            return 1
        return reward



def test_sim_teleop():
    """
    Runs a simulation to test teleoperation with the Trossen AI robotic arms.
    Testing teleoperation in sim with Trossen AI.
    Requires hardware and Trossen AI repo to work.
    """
    # setup the environment
    cam_list = ["cam_high", "cam_low", "cam_left_wrist", "cam_right_wrist"]
    env = make_sim_env(TransferCubeTask, "trossen_ai_scene_joint.xml", max_steps = 20)
    ts = env.reset()
    episode = [ts]
    # setup plotting
    plt_imgs = plot_observation_images(ts.observation, cam_list)

    time.sleep(19)



    for t in range(1000):
        action = np.random.uniform(-np.pi, np.pi, 16)
        action = np.zeros_like(action)
        action[6:8] = 0.044
        ts = env.step(action)
        episode.append(ts)

        plt_imgs[0].set_data(ts.observation["images"]["cam_high"])
        plt_imgs[1].set_data(ts.observation["images"]["cam_low"])
        plt_imgs[2].set_data(ts.observation["images"]["cam_left_wrist"])
        plt_imgs[3].set_data(ts.observation["images"]["cam_right_wrist"])

        plt.pause(0.02)






def load_demo_actions(hdf5_path, demo_name="demo_0"):
    """
    Loads the actions of a specific demo from an HDF5 dataset.
    """
    with h5py.File(hdf5_path, "r") as f:
        actions = f["data"][demo_name]["actions"][:]
    return actions

def test_sim_teleop_with_dataset(dataset_path, demo_name="demo_0"):
    """
    Runs a simulation using actions from an HDF5 dataset.
    """
    # Load actions
    actions = load_demo_actions(dataset_path, demo_name)
    
    # Setup environment
    cam_list = ["cam_high", "cam_low", "cam_left_wrist", "cam_right_wrist"]
    env = make_sim_env(TransferCubeTask, "trossen_ai_scene_joint.xml")
    ts = env.reset()
    episode = [ts]
    
    # Setup plotting
    plt_imgs = plot_observation_images(ts.observation, cam_list)

    for t in range(len(actions)):
        action = actions[t]
        ts = env.step(action)
        episode.append(ts)

        # Update images
        plt_imgs[0].set_data(ts.observation["images"]["cam_high"])
        plt_imgs[1].set_data(ts.observation["images"]["cam_low"])
        plt_imgs[2].set_data(ts.observation["images"]["cam_left_wrist"])
        plt_imgs[3].set_data(ts.observation["images"]["cam_right_wrist"])

        plt.pause(0.5)


import h5py
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.image import AxesImage

import h5py
import numpy as np

def load_demo_actions_and_obs(dataset_path, demo_name="demo_0"):
    """
    Loads actions and observations from the HDF5 dataset for a specific demo.
    
    Returns:
        actions: (T, 16) array of actions
        obs_dict: dict with keys 'cam_high', 'cam_low', 'cam_left_wrist', 'cam_right_wrist', 'prop'
                  images are (T, 128, 128, 3) after transposing channels
    """
    with h5py.File(dataset_path, "r") as f:
        demo = f[f"data/{demo_name}"]
        actions = demo["actions"][:]

        obs_group = demo["obs"]
        obs_dict = {
            "cam_high": np.transpose(obs_group["cam_high_image"][:], (0, 2, 3, 1)),
            "cam_low": np.transpose(obs_group["cam_low_image"][:], (0, 2, 3, 1)),
            "cam_left_wrist": np.transpose(obs_group["cam_left_wrist_image"][:], (0, 2, 3, 1)),
            "cam_right_wrist": np.transpose(obs_group["cam_right_wrist_image"][:], (0, 2, 3, 1)),
            "qpos": obs_group["qpos"][:], 
            "qvel": obs_group["qvel"][:]
        }
    return actions, obs_dict



def plotting_sim_teleop_with_dataset(dataset_path, demo_name="demo_0"):

    actions, dataset_obs = load_demo_actions_and_obs(dataset_path, demo_name)

    dataset_path_unorm = "/home/qtf5422/Desktop/AIRE/ibrl-docker/data/cube_picking_and_placing/dataset_200steps_actions16_delta_action.hdf5"
    unorm_action, _ = load_demo_actions_and_obs(dataset_path_unorm, demo_name)

    cam_list = ["cam_high", "cam_low", "cam_left_wrist", "cam_right_wrist"]
    env = make_sim_env(TransferCubeTask, "trossen_ai_scene_joint.xml")
    ts = env.reset()
    for i in range(100): 
        ts = env.step(dataset_obs["prop"][0, :16])
    episode = [ts]

    # Initialize dataset plot with the first timestep (just for AxesImage objects)
    plt.figure("Dataset Observations")
    dataset_imgs = plot_observation_images(
        {'images': {cam: dataset_obs[cam][0] for cam in cam_list}},  # just placeholder for init
        cam_list
    )

    plt.ion()
    plt.figure("Simulation Observations")
    sim_imgs = plot_observation_images(ts.observation, cam_list)

    for t in range(len(actions)):

        ## ---------------------------------
        ## Newly implemented : delta control

        joint_mins = np.array([-np.pi, 0, 0, -np.pi/2, -np.pi/2, -np.pi,
                                0, 0, -np.pi, 0, 0, -np.pi/2, -np.pi/2, -np.pi,
                                0, 0])
        joint_maxs = np.array([np.pi, np.pi, 2.36, np.pi/2, np.pi/2, np.pi,
                            0.04, 0.04, np.pi, np.pi, 2.36, np.pi/2, np.pi/2, np.pi,
                            0.04, 0.04])

        # 2. model output (joint delta)
        model_delta = ((actions[t]  + 1) / 2) * (joint_maxs - joint_mins) + joint_mins # we need to denormalize the action because the current one is between -1 and 1.

        # 4. compute next absolute positions
        next_qpos = ts.observation["qpos"] + model_delta
        next_qpos = dataset_obs["prop"][t, :16] + model_delta

        # 5. clip to joint limits
        next_qpos = np.clip(next_qpos, joint_mins, joint_maxs)

        ## ---------------------------------

        ts = env.step(next_qpos)
        episode.append(ts)

        # Update simulation images
        for i, cam in enumerate(cam_list):
            sim_imgs[i].set_data(ts.observation["images"][cam])

        # Update dataset images
        for i, cam in enumerate(cam_list):
            dataset_imgs[i].set_data(dataset_obs[cam][t])  # timestep t


        plt.pause(0.5)


    

    plt.show()





def test_sim_teleop_with_dataset_joint_control(dataset_path, demo_name="demo_0"):
    actions, dataset_obs = load_demo_actions_and_obs(dataset_path, demo_name)

    cam_list = ["cam_high", "cam_low", "cam_left_wrist", "cam_right_wrist"]
    env = make_sim_env(TransferCubeTask, "trossen_ai_scene_joint.xml")
    ts = env.reset()
    episode = [ts]

    # Initialize dataset plot with the first timestep (just for AxesImage objects)
    plt.figure("Dataset Observations")
    dataset_imgs = plot_observation_images(
        {'images': {cam: dataset_obs[cam][0] for cam in cam_list}},  # just placeholder for init
        cam_list
    )

    plt.ion()
    plt.figure("Simulation Observations")
    sim_imgs = plot_observation_images(ts.observation, cam_list)

    for t in range(len(actions)):
        # Denormalize and step
        action = actions[t]

        joint_mins = np.array([-np.pi, 0, 0, -np.pi/2, -np.pi/2, -np.pi,
                                0, 0, -np.pi, 0, 0, -np.pi/2, -np.pi/2, -np.pi,
                                0, 0])
        joint_maxs = np.array([np.pi, np.pi, 2.36, np.pi/2, np.pi/2, np.pi,
                            0.04, 0.04, np.pi, np.pi, 2.36, np.pi/2, np.pi/2, np.pi,
                            0.04, 0.04])
            
        unnormalized_action = ((action + 1) / 2) * (joint_maxs - joint_mins) + joint_mins # we need to denormalize the action because the current one is between -1 and 1. 
        ts = env.step(unnormalized_action)

        episode.append(ts)

        # Update simulation images
        for i, cam in enumerate(cam_list):
            sim_imgs[i].set_data(ts.observation["images"][cam])

        # Update dataset images
        for i, cam in enumerate(cam_list):
            dataset_imgs[i].set_data(dataset_obs[cam][t])  # timestep t

        print("prop sim :      ", ts.observation["qpos"], ts.observation["qvel"])
        print("prop recorded : ", dataset_obs["qpos"][t])

        plt.pause(0.5)


    

    plt.show()





if __name__ == "__main__":
    test_sim_teleop()
    dataset_path = "/home/qtf5422/Desktop/AIRE/ibrl-docker/data/cube_picking_and_placing/tresholded_reward/extended_gripper_dataset_wr_threshold_rewards_dones_shifted_gripper_clipped_clipped_norm_min_max_cut.hdf5"

    #test_sim_teleop()


    for i in range(50) : 
        demo_name = f"demo_{i}"
        test_sim_teleop_with_dataset_joint_control(dataset_path, demo_name)
