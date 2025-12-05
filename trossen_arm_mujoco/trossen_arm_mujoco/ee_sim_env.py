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

from dm_control.mujoco.engine import Physics
from dm_control.suite import base
import numpy as np
import time
import h5py
import numpy as np
import matplotlib.pyplot as plt

from trossen_arm_mujoco.constants import START_ARM_POSE
from trossen_arm_mujoco.utils import (
    get_observation_base,
    make_sim_env,
    plot_observation_images,
    sample_box_pose,
    set_observation_images,
)


class TrossenAIStationaryEETask(base.Task):
    """
    A base task for bimanual Cartesian manipulation with Trossen AI robotic arms in the Trossen AI
    Stationary Kit form factor.

    :param random: Randomization seed for environment initialization, defaults to ``None``.
    :param onscreen_render: Whether to enable on-screen rendering, defaults to ``False``.
    :param cam_list: List of cameras for observation capture, defaults to ``[]``.
    """

    def __init__(
        self,
        random: int | None = None,
        onscreen_render=False,
        cam_list: list[str] = [],
    ):
        super().__init__(random=random)
        self.cam_list = cam_list
        if self.cam_list == []:
            self.cam_list = ["cam_high", "cam_low", "cam_left_wrist", "cam_right_wrist"]

    def before_step(self, action: np.ndarray, physics: Physics) -> None:
        """
        Apply the action to the robotic arms before stepping the simulation.

        :param action: The action vector containing position and gripper commands.
        :param physics: The simulation physics instance.
        """
        a_len = len(action) // 2
        action_left = action[:a_len]
        action_right = action[a_len:]

        # set mocap position and quat
        # left
        np.copyto(physics.data.mocap_pos[0], action_left[:3])
        np.copyto(physics.data.mocap_quat[0], action_left[3:7])
        # right
        np.copyto(physics.data.mocap_pos[1], action_right[:3])
        np.copyto(physics.data.mocap_quat[1], action_right[3:7])

        # physics.data.qpos[6] = action_left[7] # gripper opening and closing position left one
        # physics.data.qpos[7] = action_left[7] # symetrical opening and closing for gripper left one
        # physics.data.qpos[14] = action_right[7] # right one
        # physics.data.qpos[15] = action_right[7] # right one

        # Use actuators instead of direct position control
        physics.data.ctrl[0] = action_left[7]  # left gripper motor
        physics.data.ctrl[1] = action_right[7] # right gripper motor

    def initialize_robots(self, physics: Physics) -> None:
        """
        Initialize the robots by resetting joint positions and aligning mocap bodies with end-effectors.

        :param physics: The simulation physics engine.
        """
        # reset joint position
        physics.named.data.qpos[:12] = START_ARM_POSE[:6] + START_ARM_POSE[8:14]

        # reset mocap to align with end effector
        #np.copyto(physics.data.mocap_pos[0], [-0.19657, -0.019, 0.25021])
        np.copyto(physics.data.mocap_pos[0], [-2.04248170e-01, -1.90390477e-02, 1.88026731e-01])
        np.copyto(physics.data.mocap_quat[0], [1, 0, 0, 0])
        # right
        #np.copyto(physics.data.mocap_pos[1], [0.19657, -0.019, 0.25021])
        np.copyto(physics.data.mocap_pos[1], [2.05969129e-01, -1.97438376e-02, 1.88026731e-01])
        np.copyto(physics.data.mocap_quat[1], [1, 0, 0, 0])

    def initialize_episode(self, physics: Physics):
        """
        Set up the environment state at the beginning of each episode.

        :param physics: The simulation physics engine.
        """
        """Sets the state of the environment at the start of each episode."""
        super().initialize_episode(physics)

    @staticmethod
    def get_env_state(physics: Physics) -> np.ndarray:
        """
        Retrieve the environment state.

        :param physics: The simulation physics engine.
        :raises NotImplementedError: This function must be implemented in derived classes.
        """
        raise NotImplementedError

    def get_position(self, physics: Physics) -> np.ndarray:
        """
        Get the current joint positions of the robot.

        :param physics: The simulation physics engine.
        :return: The joint positions.
        """
        positions = physics.data.qpos.copy()
        return positions[:16]

    def get_velocity(self, physics: Physics) -> np.ndarray:
        """
        Get the current joint velocities of the robot.

        :param physics: The simulation physics engine.
        :return: The joint velocities.
        """
        velocities = physics.data.qvel.copy()
        return velocities[:16]

    def get_observation(self, physics: Physics) -> dict:

        """
        Retrieve the robot's observation data, including joint positions, velocities, and camera images.

        :param physics: The simulation physics engine.
        :return: The current observation state.
        """
        obs = get_observation_base(physics, self.cam_list)
        obs["qpos"] = self.get_position(physics)
        obs["qvel"] = self.get_velocity(physics)
        obs["env_state"] = self.get_env_state(physics)
        
        obs["mocap_pose_left"] = np.concatenate(
            [physics.data.mocap_pos[0], physics.data.mocap_quat[0]]
        ).copy()
        obs["mocap_pose_right"] = np.concatenate(
            [physics.data.mocap_pos[1], physics.data.mocap_quat[1]]
        ).copy()
        obs["gripper_ctrl"] = physics.data.ctrl.copy()

        # Add the features for ibrl : 
        obs["robot0_eef_pos"] = np.concatenate([obs["mocap_pose_left"][:3],obs["mocap_pose_right"][:3]])
        obs["robot0_eef_quat"] = np.concatenate([obs["mocap_pose_left"][3:],obs["mocap_pose_right"][3:]])    
        #obs["robot0_gripper_qpos"] = np.concatenate([obs["qpos"][6:8],obs["qpos"][-2:]])  FOR 4 GRIPPERS. 
        obs["robot0_gripper_qpos"] = np.concatenate([obs["qpos"][6:7],obs["qpos"][-2:-1]])  #FOR 2 GRIPPERS. 

        return obs

    def get_reward(self, physics: Physics) -> int:
        """
        Compute the task-specific reward.

        :param physics: The simulation physics engine.
        :raises NotImplementedError: This function must be implemented in derived classes.
        """
        raise NotImplementedError


class TransferCubeEETask(TrossenAIStationaryEETask):
    """
    A task where a cube must be transferred between two robotic arms.

    :param random: Random seed for environment variability, defaults to ``None``.
    :param onscreen_render: Whether to enable real-time rendering, defaults to ``False``.
    :param cam_list: List of cameras to capture observations, defaults to ``None``.
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
        Set up the simulation environment at the start of an episode.

        :param physics: The simulation physics engine.
        """
        self.initialize_robots(physics)
        # randomize box position
        cube_pose = sample_box_pose()
        box_start_idx = physics.model.name2id("red_box_joint", "joint")
        np.copyto(physics.data.qpos[box_start_idx : box_start_idx + 7], cube_pose)

        super().initialize_episode(physics)

    @staticmethod
    def get_env_state(physics: Physics) -> np.ndarray:
        """
        Retrieve the environment state specific to this task.

        :param physics: The simulation physics engine.
        :return: The state of the environment.
        """
        env_state = physics.data.qpos.copy()[16:]
        return env_state

    def get_reward(self, physics: Physics) -> int:
        """
        Computes the reward based on whether the cube is on the table and not held by the right gripper.
        
        :param physics: The MuJoCo physics simulation instance.
        :return: 1 if cube is on table and not gripped, else 0.
        """
        # Define all collision geom names that belong to the red cube
        RED_CUBE_GEOMS = {"subcube1", "subcube2", "subcube3", "subcube4"}
        TABLE_GEOM = "table_box"
        GRIPPER_GEOM = "right/gripper_follower_left"

        # Build set of contact pairs (as unordered tuples for robust matching)
        contact_pairs = set()
        for i in range(physics.data.ncon):
            g1_id = physics.data.contact[i].geom1
            g2_id = physics.data.contact[i].geom2
            g1_name = physics.model.id2name(g1_id, "geom")
            g2_name = physics.model.id2name(g2_id, "geom")
            if g1_name and g2_name:  # skip unnamed geoms
                contact_pairs.add((g1_name, g2_name))
                contact_pairs.add((g2_name, g1_name))  # make order-agnostic

        # Check if ANY red cube geom touches the table
        touch_blue_table = any(
            (cube_geom, TABLE_GEOM) in contact_pairs
            for cube_geom in RED_CUBE_GEOMS
        )

        # Check if ANY red cube geom touches the right gripper
        touch_right_gripper = any(
            (cube_geom, GRIPPER_GEOM) in contact_pairs
            for cube_geom in RED_CUBE_GEOMS
        )

        obs = self.get_observation(physics)
        env_state = obs["env_state"]
        z_position = env_state[2]

        if touch_blue_table and not touch_right_gripper and z_position > 0.17:
            return 1
        return 0


def test_ee_sim_env():
    onscreen_render = True
    cam_list = ["cam_high", "cam_low", "cam_left_wrist", "cam_right_wrist"]
    env = make_sim_env(
        TransferCubeEETask,
        task_name="sim_transfer_cube",
        onscreen_render=onscreen_render,
        cam_list=cam_list,
    )
    action_spec = env.action_spec()
    print("action_spec is : ",action_spec)
    ts = env.reset()
    print("size of image from sim : ", ts.observation["images"]["cam_high"].shape)
    episode = [ts]
    # setup plotting
    if onscreen_render:
        plt_imgs = plot_observation_images(ts.observation, cam_list)

    for t in range(1000):
        action = np.random.uniform(-0.1, 0.1, 16)
        ts = env.step(action)
        episode.append(ts)
        if onscreen_render:
            plt_imgs = set_observation_images(ts.observation, plt_imgs, cam_list)
        


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
            "qvel": obs_group["qvel"][:],
            "robot0_eef_pos" : obs_group["robot0_eef_pos"][:], 
            "robot0_eef_quat" : obs_group["robot0_eef_quat"][:], 
            "robot0_gripper_qpos" : obs_group["robot0_gripper_qpos"][:]
        }
    return actions, obs_dict


def modif_action(action_step) : 
    # Since the action is only 8 dimensional we need to include the quaternions now : 
            # set the quaternion of both to [1, 0, 0, 0] : 
    theta = np.deg2rad(-20)  # negative = pitch down
    half_theta = theta / 2
    pitch_quat = [
        np.cos(half_theta),        # w
        0,                         # x (axis x = 0)
        np.sin(half_theta),        # y (axis y = 1)
        0                          # z (axis z = 0)
    ]

    left_quat = [1, 0, 0, 0]
    right_quat = pitch_quat

    action_step = np.concatenate([
        action_step[:3],
        left_quat,
        np.array([action_step[3]]),
        action_step[4:7],
        right_quat,
        np.array([action_step[7]])
    ])
    return action_step

def plotting_sim_teleop_with_dataset(dataset_path = "/home/qtf5422/Desktop/AIRE/ibrl-docker/data/cube_picking_and_placing_ee_pos/new_mujoco/tresholding/dataset_tresholded_wr_shifted_norm_gripper.hdf5", demo_name="demo_0"):

    for demo_number in range(50) : 
        demo_name = f"demo_{demo_number}"
        actions, dataset_obs = load_demo_actions_and_obs(dataset_path, demo_name)

        cam_list = ["cam_high", "cam_low", "cam_left_wrist", "cam_right_wrist"]
        env = make_sim_env(TransferCubeEETask, "trossen_ai_scene.xml")
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

    ##################################################################################################
            action_step = modif_action(actions[t])
    ##################################################################################################

            ## ---------------------------------
            ts = env.step(action_step)
            episode.append(ts)

            # Update simulation images
            for i, cam in enumerate(cam_list):
                sim_imgs[i].set_data(ts.observation["images"][cam])

            # Update dataset images
            for i, cam in enumerate(cam_list):
                dataset_imgs[i].set_data(dataset_obs[cam][t])  # timestep t


            plt.pause(0.01)

        plt.show()




if __name__ == "__main__":
    plotting_sim_teleop_with_dataset(dataset_path = "/home/qtf5422/Desktop/AIRE/ibrl-docker/data/cube_picking_and_placing_ee_pos/new_mujoco/dataset.hdf5", demo_name="demo_0")