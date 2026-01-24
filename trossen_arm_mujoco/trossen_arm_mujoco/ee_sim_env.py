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
from scipy.spatial.transform import Rotation as R

from trossen_arm_mujoco.constants import START_ARM_POSE
from trossen_arm_mujoco.utils import (
    get_observation_base,
    make_sim_env,
    plot_observation_images,
    sample_box_pose,
    set_observation_images,
)

USE_AXIS_ANGLE = False
USE_DELTA = True

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

    def quat_to_angle_axis(self, quats):
        """
        Convert quaternions [w, x, y, z] to angle-axis (rotvec).
        Input: (N, 4) — w first (MuJoCo convention)
        Output: (N, 3)
        """
        if quats.ndim != 2 or quats.shape[-1] != 4:
            raise ValueError(f"Expected shape (N, 4), got {quats.shape}")
        # Convert to SciPy order: [x, y, z, w]
        quats_xyzw = np.stack([quats[:, 1], quats[:, 2], quats[:, 3], quats[:, 0]], axis=-1)
        rot = R.from_quat(quats_xyzw)
        return rot.as_rotvec()  # (N, 3)

    def get_observation(self, physics: Physics) -> dict:
        """
        Retrieve the robot's observation data, including joint positions, velocities, camera images,
        end-effector poses (as both quat and angle-axis), and gripper states.

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

        # --- Original IBRL-style keys (quaternion-based) ---
        obs["robot0_eef_pos"] = np.concatenate([obs["mocap_pose_left"][:3], obs["mocap_pose_right"][:3]])      # (6,)
        obs["robot0_eef_quat"] = np.concatenate([obs["mocap_pose_left"][3:], obs["mocap_pose_right"][3:]])    # (8,)

        # --- Gripper positions (2-DoF total: 1 per arm) ---
        obs["robot0_gripper_qpos"] = np.concatenate([obs["qpos"][6:7], obs["qpos"][-2:-1]])  # (2,)

        # --- ✅ ADD angle-axis representation ---
        # Convert each arm's quat [w,x,y,z] → angle-axis (3D)
        left_quat = obs["mocap_pose_left"][3:]   # (4,) — [w, x, y, z]
        right_quat = obs["mocap_pose_right"][3:] # (4,)

        left_aa = self.quat_to_angle_axis(left_quat[None, :])[0]
        right_aa = self.quat_to_angle_axis(right_quat[None, :])[0]

        obs["robot0_eef_aa"] = np.concatenate([left_aa, right_aa])  # (6,)

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


class TransferCubeEETask_dexterous(TrossenAIStationaryEETask):
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
        self.max_reward = 3
        self.reward = 0

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
        TABLE_BOX_SMALL = "table_box_small"
        GRIPPER_GEOM_RIGHT = "right/gripper_follower_left"
        GRIPPER_GEOM_LEFT = "left/gripper_follower_left"

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

        cube_center = np.array([0.01354977, -0.00898126, 0.39927967])
        cube_size = 0.1
        r = cube_size / 2

        # Check if ANY red cube geom touches the table
        touch_blue_table = any(
            (cube_geom, TABLE_GEOM) in contact_pairs
            for cube_geom in RED_CUBE_GEOMS
        )

        # Check if ANY red cube geom touches the table
        touch_small_table = any(
            (cube_geom, TABLE_BOX_SMALL) in contact_pairs
            for cube_geom in RED_CUBE_GEOMS
        )

        # Check if ANY red cube geom touches the right gripper
        touch_right_gripper = any(
            (cube_geom, GRIPPER_GEOM_RIGHT) in contact_pairs
            for cube_geom in RED_CUBE_GEOMS
        )

        touch_left_gripper = any(
            (cube_geom, GRIPPER_GEOM_LEFT) in contact_pairs
            for cube_geom in RED_CUBE_GEOMS
        )

        obs = self.get_observation(physics)
        env_state = obs["env_state"]
        z_pos_cubs = env_state[2]
        cartesian = obs["robot0_eef_pos"][3:]
        inside_cube = np.all(
            np.abs(cartesian - cube_center) <= r
        )

        if touch_blue_table and not touch_left_gripper and z_pos_cubs > 0.1 : 
            reward = 2
            self.reward = max(self.reward, reward)
            return reward
        if touch_small_table and not touch_right_gripper and z_pos_cubs > 0.01 : 
            reward = 3
            self.reward = max(self.reward, reward)
            return reward
        if inside_cube and touch_right_gripper : 
            reward =  1
            self.reward = max(self.reward, reward)
            return reward
        
        return 0

def sample_random_action():
    # === Left arm EE position: within LEFT arm workspace ===
    left_x = np.random.uniform(-1.1575, 0.2425)   # x ∈ [base_x - 0.7, base_x + 0.7]
    left_y = np.random.uniform(-0.719, 0.681)     # y ∈ [base_y - 0.7, base_y + 0.7]
    left_z = np.random.uniform(0.0, 0.72)         # z ∈ [0, base_z + 0.7]

    # === Right arm EE position: within RIGHT arm workspace ===
    right_x = np.random.uniform(-0.2425, 1.1575)
    right_y = np.random.uniform(-0.719, 0.681)    # same y range (same base y)
    right_z = np.random.uniform(0.0, 0.72)

    # === Quaternions: as per your request, sample in [-0.1, 0.1]
    # (Note: these are NOT unit quaternions! Only use if env treats them as deltas)
    left_quat = np.random.uniform(-0.1, 0.1, 4)
    right_quat = np.random.uniform(-0.1, 0.1, 4)

    # === Grippers: keep in [-0.1, 0.1]
    left_gripper = np.random.uniform(-0.1, 0.1)
    right_gripper = np.random.uniform(-0.1, 0.1)

    # === Assemble action ===
    action = np.array([
        left_x, left_y, left_z,
        *left_quat,
        left_gripper,
        right_x, right_y, right_z,
        *right_quat,
        right_gripper
    ], dtype=np.float32)

    return action

def test_ee_sim_env():
    onscreen_render = True
    cam_list = ["cam_high", "cam_low", "cam_left_wrist", "cam_right_wrist"]
    env = make_sim_env(
        TransferCubeEETask,
        task_name="sim_transfer_cube",
        onscreen_render=onscreen_render,
        cam_list=cam_list,
        xml_file = "trossen_ai_scene_task_2.xml", 
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
        action = np.random.uniform(low=-0.1, high=0.1, size=16)

        # Step 2: override specific indices with values in [-0.1, 1]
        action[2] = np.random.uniform(-0.1, 1)
        action[8] = np.random.uniform(-0.1, 1)
        
        ts = env.step(action)
        print("ts obs quat : ", ts.observation["robot0_eef_quat"])
        print("ts obs pos : ", ts.observation["robot0_eef_pos"])
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

        quat_group = None

        if USE_AXIS_ANGLE : 
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
                "robot0_gripper_qpos" : obs_group["robot0_gripper_qpos"][:],
                "robot0_eef_aa" : obs_group["robot0_eef_aa"][:]
            }
        elif USE_DELTA : 
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
            quat_group = demo["action_quat"][:]
    return actions, obs_dict, quat_group

def action_aa_to_quat(action_aa: np.ndarray) -> np.ndarray:
    """
    Convert a single 14D action (angle-axis) to 16D action (quaternion).

    Input:
        action_aa: np.ndarray of shape (14,)
                [l_pos(3), l_aa(3), l_grip(1), r_pos(3), r_aa(3), r_grip(1)]

    Output:
        action_quat: np.ndarray of shape (16,)
                    [l_pos(3), l_quat(4), l_grip(1), r_pos(3), r_quat(4), r_grip(1)]
                    Quaternions in [w, x, y, z] order (MuJoCo convention)
    """
    if action_aa.shape != (14,):
        raise ValueError(f"Expected shape (14,), got {action_aa.shape}")

    # Unpack
    l_pos = action_aa[0:3]
    l_aa = action_aa[3:6]
    l_grip = action_aa[6:7]

    r_pos = action_aa[7:10]
    r_aa = action_aa[10:13]
    r_grip = action_aa[13:14]

    # Convert angle-axis → rotation → quat (SciPy: [x,y,z,w])
    l_quat_xyzw = R.from_rotvec(l_aa).as_quat()   # (4,)
    r_quat_xyzw = R.from_rotvec(r_aa).as_quat()   # (4,)

    # Reorder to MuJoCo convention: [w, x, y, z]
    l_quat_wxyz = np.array([l_quat_xyzw[3], l_quat_xyzw[0], l_quat_xyzw[1], l_quat_xyzw[2]])
    r_quat_wxyz = np.array([r_quat_xyzw[3], r_quat_xyzw[0], r_quat_xyzw[1], r_quat_xyzw[2]])

    # Assemble 16D action
    action_quat = np.concatenate([
        l_pos, l_quat_wxyz, l_grip,
        r_pos, r_quat_wxyz, r_grip
    ])

    return action_quat  # (16,)


def denormalize_action_deltas(action_normalized: np.ndarray) -> np.ndarray:
    """
    Denormalize delta position, rotation, and gripper from [-1, 1] to original ranges.
    
    Formula: a_raw = (a_norm + 1) * (max - min) / 2 + min
    
    Args:
        action_normalized: (14,) array [pos_L_3d, rot_L_3d, grip_L_1d, pos_R_3d, rot_R_3d, grip_R_1d]
    
    Returns:
        action_denorm: (14,) array with denormalized position, rotation, and gripper values
    """
    import json
    from pathlib import Path
    
    if action_normalized.shape != (14,):
        raise ValueError(f"Expected shape (14,), got {action_normalized.shape}")
    
    action_denorm = action_normalized.copy()
    
    # === Load stats from JSON file ===
    stats_file = Path("/home/qtf5422/Desktop/AIRE/ibrl-docker/data/cube_picking_and_placing_ee/delta/delta_action_stats.json")
    with open(stats_file, 'r') as f:
        stats = json.load(f)
    
    # Extract Position/Rotation Delta Min/Max from JSON
    pos_x_min, pos_x_max = stats["Pos X Delta"]["min"], stats["Pos X Delta"]["max"]
    pos_y_min, pos_y_max = stats["Pos Y Delta"]["min"], stats["Pos Y Delta"]["max"]
    pos_z_min, pos_z_max = stats["Pos Z Delta"]["min"], stats["Pos Z Delta"]["max"]
    
    rot_x_min, rot_x_max = stats["Rot X Delta"]["min"], stats["Rot X Delta"]["max"]
    rot_y_min, rot_y_max = stats["Rot Y Delta"]["min"], stats["Rot Y Delta"]["max"]
    rot_z_min, rot_z_max = stats["Rot Z Delta"]["min"], stats["Rot Z Delta"]["max"]
    
    # === Gripper Min/Max (normalized with min=0, max=0.04) ===
    grip_min, grip_max = 0.0, 0.04
    
    # Denormalization formula: a_raw = (a_norm + 1) * (max - min) / 2 + min
    
    # --- LEFT ARM ---
    # Position deltas (indices 0, 1, 2)
    action_denorm[0] = (action_normalized[0] + 1.0) * (pos_x_max - pos_x_min) / 2.0 + pos_x_min
    action_denorm[1] = (action_normalized[1] + 1.0) * (pos_y_max - pos_y_min) / 2.0 + pos_y_min
    action_denorm[2] = (action_normalized[2] + 1.0) * (pos_z_max - pos_z_min) / 2.0 + pos_z_min
    
    # Rotation deltas (indices 3, 4, 5)
    action_denorm[3] = (action_normalized[3] + 1.0) * (rot_x_max - rot_x_min) / 2.0 + rot_x_min
    action_denorm[4] = (action_normalized[4] + 1.0) * (rot_y_max - rot_y_min) / 2.0 + rot_y_min
    action_denorm[5] = (action_normalized[5] + 1.0) * (rot_z_max - rot_z_min) / 2.0 + rot_z_min
    
    # Gripper (index 6) - denormalize to [0, 0.04]
    action_denorm[6] = (action_normalized[6] + 1.0) * (grip_max - grip_min) / 2.0 + grip_min
    
    # --- RIGHT ARM ---
    # Position deltas (indices 7, 8, 9)
    action_denorm[7] = (action_normalized[7] + 1.0) * (pos_x_max - pos_x_min) / 2.0 + pos_x_min
    action_denorm[8] = (action_normalized[8] + 1.0) * (pos_y_max - pos_y_min) / 2.0 + pos_y_min
    action_denorm[9] = (action_normalized[9] + 1.0) * (pos_z_max - pos_z_min) / 2.0 + pos_z_min
    
    # Rotation deltas (indices 10, 11, 12)
    action_denorm[10] = (action_normalized[10] + 1.0) * (rot_x_max - rot_x_min) / 2.0 + rot_x_min
    action_denorm[11] = (action_normalized[11] + 1.0) * (rot_y_max - rot_y_min) / 2.0 + rot_y_min
    action_denorm[12] = (action_normalized[12] + 1.0) * (rot_z_max - rot_z_min) / 2.0 + rot_z_min
    
    # Gripper (index 13) - denormalize to [0, 0.04]
    action_denorm[13] = (action_normalized[13] + 1.0) * (grip_max - grip_min) / 2.0 + grip_min
    
    return action_denorm


def delta_to_absolute_action(action_delta: np.ndarray, obs_current: dict, dataset_obs, t) -> np.ndarray:
    """
    Convert 14D delta action to 16D absolute action for simulator.
    
    Converts:
    - Delta positions to absolute positions: pos_target = pos_current + delta_pos
    - Delta rotations to absolute quaternions: applies delta rotation to current quaternion
    - Keeps gripper absolute (no modification)
    
    Args:
        action_delta: (14,) [pos_delta_L_3d, rot_delta_L_3d, grip_L_1d, pos_delta_R_3d, rot_delta_R_3d, grip_R_1d]
        obs_current: dict with 'robot0_eef_pos' (6,), 'robot0_eef_quat' (8,), 'robot0_gripper_qpos' (2,)
    
    Returns:
        action_absolute: (16,) [pos_L_3d, quat_L_4d, grip_L_1d, pos_R_3d, quat_R_4d, grip_R_1d]
    """
    if action_delta.shape != (14,):
        raise ValueError(f"Expected shape (14,), got {action_delta.shape}")
    
    if dataset_obs is None : 
        # Extract current observations
        eef_pos = obs_current["robot0_eef_pos"]  # (6,)
        eef_quat = obs_current["robot0_eef_quat"]  # (8,) [wxyz_left, wxyz_right]
    else : 
        eef_pos = dataset_obs["robot0_eef_pos"][t]  # (6,)
        eef_quat = dataset_obs["robot0_eef_quat"][t]  # (8,) [wxyz_left, wxyz_right]

    # --- LEFT ARM ---
    pos_curr_left = eef_pos[0:3]
    quat_curr_left = eef_quat[0:4]  # [w, x, y, z]
    grip_left = action_delta[6:7]  # Absolute gripper (no change needed)
    
    # Absolute position
    pos_delta_left = action_delta[0:3]
    pos_target_left = pos_curr_left + pos_delta_left
    
    # Absolute quaternion
    rot_delta_left = action_delta[3:6]  # axis-angle
    quat_target_left = _apply_delta_rotation_to_quat(quat_curr_left, rot_delta_left)

    print("quat_target_left : ", quat_target_left)
    print("pos_target_left", pos_target_left)
    
    # --- RIGHT ARM ---
    pos_curr_right = eef_pos[3:6]
    quat_curr_right = eef_quat[4:8]  # [w, x, y, z]
    grip_right = action_delta[13:14]  # Absolute gripper
    
    # Absolute position
    pos_delta_right = action_delta[7:10]
    pos_target_right = pos_curr_right + pos_delta_right
    
    # Absolute quaternion
    rot_delta_right = action_delta[10:13]  # axis-angle
    quat_target_right = _apply_delta_rotation_to_quat(quat_curr_right, rot_delta_right)

    print("quat_target_right : ", quat_target_right)
    print("pos_target_right : ",pos_target_right)
    
    # Assemble 16D action
    action_absolute = np.concatenate([
        pos_target_left, quat_target_left, grip_left,
        pos_target_right, quat_target_right, grip_right
    ])
    
    return action_absolute


def _apply_delta_rotation_to_quat(q_curr: np.ndarray, a_delta: np.ndarray) -> np.ndarray:
    """
    Apply delta rotation (axis-angle) to current quaternion.
    
    Applies local frame rotation: q_target = q_curr ⊗ q_delta
    
    Args:
        q_curr: (4,) quaternion in [w, x, y, z] format (MuJoCo convention)
        a_delta: (3,) axis-angle delta rotation vector
    
    Returns:
        q_target: (4,) quaternion in [w, x, y, z] format
    """
    # Convert current quat to scipy format [x, y, z, w]
    q_curr_scipy = np.array([q_curr[1], q_curr[2], q_curr[3], q_curr[0]])
    
    # Convert axis-angle delta to quaternion (scipy format)
    q_delta_scipy = R.from_rotvec(a_delta).as_quat()  # [x, y, z, w]
    
    # Compose: q_target = q_curr ⊗ q_delta (local frame)
    R_curr = R.from_quat(q_curr_scipy)
    R_delta = R.from_quat(q_delta_scipy)
    R_target = R_curr * R_delta  # right-multiply = local frame
    q_target_scipy = R_target.as_quat()
    
    # Convert back to MuJoCo format [w, x, y, z]
    q_target = np.array([
        q_target_scipy[3],  # w
        q_target_scipy[0],  # x
        q_target_scipy[1],  # y
        q_target_scipy[2]   # z
    ])
    
    return q_target


def convert_action_for_simulator(action: np.ndarray, obs_current: dict, dataset_obs, t) -> np.ndarray:
    """
    Full pipeline to convert normalized delta action to absolute action for simulator.
    
    Steps:
    1. Denormalize action deltas (including gripper)
    2. Apply deltas to current observations to get absolute positions/quaternions
    3. Return 16D absolute action
    
    Args:
        action: (14,) normalized delta action from dataset
        obs_current: dict with current observations
    
    Returns:
        action_sim: (16,) absolute action for simulator
    """
    # Denormalize
    action_denorm = denormalize_action_deltas(action)
    
    # Convert to absolute
    action_sim = delta_to_absolute_action(action_denorm, obs_current, dataset_obs, t)
    
    return action_sim

    
def plotting_sim_teleop_with_dataset(dataset_path = "/home/qtf5422/Desktop/AIRE/ibrl-docker/data/cube_picking_and_placing_ee/tresholding_cube_nf_floor_cut_angle_axis//home/qtf5422/Desktop/AIRE/ibrl-docker/data/cube_picking_and_placing_ee/tresholding_cube_nf_floor_cut_angle_axis.hdf5", demo_name="demo_0"):

    for demo_number in range(50) : 
        demo_name = f"demo_{demo_number}"
        actions, dataset_obs, action_quat = load_demo_actions_and_obs(dataset_path, demo_name)

        cam_list = ["cam_high", "cam_low", "cam_left_wrist", "cam_right_wrist"]
        env = make_sim_env(TransferCubeEETask, "trossen_ai_scene.xml")
        ts = env.reset()

        ###########################################################################

        start_pos = dataset_obs["robot0_eef_pos"][0]
        start_quat = dataset_obs["robot0_eef_quat"][0]
        gripper_pos = dataset_obs["robot0_gripper_qpos"][0]

        print("start_pos : ", start_pos )
        print("start_quat : ", start_quat)
        print("gripper_pos : ", gripper_pos)

        start = np.concatenate([start_pos[0:3], start_quat[0:4], np.array([gripper_pos[0]]), 
                         start_pos[3:], start_quat[4:], np.array([gripper_pos[1]])])
        
        print("start : ", start)

        for i in range(100) : 
            ts = env.step(start)

        print("ts observation : ", ts.observation["robot0_eef_pos"], ts.observation["robot0_eef_quat"], ts.observation["robot0_gripper_qpos"])

            #   ##########################################################################
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
            action_step = actions[t]

            if USE_AXIS_ANGLE : 
                action_step = action_aa_to_quat(action_step)
            elif USE_DELTA :
                 
                use_dataset_obs = False 
                if use_dataset_obs : 
                    dataset_obs_input = dataset_obs
                else : 
                    dataset_obs_input = None

                # Convert normalized delta action to absolute action for simulator
                action_step = convert_action_for_simulator(
                    action_step, 
                    ts.observation, 
                    dataset_obs_input,
                    t 
                )

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
    test_ee_sim_env()

    # plotting_sim_teleop_with_dataset(dataset_path = "/home/qtf5422/Desktop/AIRE/ibrl-docker/data/cube_picking_and_placing_ee/delta/dataset_1_tresholded_wr_shifted_delta_gripper_normed_cut_end_normalized.hdf5", demo_name="demo_0")