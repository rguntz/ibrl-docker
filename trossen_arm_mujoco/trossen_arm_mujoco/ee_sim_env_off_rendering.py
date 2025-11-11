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

import os
# Set environment variables for headless rendering BEFORE importing MuJoCo
os.environ['MUJOCO_GL'] = 'egl'  # Use EGL for headless rendering
os.environ['PYOPENGL_PLATFORM'] = 'egl'

from dm_control.mujoco.engine import Physics
from dm_control.suite import base
import numpy as np
from flask import Flask, Response
import cv2
import threading
import time

from trossen_arm_mujoco.constants import START_ARM_POSE
from trossen_arm_mujoco.utils import (
    get_observation_base,
    make_sim_env,
    sample_box_pose,
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

        physics.data.qpos[6] = action_left[7]
        physics.data.qpos[7] = action_left[7]
        physics.data.qpos[14] = action_right[7]
        physics.data.qpos[15] = action_right[7]

    def initialize_robots(self, physics: Physics) -> None:
        """
        Initialize the robots by resetting joint positions and aligning mocap bodies with end-effectors.

        :param physics: The simulation physics engine.
        """
        # reset joint position
        physics.named.data.qpos[:12] = START_ARM_POSE[:6] + START_ARM_POSE[8:14]

        # reset mocap to align with end effector
        np.copyto(physics.data.mocap_pos[0], [-0.19657, -0.019, 0.25021])
        np.copyto(physics.data.mocap_quat[0], [1, 0, 0, 0])
        # right
        np.copyto(physics.data.mocap_pos[1], [0.19657, -0.019, 0.25021])
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
        self.max_reward = 4 # maximum reward for this task => used in the trossen wrapper. 

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
        Compute the reward based on the cube's interaction with the robot and the environment.

        :param physics: The simulation physics engine.
        :return: The computed reward.
        """
        # return whether left gripper is holding the box
        all_contact_pairs = []
        for i_contact in range(physics.data.ncon):
            id_geom_1 = physics.data.contact[i_contact].geom1
            id_geom_2 = physics.data.contact[i_contact].geom2
            name_geom_1 = physics.model.id2name(id_geom_1, "geom")
            name_geom_2 = physics.model.id2name(id_geom_2, "geom")
            contact_pair = (name_geom_1, name_geom_2)
            all_contact_pairs.append(contact_pair)
        touch_left_gripper = (
            "red_box",
            "left/gripper_follower_left",
        ) in all_contact_pairs
        touch_right_gripper = (
            "red_box",
            "right/gripper_follower_left",
        ) in all_contact_pairs
        touch_table = ("red_box", "table") in all_contact_pairs

        reward = 0
        if touch_right_gripper:
            reward = 1
        if touch_right_gripper and not touch_table:  # lifted
            reward = 2
        if touch_left_gripper:  # attempted transfer
            reward = 3
        if touch_left_gripper and not touch_table:  # successful transfer
            reward = 4
        return reward


# Flask web streaming setup
app = Flask(__name__)

# Global variables for frame sharing
latest_frames = {}
frame_lock = threading.Lock()
running = True


def encode_frame(image):
    """Encode numpy array image to JPEG bytes."""
    _, buffer = cv2.imencode('.jpg', image)
    return buffer.tobytes()


def generate_stream(camera_name):
    """Generator function for streaming camera frames."""
    while running:
        with frame_lock:
            if camera_name in latest_frames:
                frame = latest_frames[camera_name]
            else:
                # Create blank frame if no data yet
                frame = np.zeros((480, 640, 3), dtype=np.uint8)
        
        jpeg_bytes = encode_frame(frame)
        yield (b'--frame\r\n'
               b'Content-Type: image/jpeg\r\n\r\n' + jpeg_bytes + b'\r\n')
        time.sleep(0.03)  # ~30 FPS


@app.route('/video_feed/<camera_name>')
def video_feed(camera_name):
    """Video streaming route for each camera."""
    return Response(generate_stream(camera_name),
                    mimetype='multipart/x-mixed-replace; boundary=frame')


@app.route('/')
def index():
    """Main page with all camera feeds."""
    html = """
    <!DOCTYPE html>
    <html>
    <head>
        <title>Robot Camera Feeds</title>
        <style>
            body {
                font-family: Arial, sans-serif;
                background-color: #1a1a1a;
                color: white;
                margin: 0;
                padding: 20px;
            }
            h1 {
                text-align: center;
            }
            .camera-grid {
                display: grid;
                grid-template-columns: repeat(2, 1fr);
                gap: 20px;
                max-width: 1400px;
                margin: 0 auto;
            }
            .camera-view {
                background: #2a2a2a;
                border-radius: 8px;
                padding: 15px;
                box-shadow: 0 4px 6px rgba(0,0,0,0.3);
            }
            .camera-view h2 {
                margin-top: 0;
                color: #4CAF50;
                font-size: 18px;
            }
            .camera-view img {
                width: 100%;
                height: auto;
                border-radius: 4px;
            }
        </style>
    </head>
    <body>
        <h1>Trossen AI Robot Camera Feeds</h1>
        <div class="camera-grid">
            <div class="camera-view">
                <h2>High Camera</h2>
                <img src="/video_feed/cam_high" />
            </div>
            <div class="camera-view">
                <h2>Low Camera</h2>
                <img src="/video_feed/cam_low" />
            </div>
            <div class="camera-view">
                <h2>Left Wrist Camera</h2>
                <img src="/video_feed/cam_left_wrist" />
            </div>
            <div class="camera-view">
                <h2>Right Wrist Camera</h2>
                <img src="/video_feed/cam_right_wrist" />
            </div>
        </div>
    </body>
    </html>
    """
    return html


def run_simulation():
    """Run the simulation and update frames."""
    global latest_frames, running
    
    cam_list = ["cam_high", "cam_low", "cam_left_wrist", "cam_right_wrist"]
    env = make_sim_env(
        TransferCubeEETask,
        task_name="sim_transfer_cube",
        onscreen_render=False,  # Headless mode
        cam_list=cam_list,
    )
    
    ts = env.reset()
    
    for t in range(10000):  # Long-running simulation
        if not running:
            break
            
        action = np.random.uniform(-0.1, 0.1, 23)
        ts = env.step(action)
        print(ts.reward, "reward")
        
        # Update frames for streaming
        with frame_lock:
            print("updating frames")
            for cam_name in cam_list:
                img = ts.observation['images'][cam_name]
                latest_frames[cam_name] = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)
        
        if t % 100 == 0:
            print(f"Step: {t}, Reward: {ts.reward}")
        
        time.sleep(0.01)  # Small delay to control simulation speed


def test_ee_sim_env():
    """Test the simulation environment with Flask streaming."""
    # Start simulation in a separate thread
    sim_thread = threading.Thread(target=run_simulation, daemon=True)
    sim_thread.start()
    
    print("\n" + "="*60)
    print("Starting Flask server for camera streaming...")
    print("Open your browser and navigate to:")
    print("  http://localhost:5000")
    print("\nIf running in Docker, make sure to expose port 5000")
    print("Example: docker run -p 5000:5000 your_image")
    print("="*60 + "\n")
    
    # Start Flask server (use 0.0.0.0 to allow external access)
    app.run(host='0.0.0.0', port=5000, threaded=True, debug=False)


if __name__ == "__main__":
    try:
        test_ee_sim_env()
    except KeyboardInterrupt:
        print("\nShutting down...")
        running = False