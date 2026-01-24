#!/usr/bin/env python3
"""
MuJoCo Teleop with Server Integration
Runs MuJoCo viewer + sends camera frames to web UI server
Server controls when to record (via START/STOP buttons)
"""

import mujoco
import mujoco.viewer
import time
import numpy as np
import trossen_arm
from pathlib import Path
import requests
import json
from trossen_arm_mujoco.utils import make_sim_env
from trossen_arm_mujoco.ee_sim_env import TransferCubeEETask
from trossen_arm_mujoco.utils import (
    get_observation_base,
    make_sim_env,
    plot_observation_images,
    sample_box_pose,
    set_observation_images,
)
import cv2

import numpy as np
from scipy.spatial.transform import Rotation as R
from collections import OrderedDict
from typing import Optional
import threading
import pyrealsense2 as rs

REAL_DATA = True


class TimeStep:
    """Simple timestep container to mimic dm_control TimeStep."""
    def __init__(self, observation, reward, discount=1.0):
        self.observation = observation
        self.reward = reward
        self.discount = discount  

class Trossen_env : 
    def __init__(
            self, 
            follower_left_ip, 
            follower_right_ip, 
            control_dt, 
            model
            
    ): 
        self.follower_left_ip = follower_left_ip
        self.follower_right_ip = follower_right_ip
        self.control_dt = control_dt

        # State tracking for velocity estimation
        self.prev_poses = {}
        self.filtered_velocities = {}
        
        # Configuration parameters
        self.use_velocity_feedforward = True
        self.interpolation_mode = trossen_arm.InterpolationSpace.cartesian
        self.velocity_filter_alpha = 0.3
        self.model = self._parse_model(model_str=model)
        
        # Robot state (16D: 7 per arm joint states + 1 gripper each + 1 for discrepancy)
        self._qpos = np.zeros(16)
        self._qvel = np.zeros(16)
        self._step_count = 0

        self.follower_left = self.initialize_robot(self.follower_left_ip, 'follower', 'follower_left')
        self.follower_right = self.initialize_robot(self.follower_right_ip, 'follower', 'follower_right')

                # Initialize cameras
        scene_cam_ips = ["339222070421", "218622273297"]  # ← as STRINGS
        self.scene_cam_ips = scene_cam_ips or [None, None]
        left_wrist_serial = "230422271087"
        right_wrist_serial = "128422270155"
        self.camera_pipelines = {}
        self._init_cameras(left_wrist_serial, right_wrist_serial)

    def _init_cameras(self, left_wrist_serial, right_wrist_serial):
        """Initialize 4 RealSense pipelines."""
        cam_map = {
            'cam_left_wrist': left_wrist_serial,
            'cam_right_wrist': right_wrist_serial,
            'cam_high': self.scene_cam_ips[0],
            'cam_low': self.scene_cam_ips[1],
        }
        print("cam_map : ", cam_map)
        
        for name, serial in cam_map.items():
            pipe = rs.pipeline()
            cfg = rs.config()
            if serial:
                cfg.enable_device(serial)
            cfg.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
            pipe.start(cfg)
            self.camera_pipelines[name] = pipe
            print(f"📷 Camera {name} started (device: {serial or 'auto'})")

    def _capture_images(self):
        """Capture and return images from all 4 cameras."""
        images = {}
        for name, pipe in self.camera_pipelines.items():
            try:
                frames = pipe.wait_for_frames(timeout_ms=500)
                color_frame = frames.get_color_frame()
                if color_frame:
                    img = np.asanyarray(color_frame.get_data())
                    # Ensure correct shape (H, W, C)
                    if img.ndim == 3 and img.shape[2] == 3:
                        images[name] = img
                    else:
                        images[name] = np.zeros((480, 640, 3), dtype=np.uint8)
                else:
                    images[name] = np.zeros((480, 640, 3), dtype=np.uint8)
            except RuntimeError:
                # Timeout or no frame
                images[name] = np.zeros((480, 640, 3), dtype=np.uint8)
        return images
    
    def _parse_model(self, model_str: str):
        """Parse model string to trossen_arm.Model enum."""
        if model_str == 'wxai_v0':
            return trossen_arm.Model.wxai_v0
        elif model_str == 'vxai_v0':
            return trossen_arm.Model.vxai_v0
        else:
            raise ValueError(f"Unknown model: {model_str}")
        
    def estimate_velocity(
        self,
        current_pose: np.ndarray,
        robot_id: str
    ) -> np.ndarray:
        """
        Estimate Cartesian velocity using finite differences with exponential smoothing.
        
        Args:
            current_pose: Current 6D pose [x, y, z, rx, ry, rz]
            robot_id: Robot identifier
            
        Returns:
            Smoothed velocity estimate [vx, vy, vz, wx, wy, wz]
        """
        if self.prev_poses[robot_id] is None:
            self.prev_poses[robot_id] = current_pose
            return np.zeros(6)
        
        # Raw velocity from finite difference
        raw_velocity = (current_pose - self.prev_poses[robot_id]) / self.control_dt
        
        # Exponential smoothing filter
        self.filtered_velocities[robot_id] = (
            self.velocity_filter_alpha * raw_velocity +
            (1 - self.velocity_filter_alpha) * self.filtered_velocities[robot_id]
        )
        
        # Update previous pose
        self.prev_poses[robot_id] = current_pose.copy()
        
        return self.filtered_velocities[robot_id]
    
    def send_cartesian_command(
        self,
        driver: trossen_arm.TrossenArmDriver,
        target_pose: np.ndarray,
        gripper_pos: float,
        velocity: Optional[np.ndarray] = None,
        robot_id: str = 'robot'
    ):
        """
        Send a Cartesian pose command to the robot.
        
        Args:
            driver: TrossenArmDriver instance
            target_pose: Target 6D pose [x, y, z, rx, ry, rz]
            gripper_pos: Gripper position (0.0 to 1.0)
            velocity: Optional velocity feedforward [vx, vy, vz, wx, wy, wz]
            robot_id: Robot identifier for velocity tracking
        """
        # Estimate velocity if not provided
        if velocity is None and self.use_velocity_feedforward:
            
            velocity = self.estimate_velocity(target_pose, robot_id)
        
        # Send Cartesian command for arm
        if self.use_velocity_feedforward and velocity is not None:
            driver.set_cartesian_positions(
                goal_positions=target_pose.tolist(),
                interpolation_space=self.interpolation_mode,
                goal_time=self.control_dt * 1.5,  # Slightly longer than control period
                goal_feedforward_velocities=velocity.tolist(),
                blocking=False  # Non-blocking for continuous control
            )
        else:
            driver.set_cartesian_positions(
                goal_positions=target_pose.tolist(),
                interpolation_space=self.interpolation_mode,
                goal_time=self.control_dt * 1.5,
                blocking=False
            )
        
        # Send gripper command separately with FAST response time
        # Use very short goal_time (0.05s = 50ms) for immediate gripper response
        driver.set_gripper_position(gripper_pos, goal_time=0.05, blocking=False)
    
    
    def _configure_driver_with_retry(self, driver, model, ee_type, ip, robot_id, max_retries=10, attempt_timeout=1.0):
        """
        Configure a driver with retry logic and timeout per attempt.
        
        Args:
            driver: TrossenArmDriver instance
            model: Robot model
            ee_type: End effector type
            ip: Robot IP address
            robot_id: Robot identifier for logging
            max_retries: Maximum number of retry attempts
            attempt_timeout: Timeout per attempt in seconds (will abort and retry if exceeded)
            
        Returns:
            True if successful, False otherwise
        """
        for attempt in range(1, max_retries + 1):
            print(f"  [{attempt}/{max_retries}] Attempting to configure {robot_id} at {ip} (timeout: {attempt_timeout}s)...")
            
            # Flag to track if configure completed
            success_flag = {'completed': False, 'exception': None}
            
            def configure_thread():
                try:
                    driver.configure(model, ee_type, ip, False)
                    success_flag['completed'] = True
                except Exception as e:
                    success_flag['exception'] = e
            
            # Run configure in a thread with timeout
            thread = threading.Thread(target=configure_thread, daemon=True)
            thread.start()
            thread.join(timeout=attempt_timeout)
            
            # Check if thread completed successfully
            if success_flag['completed']:
                print(f"✓ {robot_id} configured successfully on attempt {attempt}")
                return True
            
            # Thread either timed out or had an exception
            if success_flag['exception']:
                print(f"  ⚠️  Attempt {attempt} failed with error: {success_flag['exception']}")
            else:
                print(f"  ⚠️  Attempt {attempt} timed out after {attempt_timeout}s")
            
            if attempt < max_retries:
                print(f"     Retrying...")
            else:
                print(f"  ❌ Failed to configure {robot_id} after {max_retries} attempts")
                raise RuntimeError(f"Failed to configure {robot_id} after {max_retries} attempts with {attempt_timeout}s timeout each")
        
        return False
    
    def initialize_robot(
        self,
        ip: str,
        end_effector: str = 'leader',
        robot_id: str = 'robot'
    ) -> trossen_arm.TrossenArmDriver:
        """
        Initialize a robot driver.
        
        Args:
            ip: Robot IP address
            end_effector: 'leader' or 'follower'
            robot_id: Unique identifier for this robot
            
        Returns:
            Initialized TrossenArmDriver
        """
        print(f"Connecting to {robot_id} at {ip}...")
        
        driver = trossen_arm.TrossenArmDriver()
        
        # Set end effector type
        if end_effector == 'leader':
            ee_type = trossen_arm.StandardEndEffector.wxai_v0_leader
        else:
            ee_type = trossen_arm.StandardEndEffector.wxai_v0_follower
        
        # Configure with retry logic
        self._configure_driver_with_retry(driver, self.model, ee_type, ip, robot_id, max_retries=10, attempt_timeout=1.0)
        
        # Set appropriate mode based on robot role
        if end_effector == 'leader':
            # Leaders: external effort mode (free movement, like joint teleop)
            # Don't set mode here - will be set when starting teleop
            print(f"✓ {robot_id} connected ({driver.get_num_joints()} joints)")
            print(f"  Role: LEADER (will be set to external effort mode)")
        else:
            print("we are initializing follower")
            # Followers: position mode (stiff, controlled)
            driver.set_arm_modes(trossen_arm.Mode.position)
            # IMPORTANT: Set gripper to position mode separately
            driver.set_gripper_mode(trossen_arm.Mode.position)
            print(f"✓ {robot_id} connected ({driver.get_num_joints()} joints)")
            print(f"  Mode: POSITION (arm + gripper controlled)")
        
        # Initialize state tracking
        self.prev_poses[robot_id] = None
        self.filtered_velocities[robot_id] = np.zeros(6)
        
        return driver
        
    def move_followers_home(self,gripper_open=0.044) : 

        # Home state for arms
        home_arm = np.zeros(6)  # 6 arm joints
        left_state = np.concatenate([home_arm, [gripper_open]])
        right_state = np.concatenate([home_arm, [gripper_open]])
        
        # Switch leaders to position mode temporarily
        self.follower_left.set_all_modes(trossen_arm.Mode.position)
        self.follower_right.set_all_modes(trossen_arm.Mode.position)
        
        # Move real robots
        self.follower_left.set_all_positions(left_state)
        self.follower_right.set_all_positions(right_state)


        # After moving to home, make sure leaders are free to teleoperate again
        try:
            # Set both leaders back to external effort (free movement)
            if self.follower_left is not None:
                robot_id = 'follower_left'
                self.follower_left.set_arm_modes(trossen_arm.Mode.position)
                self.follower_left.set_gripper_mode(trossen_arm.Mode.position)

                print(f"✓ {robot_id} connected ({self.follower_left.get_num_joints()} joints)")
                print(f"  Mode: POSITION (arm + gripper controlled)")

            if self.follower_right is not None:
                robot_id = 'follower_right'
                self.follower_right.set_arm_modes(trossen_arm.Mode.position)
                self.follower_right.set_gripper_mode(trossen_arm.Mode.position)

                print(f"✓ {robot_id} connected ({self.follower_right.get_num_joints()} joints)")
                print(f"  Mode: POSITION (arm + gripper controlled)")

            print("✓ Followers set to external effort (free to teleoperate)")
        except Exception as e:
            print(f"⚠️  Failed to set leaders free after homing: {e}")

        print("✓ Robots are now at HOME configuration (grippers open)")

        # Initialize state tracking
        self.prev_poses[robot_id] = None
        self.filtered_velocities[robot_id] = np.zeros(6)


    def reset(self) : 

        # Move the followers to home
        print("moving the followers to home")
        self.move_followers_home()
        print("finished moving robots to home")
        
        # Reset state tracking
        self._step_count = 0
        self._qpos = np.zeros(16)
        self._qvel = np.zeros(16)
        
        # Create dummy observation matching the simulation environment structure
        self.observation = self._create_dummy_observation()
        
        # Return TimeStep with dummy values
        return TimeStep(
            observation=self.observation,
            reward=0.0,
            discount=1.0
        )
    
    def _create_dummy_observation(self):
        """Create a dummy observation dict matching the sim env structure."""
        observation = OrderedDict()
        
        # Joint states (16D: 6 arm joints + 1 gripper per arm, x2)
        observation['qpos'] = np.zeros(16)
        observation['qvel'] = np.zeros(16)
        
        # End-effector pose (6D: 3D position for each arm)
        observation['robot0_eef_pos'] = np.zeros(6)
        
        # End-effector orientation (8D: 4D quat for each arm)
        observation['robot0_eef_quat'] = np.array([1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0])
        
        # Gripper positions (2D: 1 per gripper)
        observation['robot0_gripper_qpos'] = np.zeros(2)
        
        # Camera images (dict of camera name -> image array)
        observation['images'] = self._capture_images()

        return observation


    def step(self, action): 
        loop_start = time.time()

        self.send_cartesian_command(self.follower_left, action[:6], action[6], robot_id='follower_left')
        self.send_cartesian_command(self.follower_right, action[7:-1], action[-1], robot_id='follower_right')
        
        # Update internal state tracking
        self._step_count += 1
        
        # Update dummy qpos and qvel (in real scenario, you'd read from robot sensors)
        self._qpos = action.copy()  # Simplified: use action as qpos
        self._qvel = np.zeros(16)   # Dummy velocities
            
        # Sleep to maintain control frequency
        loop_time = time.time() - loop_start
        sleep_time = max(0, self.control_dt - loop_time)
        if sleep_time > 0:
            time.sleep(sleep_time)
        
        observation = self._create_dummy_observation_with_action(action)
        # Return TimeStep (with dummy reward for now)
        reward = 0.0
        return TimeStep(
            observation=observation,
            reward=reward,
            discount=1.0
        )
    
    def _create_dummy_observation_with_action(self, action):
        """Create a dummy observation dict with the current action as state."""
        observation = OrderedDict()
        
        # Joint states (16D)
        observation['qpos'] = self._qpos.copy()
        observation['qvel'] = self._qvel.copy()
        
        # Extract pose and gripper from action and populate observation
        left_pose = action[:6]
        left_gripper = action[6]
        right_pose = action[7:-1]
        right_gripper = action[-1]
        
        # End-effector position (6D: 3D per arm)
        observation['robot0_eef_pos'] = np.concatenate([left_pose[:3], right_pose[:3]])
        
        # End-effector orientation (8D: 4D quat per arm, using identity as dummy)
        observation['robot0_eef_quat'] = np.array([1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0])
        
        # Gripper positions (2D)
        observation['robot0_gripper_qpos'] = np.array([left_gripper, right_gripper])
        
        # Camera images (dummy)
        observation['images'] = self._capture_images()

        return observation

class TeleopWithServer:
    def __init__(self, 
                 leader_left_ip='192.168.1.2',
                 leader_right_ip='192.168.1.4',
                 server_url='http://localhost:5000',
                 visualize=True,
                 camera_names=None,
                 camera_resolution=(128, 128)): #  here is where we set the resolution of the image. 
        
        self.leader_left_ip = leader_left_ip
        self.leader_right_ip = leader_right_ip
        self.server_url = server_url
        self.visualize = visualize
        
        if camera_names is None:
            self.camera_names = ['cam_high', 'cam_low', 'cam_left_wrist', 'cam_right_wrist']
        else:
            self.camera_names = camera_names
        
        # Map camera names to IDs (for server API)
        self.camera_name_to_id = {
            'cam_high': 0,
            'cam_low': 1,
            'cam_left_wrist': 2,
            'cam_right_wrist': 3
        }
        
        self.camera_resolution = camera_resolution
        # Track remote recording state so we can react to STOP events
        self.prev_remote_recording = False
        
        # Robot drivers
        self.driver_left = None
        self.driver_right = None
        self.mj_model = None
        self.mj_data = None
        self.viewer = None
        self.renderer = None
        
    def test_camera_feeds(self, follower_left_ip, follower_right_ip, scene_cam_ips, duration=10):
        """
        Initialize and display live feeds from 4 RealSense cameras:
        - 2 wrist cameras (follower arms)
        - 2 scene cameras (static)

        Args:
            follower_left_ip (str): IP or serial of left wrist cam
            follower_right_ip (str): IP or serial of right wrist cam
            scene_cam_ips (List[str]): IPs/serials of two scene cams (e.g., ['192.168.1.10', '192.168.1.11'])
            duration (int): Seconds to display feeds (default: 10s)
        """
        from collections import OrderedDict
        import signal

        # Map camera names to sources
        cam_sources = OrderedDict([
            ('cam_left_wrist', follower_left_ip),
            ('cam_right_wrist', follower_right_ip),
            ('cam_high', scene_cam_ips[0]),
            ('cam_low', scene_cam_ips[1]),
        ])

        pipelines = {}
        configs = {}

        try:
            print("🔍 Initializing RealSense cameras for preview...")
            for name, src in cam_sources.items():
                pipe = rs.pipeline()
                cfg = rs.config()

                # Use IP/serial if provided, otherwise auto-select
                if src:
                    cfg.enable_device(src)
                
                # Enable color stream (you can adjust resolution/fps)
                cfg.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
                pipelines[name] = pipe
                configs[name] = cfg

                # Start pipeline
                pipe.start(cfg)
                print(f"  ✓ {name} → {src or 'auto'}")

            print(f"\n🎥 Showing live feeds for {duration} seconds. Press Ctrl+C to skip early.")
            print("   Close windows or wait to continue...\n")

            start_time = time.time()
            while (time.time() - start_time) < duration:
                frames_dict = {}
                valid = True
                for name, pipe in pipelines.items():
                    try:
                        frames = pipe.wait_for_frames(timeout_ms=500)
                        color_frame = frames.get_color_frame()
                        if not color_frame:
                            valid = False
                            continue
                        img = np.asanyarray(color_frame.get_data())
                        frames_dict[name] = img
                    except RuntimeError as e:
                        print(f"⚠️  Timeout on {name}: {e}")
                        valid = False

                if not valid:
                    time.sleep(0.1)
                    continue

                # Display all feeds in a grid or separate windows
                for name, img in frames_dict.items():
                    cv2.imshow(name, img)

                if cv2.waitKey(1) & 0xFF == ord('q'):
                    break

                time.sleep(0.03)  # ~30 Hz

            cv2.destroyAllWindows()

        except Exception as e:
            print(f"❌ Camera test failed: {e}")
            raise
        finally:
            # Stop all pipelines
            for pipe in pipelines.values():
                try:
                    pipe.stop()
                except:
                    pass

        # Reinitialize pipelines for actual use during teleop (will be done in Trossen_env)
        print("✅ Camera test complete. Proceeding to teleop setup.")


    def _configure_driver_with_retry(self, driver, model, ee_type, ip, robot_id, max_retries=10, attempt_timeout=1.0):
        """
        Configure a driver with retry logic and timeout per attempt.
        
        Args:
            driver: TrossenArmDriver instance
            model: Robot model
            ee_type: End effector type
            ip: Robot IP address
            robot_id: Robot identifier for logging
            max_retries: Maximum number of retry attempts
            attempt_timeout: Timeout per attempt in seconds (will abort and retry if exceeded)
            
        Returns:
            True if successful, False otherwise
        """
        for attempt in range(1, max_retries + 1):
            print(f"  [{attempt}/{max_retries}] Attempting to configure {robot_id} at {ip} (timeout: {attempt_timeout}s)...")
            
            # Flag to track if configure completed
            success_flag = {'completed': False, 'exception': None}
            
            def configure_thread():
                try:
                    driver.configure(model, ee_type, ip, False)
                    success_flag['completed'] = True
                except Exception as e:
                    success_flag['exception'] = e
            
            # Run configure in a thread with timeout
            thread = threading.Thread(target=configure_thread, daemon=True)
            thread.start()
            thread.join(timeout=attempt_timeout)
            
            # Check if thread completed successfully
            if success_flag['completed']:
                print(f"✓ {robot_id} configured successfully on attempt {attempt}")
                return True
            
            # Thread either timed out or had an exception
            if success_flag['exception']:
                print(f"  ⚠️  Attempt {attempt} failed with error: {success_flag['exception']}")
            else:
                print(f"  ⚠️  Attempt {attempt} timed out after {attempt_timeout}s")
            
            if attempt < max_retries:
                print(f"     Retrying...")
            else:
                print(f"  ❌ Failed to configure {robot_id} after {max_retries} attempts")
                raise RuntimeError(f"Failed to configure {robot_id} after {max_retries} attempts with {attempt_timeout}s timeout each")
        
        return False

    def initialize(self):
        """Connect to both leader robots and load MuJoCo"""
        print("="*60)
        print("Initializing Dual Robot Teleop + Server Connection")
        print("="*60)
        
        # Connect to leader robots
        print(f"📡 Connecting to leader robots...")
        
        # Left leader
        print(f"  Left at {self.leader_left_ip}...")
        self.driver_left = trossen_arm.TrossenArmDriver()
        self._configure_driver_with_retry(
            self.driver_left,
            trossen_arm.Model.wxai_v0,
            trossen_arm.StandardEndEffector.wxai_v0_leader,
            self.leader_left_ip,
            'left_leader',
            max_retries=10,
            attempt_timeout=1.0
        )
        
        # Right leader
        print(f"  Right at {self.leader_right_ip}...")
        self.driver_right = trossen_arm.TrossenArmDriver()
        self._configure_driver_with_retry(
            self.driver_right,
            trossen_arm.Model.wxai_v0,
            trossen_arm.StandardEndEffector.wxai_v0_leader,
            self.leader_right_ip,
            'right_leader',
            max_retries=10,
            attempt_timeout=1.0
        )
        
        num_joints_left = self.driver_left.get_num_joints()
        num_joints_right = self.driver_right.get_num_joints()
        print(f"✓ Both leaders connected")
        print(f"  Left: {num_joints_left} joints, Right: {num_joints_right} joints")
        
        onscreen_render = True
        self.cam_list = ["cam_high", "cam_low", "cam_left_wrist", "cam_right_wrist"]

        if not REAL_DATA : 
            # Create Mujoco env. 
            self.env = make_sim_env(
                TransferCubeEETask,
                task_name="sim_transfer_cube",
                onscreen_render=onscreen_render,
                cam_list=self.cam_list,
            )
        else : # Use real robots => use the trossen env custom
            follower_left_ip = '192.168.1.5'
            follower_right_ip ='192.168.1.3'
            control_frequency = 50 # 50 hertz to start
            control_dt = 1.0 / control_frequency
            model = "wxai_v0"
            self.env = Trossen_env(follower_left_ip, follower_right_ip, control_dt, model)


        # Check server connection
        print(f"🌐 Connecting to server at {self.server_url}...")
        try:
            response = requests.get(f"{self.server_url}/api/status", timeout=2)
            if response.status_code == 200:
                print(f"✓ Server connected")
                # Initialize remote recording state from server
                try:
                    self.prev_remote_recording = bool(response.json().get('recording', False))
                except Exception:
                    self.prev_remote_recording = False
            else:
                print(f"⚠️  Server responded with status {response.status_code}")
        except Exception as e:
            print(f"❌ Cannot connect to server: {e}")
            print(f"   Make sure Flask server is running: cd sim_recorder/server && python app.py")
            return False
        
        print("="*60)
        print("✅ Initialization complete!")
        if self.visualize:
            print(f"🎥 MuJoCo viewer will open now")
        else:
            print(f"🎥 MuJoCo viewer disabled - running headless")
        print(f"🌐 Web UI at {self.server_url}")
        print(f"🔴 Click START in web UI to begin recording")
        print("="*60)
        return True

    
    def move_robots_to_home(self, first_time_init = False, gripper_open=0.044):
        """Move both leader robots and sim to home (arms zero, grippers open)"""
        print("🏠 Moving both robots to HOME configuration (arms=0, gripper=open)...")
        
        # Home state for arms
        home_arm = np.zeros(6)  # 6 arm joints
        left_state = np.concatenate([home_arm, [gripper_open]])
        right_state = np.concatenate([home_arm, [gripper_open]])
        
        # Switch leaders to position mode temporarily
        self.driver_left.set_all_modes(trossen_arm.Mode.position)
        self.driver_right.set_all_modes(trossen_arm.Mode.position)
        
        # Move real robots
        self.driver_left.set_all_positions(left_state)
        self.driver_right.set_all_positions(right_state)
        
        # Reset MuJoCo simulation (new cube spawned and position of arm set to 0, 0, 0, 0 ..., 0.044)
        self.ts = self.env.reset()
        if first_time_init : 
            self.plt_imgs = plot_observation_images(self.ts.observation, self.cam_list)

        # After moving to home, make sure leaders are free to teleoperate again
        try:
            zero_efforts = np.zeros(7)
            # Set both leaders back to external effort (free movement)
            if self.driver_left is not None:
                self.driver_left.set_all_modes(trossen_arm.Mode.external_effort)
                self.driver_left.set_all_external_efforts(zero_efforts, 0.0, False)
            if self.driver_right is not None:
                self.driver_right.set_all_modes(trossen_arm.Mode.external_effort)
                self.driver_right.set_all_external_efforts(zero_efforts, 0.0, False)
            print("✓ Leaders set to external effort (free to teleoperate)")
        except Exception as e:
            print(f"⚠️  Failed to set leaders free after homing: {e}")

        print("✓ Robots are now at HOME configuration (grippers open)")

    
    def capture_cameras(self):
        """Capture all camera images from self.ts and resize to server resolution"""
        images = {}
        for cam_name in self.camera_names:
            try:
                image = self.ts.observation['images'][cam_name]
                # Resize to 128x128
                resized = cv2.resize(image, self.camera_resolution, interpolation=cv2.INTER_AREA)
                images[cam_name] = resized.copy()
            except Exception as e:
                print(f"⚠️  Failed to capture {cam_name}: {e}")
        return images


    
    def push_frame_to_server(self, cam_name, image):
        """Send camera frame to server"""
        try:
            # Get camera ID
            cam_id = self.camera_name_to_id.get(cam_name)
            if cam_id is None:
                return
            
            # Convert to bytes
            img_bytes = image.tobytes()
            
            # POST to server with camera ID
            response = requests.post(
                f"{self.server_url}/api/frame/{cam_id}",
                data=img_bytes,
                headers={'Content-Type': 'application/octet-stream'},
                timeout=0.5
            )
            
            if response.status_code != 200:
                print(f"⚠️  Server returned {response.status_code} for {cam_name}")
                
        except requests.exceptions.Timeout:
            pass  # Ignore timeouts to not block control loop
        except Exception as e:
            print(f"⚠️  Failed to push frame: {e}")
    
    def push_state_to_server(self, qpos, qvel, action, robot0_eef_pos, robot0_eef_quat, robot0_gripper_qpos, reward):
        """Send robot state + action to server"""
        try:
            data = {
                'qpos': qpos.tolist(),
                'qvel': qvel.tolist(),
                'action': action.tolist(), 
                'robot0_eef_pos' : robot0_eef_pos.tolist(), 
                'robot0_eef_quat' : robot0_eef_quat.tolist(), 
                'robot0_gripper_qpos' : robot0_gripper_qpos.tolist(), 
                'reward' : float(reward) 
            }
            
            response = requests.post(
                f"{self.server_url}/api/state",
                json=data,
                timeout=0.5
            )
            
        except requests.exceptions.Timeout:
            pass
        except Exception as e:
            print(f"⚠️  Failed to push state: {e}")
    
    def run_teleop(self):
        """Main teleop loop with viewer"""
        if not self.initialize():
            return
        
        first_time_init = True # to init the graph only once. 
        self.move_robots_to_home(first_time_init)
        
        print("\n🎮 Starting dual robot teleop control loop...")
        print("   Move both leader robots to control sim robots")
        print("   Server will record when you click START in web UI")
        print("   Close MuJoCo viewer window to exit\n")
        
        # Set both leaders to external effort mode (free movement)
        print("Setting leaders to external effort mode (will be free to move)...")
        zero_efforts = np.zeros(7)
        
        self.driver_left.set_all_modes(trossen_arm.Mode.external_effort)
        self.driver_left.set_all_external_efforts(zero_efforts, 0.0, False)
        
        self.driver_right.set_all_modes(trossen_arm.Mode.external_effort)
        self.driver_right.set_all_external_efforts(zero_efforts, 0.0, False)
        
        print("✓ Both leaders are now FREE to move - start teleoperation!\n")
        
        try:
            if self.visualize:
                # Run with MuJoCo viewer
                self._run_teleop_loop()
            else:
                # Run headless (no viewer)
                self._run_teleop_loop_headless()
        
        except KeyboardInterrupt:
            print("\n⚠️  Interrupted by user")
        
        finally:
            print("\n🛑 Stopping teleop...")
            self.cleanup()
    

    def _run_teleop_loop(self):
        """Main teleop loop with viewer""" 
        print("teleop  starting")
        step_count = 0
        
        while True:
            step_start = time.time()
            
            self._teleop_step()
            
            step_count += 1
            if step_count % 500 == 0:  # Log every 500 steps (~1 second at 500Hz)
                elapsed = time.time() - step_start
                print(f"✓ Step {step_count} (running at ~{1.0/elapsed:.0f} Hz)")
    
    def quat_mul(self, q1, q2):
        """Multiply two quaternions (w, x, y, z)."""
        w1, x1, y1, z1 = q1
        w2, x2, y2, z2 = q2
        return np.array([
            w1*w2 - x1*x2 - y1*y2 - z1*z2,
            w1*x2 + x1*w2 + y1*z2 - z1*y2,
            w1*y2 - x1*z2 + y1*w2 + z1*x2,
            w1*z2 + x1*y2 - y1*x2 + z1*w2
        ])

    def quat_rotate(self, q, v):
        """Rotate vector v by unit quaternion q (w, x, y, z)."""
        # Convert vector to pure quaternion
        vq = np.array([0.0, v[0], v[1], v[2]])
        q_conj = np.array([q[0], -q[1], -q[2], -q[3]])
        # q * v * q_conj
        return self.quat_mul(self.quat_mul(q, vq), q_conj)[1:]

    def transform_robot_to_world_frame_qwen(self, pos_robot, quat_robot, robot_name):
        """
        Transform end-effector pose from robot base frame to world frame.
        
        Args:
            pos_robot (np.ndarray): (3,) position in robot's base frame.
            quat_robot (np.ndarray): (4,) [w, x, y, z] orientation in robot's base frame.
            robot_name (str): 'left' or 'right'
        
        Returns:
            (world_pos, world_quat): both (3,) and (4,) arrays in world frame.
        """
        if robot_name == "left":
            base_pos = np.array([-0.4575, -0.019, 0.02])
            base_quat = np.array([1.0, 0.0, 0.0, 0.0])  # identity
        elif robot_name == "right":
            base_pos = np.array([0.4575, -0.019, 0.02])
            base_quat = np.array([0.0, 0.0, 0.0, 1.0])  # 180° around Z
        else:
            raise ValueError("robot_name must be 'left' or 'right'")
        
        # Rotate end-effector position by base orientation
        pos_world = base_pos + self.quat_rotate(base_quat, pos_robot)
        
        # Compose orientations: world = base_quat * quat_robot
        quat_world = quat_robot

        if robot_name == "right":
            # Invert roll and pitch → equivalent to mirroring X and Y
            # This can be done by conjugating the quaternion and flipping Z?
            # Or more simply: negate x and y components
            quat_world = np.array([quat_world[0], -quat_world[1], -quat_world[2], quat_world[3]])

        return pos_world, quat_world

    def angle_axis_to_quaternion(self, cartesian):
        """
        Convert a 6-element cartesian position to a quaternion.
        
        Parameters:
            cartesian (array-like): 6 elements [x, y, z, rx, ry, rz]
                                    last 3 elements are angle-axis (rad)
        
        Returns:
            np.ndarray: Quaternion [w, x, y, z]
        """
        angle_axis = np.array(cartesian[3:6])
        angle = np.linalg.norm(angle_axis)
        
        if angle == 0.0:
            print("angle = 0")
            # No rotation, identity quaternion
            return np.array([1.0, 0.0, 0.0, 0.0])
        else:
            axis = angle_axis / angle
            rot = R.from_rotvec(axis * angle)
            q = rot.as_quat()  # Returns [x, y, z, w]
            # Convert to [w, x, y, z]
            return np.array([q[3], q[0], q[1], q[2]])
        
    def _teleop_step(self):
        """Single teleop step - shared between viewer and headless modes"""

        # # Combine for recording (16D state + 14D actions)
        # qpos = self.ts.observation["qpos"]
        # qvel = self.ts.observation["qvel"]
        # robot0_eef_pos = self.ts.observation["robot0_eef_pos"]
        # robot0_eef_quat = self.ts.observation["robot0_eef_quat"]    
        # robot0_gripper_qpos = self.ts.observation["robot0_gripper_qpos"]

        # # Capture cameras (every step)
        images = self.capture_cameras()
        
        # Get left arm state
        left_state = np.array(self.driver_left.get_cartesian_positions())
        left_gripper = self.driver_left.get_gripper_position()

        # Get right arm state
        right_state = np.array(self.driver_right.get_cartesian_positions())
        right_gripper = self.driver_right.get_gripper_position()

        full_state_vector = np.concatenate([
            left_state, [left_gripper],
            right_state, [right_gripper]
        ])

        # Apply to MuJoCo ctrl
        self.ts = self.env.step(full_state_vector)

        # # Get the reward after the stepping function. 
        # reward = 0.0 if self.ts.reward is None else self.ts.reward

        # set_observation_images(self.ts.observation, self.plt_imgs, self.cam_list)
                
        # action = full_state_vector.copy()  # 16D total 
        # # action is the left state which is what we read from the real robot. 
        
        # # Push data camera to server
        # for cam_name, image in images.items():
        #     self.push_frame_to_server(cam_name, image) 
        
        # # Send state data to server for recording
        # self.push_state_to_server(qpos, qvel, action, robot0_eef_pos, robot0_eef_quat, robot0_gripper_qpos, reward) 

        # # Both are pushed at the same moment so they correspond to the same image-state-action triplet. 
        # # Check server recording status to detect STOP event
        # try:
        #     resp = requests.get(f"{self.server_url}/api/status", timeout=0.2)
        #     if resp.status_code == 200:
        #         remote_rec = bool(resp.json().get('recording', False))
        #         # If server just transitioned from recording->not-recording, move robots to home
        #         if self.prev_remote_recording and not remote_rec:
        #             print("🎯 Detected remote STOP - moving robots to home...")
        #             try:
        #                 self.move_robots_to_home()
        #             except Exception as e:
        #                 print(f"⚠️  Failed to move robots to home after STOP: {e}")
        #         self.prev_remote_recording = remote_rec
        # except requests.exceptions.Timeout:
        #     # ignore short timeouts to avoid blocking teleop loop
        #     pass
        # except Exception as e:
        #     print(f"⚠️  Error checking server status: {e}")

        return True
    
    
    def cleanup(self):
        """Cleanup resources"""
        if self.driver_left:
            # TrossenArmDriver doesn't have disconnect, just delete the object
            self.driver_left = None
            print("✓ Left leader disconnected")
        
        if self.driver_right:
            self.driver_right = None
            print("✓ Right leader disconnected")
        
        if self.renderer:
            self.renderer.close()

        for pipe in self.camera_pipelines.values():
            try:
                pipe.stop()
            except:
                pass
        
        print("✓ Cleanup complete")


def main():
    import argparse
    
    parser = argparse.ArgumentParser(description='MuJoCo Dual Robot Teleop with Server Recording')
    parser.add_argument('--leader-left-ip', type=str, default='192.168.1.4',
                       help='Left leader robot IP address')
    parser.add_argument('--leader-right-ip', type=str, default='192.168.1.2',
                       help='Right leader robot IP address')
    parser.add_argument('--server-url', type=str, default='http://localhost:5000',
                       help='Recording server URL')
    parser.add_argument('--no-visualize', action='store_true',
                       help='Run without MuJoCo viewer (headless mode for web UI only)')
    
    args = parser.parse_args()
    
    teleop = TeleopWithServer(
        leader_left_ip=args.leader_left_ip,
        leader_right_ip=args.leader_right_ip,
        server_url=args.server_url,
        visualize=not args.no_visualize
    )
    
    teleop.run_teleop()


if __name__ == '__main__':
    main()
