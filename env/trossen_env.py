import time
import numpy as np
import trossen_arm
from pathlib import Path
import requests
import json
import cv2

import numpy as np
from scipy.spatial.transform import Rotation as R
from collections import OrderedDict
from typing import Optional
import threading
import pyrealsense2 as rs
import h5py
import json
from datetime import datetime
from pynput import keyboard

STATION1 = True
STATION2 = False

class TimeStep:
    def __init__(self, observation, reward, discount=1.0, step_type=None):
        self.observation = observation
        self.reward = reward
        self.discount = discount
        self._step_type = step_type  # 'mid', 'last'

    def first(self):
        return self._step_type == 'first'

    def mid(self):
        return self._step_type == 'mid'

    def last(self):
        return self._step_type == 'last'

class KeyboardSuccessMonitor:
    """Keyboard-based success monitor using space bar and interrupt with Enter."""
    def __init__(self):
        self.success_flag = False
        self.interrupt_flag = False  # Flag for episode interruption
        self.pause_flag = False  # NEW: Flag for pausing episode
        self.listener = None
        self.running = False
        
    def start(self):
        """Start the keyboard listener in a separate thread."""
        self.running = True
        self.listener = keyboard.Listener(on_press=self._on_key_press)
        self.listener.start()
        print("✓ Keyboard success monitor started")
        print("  Press SPACE to mark episode as successful")
        print("  Press ENTER to interrupt and end episode")
        print("  Press DELETE to pause/resume episode")
        print("  Press 'r' to reset success flag")
        
    def _on_key_press(self, key):
        """Handle key press events."""
        try:
            # Check for space bar
            if key == keyboard.Key.space:
                self.success_flag = True
                print("\n✓ SPACE pressed - Episode marked as SUCCESS (reward = 1.0)")
            # Check for Enter key to interrupt
            elif key == keyboard.Key.enter:
                self.interrupt_flag = True
                print("\n⏹ ENTER pressed - Episode INTERRUPTED (ending now)")
            # Check for Delete key to pause/resume
            elif key == keyboard.Key.backspace:
                self.pause_flag = not self.pause_flag
                if self.pause_flag:
                    print("\n⏸ DELETE pressed - Episode PAUSED (press DELETE again to resume)")
                else:
                    print("\n▶ DELETE pressed - Episode RESUMED")
            # Check for 'r' key to reset
            elif hasattr(key, 'char') and key.char == 'r':
                self.success_flag = False
                print("\n↻ 'r' pressed - Success flag reset (reward = 0.0)")
        except AttributeError:
            pass
        
    def get_success_state(self):
        """Get current success state."""
        return self.success_flag
    
    def get_interrupt_state(self):
        """Get current interrupt state."""
        return self.interrupt_flag
    
    def get_pause_state(self):
        """Get current pause state."""
        return self.pause_flag
    
    def reset_success_state(self):
        """Reset success state."""
        self.success_flag = False
        
    def reset_interrupt_state(self):
        """Reset interrupt state."""
        self.interrupt_flag = False
    
    def reset_pause_state(self):
        """Reset pause state."""
        self.pause_flag = False
        
    def reset_all_states(self):
        """Reset all states."""
        self.success_flag = False
        self.interrupt_flag = False
        self.pause_flag = False
        
    def stop(self):
        """Stop the keyboard listener."""
        self.running = False
        if self.listener:
            self.listener.stop()
        print("✓ Keyboard success monitor stopped")

class Trossen_env : 
    def __init__(
            self, 
            follower_left_ip, 
            follower_right_ip, 
            control_dt, 
            model, 
            max_episode_steps, 
            
    ): 
        self.max_episode_steps = max_episode_steps
        self.follower_left_ip = follower_left_ip
        self.follower_right_ip = follower_right_ip
        self.control_dt = control_dt

        # State tracking for velocity estimation
        self.prev_poses = {}
        self.filtered_velocities = {}
        
        # Configuration parameters
        self.use_velocity_feedforward = False
        self.interpolation_mode = trossen_arm.InterpolationSpace.cartesian
        self.velocity_filter_alpha = 0.3
        self.model = self._parse_model(model_str=model)
        
        self._step_count = 0

        self.follower_left = self.initialize_robot(self.follower_left_ip, 'follower', 'follower_left')
        self.follower_right = self.initialize_robot(self.follower_right_ip, 'follower', 'follower_right')

        # Initialize cameras
        if STATION1 : 
            scene_cam_ips = ["420222071698", "218622273530"]  # ← as STRINGS
            self.scene_cam_ips = scene_cam_ips or [None, None]
            left_wrist_serial = "230322276713"
            right_wrist_serial = "230322273819"
            self.camera_pipelines = {}

        elif STATION2 : 
            scene_cam_ips = ["339222070421", "218622273297"]  # ← as STRINGS
            self.scene_cam_ips = scene_cam_ips or [None, None]
            left_wrist_serial = "230422271087"

            right_wrist_serial = "128422270155"
            self.camera_pipelines = {}
        
        # Background image capture thread and buffer
        self.image_buffer = {}
        self.image_buffer_lock = threading.Lock()
        self.capture_thread = None
        self.capture_thread_running = False
        
        self._init_cameras(left_wrist_serial, right_wrist_serial)
        self._start_background_capture()

        self.num_joints_left = self.follower_left.get_num_joints()
        self.num_joints_right = self.follower_right.get_num_joints()

        # Ensure both arms have the same number of joints
        assert self.num_joints_left == self.num_joints_right, \
            f"Left arm has {self.num_joints_left} joints but right arm has {self.num_joints_right} joints"

        self.num_joints = self.num_joints_left
        
        # Initialize keyboard success monitor (replaces GUI)
        self.success_monitor = KeyboardSuccessMonitor()
        self.success_monitor.start()
        # Give monitor time to initialize
        time.sleep(0.1)

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
                    # Resize to 128x128
                    img_resized = cv2.resize(img, (128, 128), interpolation=cv2.INTER_AREA)
                    if img_resized.ndim == 3 and img_resized.shape[2] == 3:
                        images[name] = img_resized
                    else:
                        images[name] = np.zeros((128, 128, 3), dtype=np.uint8)
                else:
                    images[name] = np.zeros((128, 128, 3), dtype=np.uint8)
            except RuntimeError:
                # Timeout or no frame
                images[name] = np.zeros((128, 128, 3), dtype=np.uint8)
        return images
    
    def _start_background_capture(self):
        """Start the background image capture thread."""
        self.capture_thread_running = True
        self.capture_thread = threading.Thread(target=self._background_capture_loop, daemon=True)
        self.capture_thread.start()
        print("✓ Background camera capture thread started")
    
    def _background_capture_loop(self):
        """Background thread that continuously captures images."""
        while self.capture_thread_running:
            try:
                # Capture images without blocking the main thread
                images = self._capture_images()
                
                # Update buffer with lock
                with self.image_buffer_lock:
                    self.image_buffer = images.copy()
                
                # Small sleep to prevent CPU spinning (capture runs independently)
                time.sleep(0.001)
            except Exception as e:
                print(f"⚠️  Error in background capture thread: {e}")
                time.sleep(0.1)
        
    def get_latest_images(self):
        with self.image_buffer_lock:
            if not self.image_buffer:
                return {name: np.zeros((128, 128, 3), dtype=np.uint8) 
                    for name in self.camera_pipelines.keys()}
            return self.image_buffer.copy()
    
    def stop_background_capture(self):
        """Stop the background capture thread."""
        self.capture_thread_running = False
        if self.capture_thread:
            self.capture_thread.join(timeout=2.0)
        print("✓ Background camera capture thread stopped")
    
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
        
        # Send Cartesian command for arm with error handling
        try:
            if self.use_velocity_feedforward and velocity is not None:
                driver.set_cartesian_positions(
                    goal_positions=target_pose.tolist(),
                    interpolation_space=self.interpolation_mode,
                    goal_time = max(0.2, self.control_dt * 2.0),  # never go below 0.2s,  # Slightly longer than control period
                    goal_feedforward_velocities=velocity.tolist(),
                    blocking=False,   # Non-blocking for continuous control
                )
            else:
                driver.set_cartesian_positions(
                    goal_positions=target_pose.tolist(),
                    interpolation_space=self.interpolation_mode,
                    # goal_time = max(0.2, self.control_dt * 2.0),  # never go below 0.2s,
                    goal_time = self.control_dt * 5.0, 
                    blocking=False
                )
        except Exception as e:
            print(f"⚠️  Failed to send Cartesian command to {robot_id}: {e}")
            # Skip this command and wait for next action from policy
            return
        
        # Send gripper command separately with FAST response time
        # Use very short goal_time (0.05s = 50ms) for immediate gripper response
        try:
            driver.set_gripper_position(gripper_pos, goal_time=0.05, blocking=False)
        except Exception as e:
            print(f"⚠️  Failed to send gripper command to {robot_id}: {e}")
            # Skip gripper command if it fails
            return
    
    
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
        
    def move_followers_home(self, home_arm_left = np.zeros(6), home_arm_right = np.zeros(6), gripper_open=0.044) : 

        # Home state for arms
        left_state = np.concatenate([home_arm_left, [gripper_open]])
        right_state = np.concatenate([home_arm_right, [gripper_open]])
        
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


    def angle_axis_to_quaternion(self, aa):
        """
        Convert angle-axis to quaternion.

        Parameters
        ----------
        aa : array-like, shape (3,)
            Angle-axis vector (axis * angle in radians)

        Returns
        -------
        q : np.ndarray, shape (4,)
            Quaternion (w, x, y, z)
        """
        aa = np.asarray(aa, dtype=float)
        theta = np.linalg.norm(aa)

        if theta < 1e-8:
            # No rotation
            return np.array([1.0, 0.0, 0.0, 0.0])

        axis = aa / theta
        half_theta = theta / 2.0

        w = np.cos(half_theta)
        xyz = axis * np.sin(half_theta)

        return np.array([w, xyz[0], xyz[1], xyz[2]])
        

    def _get_observation(self):
        """Create observation dict with reward based on keyboard success state."""
        observation = OrderedDict()

        # assuming you know the number of joints
        follower_left_qpos = np.array([self.follower_left.get_joint_position(i) for i in range(self.num_joints)])
        follower_right_qpos = np.array([self.follower_right.get_joint_position(i) for i in range(self.num_joints)])

        # Get joint velocities
        follower_left_qvel = np.array([self.follower_left.get_joint_velocity(i) for i in range(self.num_joints)])
        follower_right_qvel = np.array([self.follower_right.get_joint_velocity(i) for i in range(self.num_joints)])

        # Joint states (16D)
        observation['qpos'] = np.concatenate([follower_left_qpos, follower_right_qpos])
        observation['qvel'] = np.concatenate([follower_left_qvel, follower_right_qvel])

        follower_left_pose = np.array(self.follower_left.get_cartesian_positions())
        follower_right_pose = np.array(self.follower_right.get_cartesian_positions())

        follower_left_gripper = np.array(self.follower_left.get_gripper_position())
        follower_right_gripper = np.array(self.follower_right.get_gripper_position())

        # End-effector position (6D: 3D per arm)
        observation['robot0_eef_pos'] = np.concatenate([follower_left_pose[:3], follower_right_pose[:3]])
        observation['robot0_eef_aa'] = np.concatenate([follower_left_pose[3:], follower_right_pose[3:]])
        
        # End-effector orientation (8D: 4D quat per arm, using identity as dummy)
        observation['robot0_eef_quat'] = np.concatenate([self.angle_axis_to_quaternion(follower_left_pose[3:]), 
                                                   self.angle_axis_to_quaternion(follower_right_pose[3:])])
        
        # Gripper positions (2D)
        observation['robot0_gripper_qpos'] = np.concatenate([[follower_left_gripper], [follower_right_gripper]])
        
        # Camera images (get latest from background thread - non-blocking)
        observation['images'] = self.get_latest_images()
        
        # Add reward based on keyboard success state
        observation['reward'] = 1.0 if self.success_monitor.get_success_state() else 0.0

        return observation
    
    def _get_velocities(self):
        """Create observation dict with reward based on keyboard success state."""
        observation = OrderedDict()

        follower_left_pose = np.array(self.follower_left.get_cartesian_velocities())
        follower_right_pose = np.array(self.follower_right.get_cartesian_velocities())

        # End-effector position (6D: 3D per arm)
        observation['velocity'] = np.concatenate([follower_left_pose, follower_right_pose])        

        return observation

    def reset(self):
        self.success_monitor.reset_all_states()  # Reset both success and interrupt flags
        print("🔄 Episode reset - all flags cleared")
        print("🏠 Moving the followers to home")
        self.move_followers_home()
        print("🏠 Finished moving robots to home")
        
        self._step_count = 0

        obs = self._get_observation()
        # First timestep is not "last"
        return TimeStep(observation=obs, reward=0.0, discount=1.0, step_type='mid')

    def step(self, action):
        loop_start = time.time()

        # Check if episode is paused - if so, wait until unpaused
        while self.success_monitor.get_pause_state():
            print("⏸ Episode paused - waiting for resume (press DELETE)...", end='\r')
            time.sleep(0.1)  # Check every 100ms if still paused
        
        # Clear the pause message line if we were paused
        print(" " * 80, end='\r')  # Clear the line

        self.send_cartesian_command(self.follower_left, action[:6], action[6], robot_id='follower_left')
        self.send_cartesian_command(self.follower_right, action[7:-1], action[-1], robot_id='follower_right')

        self._step_count += 1

        loop_time = time.time() - loop_start
        sleep_time = max(0, self.control_dt - loop_time)
        if sleep_time > 0:
            time.sleep(sleep_time) # sleep for the motion to happen. 

        observation = self._get_observation()
        reward = observation['reward']

        # Check for interrupt flag
        interrupted = self.success_monitor.get_interrupt_state()
        
        # Determine if episode should end
        success = self.success_monitor.get_success_state()
        truncated = (self._step_count >= self.max_episode_steps)
        done = success or truncated or interrupted  # Add interrupt condition

        step_type = 'last' if done else 'mid'
        return TimeStep(
            observation=observation,
            reward=reward,
            discount=1.0,
            step_type=step_type
        )

    def close(self):
        """Clean up resources."""
        self.stop_background_capture()
        self.success_monitor.stop()
        print("✓ Environment closed")