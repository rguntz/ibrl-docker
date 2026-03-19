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
import h5py
import json
from datetime import datetime

# IMPORT CLASSES
from env.trossen_env import Trossen_env 
from sim_recorder_ee_real.examples.localrecorder import LocalRecorder

REAL_DATA = True

class TeleopWithServer:
    def __init__(self, 
                 leader_left_ip='192.168.1.2',
                 leader_right_ip='192.168.1.4',
                 server_url='http://localhost:5000',
                 visualize=True,
                 camera_names=None,
                 camera_resolution=(128, 128),
                 record_fps=15, 
                 control_frequency = 50, # 50 hertz to start
                 base_path = "data/dataset.hdf5"
                 ): #  here is where we set the resolution of the image. 
        
        self.leader_left_ip = leader_left_ip
        self.leader_right_ip = leader_right_ip
        self.server_url = server_url
        self.visualize = visualize
        self.control_frequency = control_frequency
        self.base_path = base_path
        
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
        
        # Local recorder (no more server-side recording)
        self.local_recorder = LocalRecorder(fps=record_fps, base_path = self.base_path)
        
        # Robot drivers
        self.driver_left = None
        self.driver_right = None
        self.mj_model = None
        self.mj_data = None
        self.viewer = None
        self.renderer = None


        self.total_position = []
        
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
            control_dt = 1.0 / self.control_frequency
            model = "wxai_v0"
            max_number_steps = 1000
            self.env = Trossen_env(follower_left_ip, follower_right_ip, control_dt, model, max_episode_steps= max_number_steps)


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
                np.save("total_position.npy", self.total_position)
                print("saved position map")

    
        
    def _teleop_step(self):
        """Single teleop step - shared between viewer and headless modes"""

        # Combine for recording (16D state + 14D actions)
        qpos = self.ts.observation["qpos"]
        qvel = self.ts.observation["qvel"]
        robot0_eef_pos = self.ts.observation["robot0_eef_pos"]
        robot0_eef_aa = self.ts.observation["robot0_eef_aa"]
        robot0_eef_quat = self.ts.observation["robot0_eef_quat"]    
        robot0_gripper_qpos = self.ts.observation["robot0_gripper_qpos"]

        #self.total_position.append(robot0_eef_pos)

        # # Capture cameras (every step)
        images = self.capture_cameras()
        
        # Get left arm state
        left_state = np.array(self.driver_left.get_cartesian_positions())
        left_gripper = self.driver_left.get_gripper_position()

        # Get right arm state
        right_state = np.array(self.driver_right.get_cartesian_positions())
        right_gripper = self.driver_right.get_gripper_position()

        action = np.concatenate([
            left_state, [left_gripper],
            right_state, [right_gripper]
        ])

        # Apply to MuJoCo ctrl
        self.ts = self.env.step(action)

        # Get the reward after the stepping function. 
        reward = 0.0 if self.ts.reward is None else self.ts.reward

        print("reward : ", reward)

        set_observation_images(self.ts.observation, self.plt_imgs, self.cam_list)
                
        # Push data to local recorder if recording
        if self.local_recorder.is_recording():
            self.local_recorder.push_state(images, qpos, qvel, action, robot0_eef_pos, robot0_eef_quat, robot0_gripper_qpos, robot0_eef_aa, reward)

        # Check server recording status to detect START/STOP events
        try:
            resp = requests.get(f"{self.server_url}/api/status", timeout=0.2)
            if resp.status_code == 200:
                remote_rec = bool(resp.json().get('recording', False))
                # If server just transitioned to recording, start local recording
                if not self.local_recorder.is_recording() and remote_rec:
                    print("🔴 Detected START signal - starting local recording...")
                    self.local_recorder.start_recording()
                # If server just transitioned from recording->not-recording, stop local recording
                elif self.local_recorder.is_recording() and not remote_rec:
                    print("⏹️  Detected STOP signal - stopping local recording...")
                    self.local_recorder.stop_recording()
                    try:
                        self.move_robots_to_home()
                    except Exception as e:
                        print(f"⚠️  Failed to move robots to home after STOP: {e}")
        except requests.exceptions.Timeout:
            # ignore short timeouts to avoid blocking teleop loop
            pass
        except Exception as e:
            print(f"⚠️  Error checking server status: {e}")

        return True
    
    
    def cleanup(self):
        """Cleanup resources"""
        # Stop local recorder if still recording
        if self.local_recorder.is_recording():
            print("⏹️  Stopping local recorder on cleanup...")
            self.local_recorder.stop_recording()
        
        # Stop background capture thread in env if it exists
        if hasattr(self, 'env') and hasattr(self.env, 'stop_background_capture'):
            try:
                self.env.stop_background_capture()
            except Exception as e:
                print(f"⚠️  Error stopping background capture: {e}")
        
        if self.driver_left:
            # TrossenArmDriver doesn't have disconnect, just delete the object
            self.driver_left = None
            print("✓ Left leader disconnected")
        
        if self.driver_right:
            self.driver_right = None
            print("✓ Right leader disconnected")
        
        if self.renderer:
            self.renderer.close()

        
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
    parser.add_argument('--record-fps', type=float, default=15,
                       help='Recording frequency (FPS) for data collection')
    parser.add_argument('--base-path', type=str, default="sim_recorder_ee_real/server/data/dataset.hdf5",
                        help='base path to store the dataset')
    
    args = parser.parse_args()
    
    teleop = TeleopWithServer(
        leader_left_ip=args.leader_left_ip,
        leader_right_ip=args.leader_right_ip,
        server_url=args.server_url,
        visualize=not args.no_visualize,
        record_fps=args.record_fps, 
        control_frequency= 20, 
        base_path  =args.base_path
    )
    
    teleop.run_teleop()


if __name__ == '__main__':
    main()
