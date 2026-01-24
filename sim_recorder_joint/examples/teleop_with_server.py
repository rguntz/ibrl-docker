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
from trossen_arm_mujoco.sim_env import TransferCubeTask
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
        self.driver_left.configure(
            trossen_arm.Model.wxai_v0,
            trossen_arm.StandardEndEffector.wxai_v0_leader,
            self.leader_left_ip,
            False
        )
        
        # Right leader
        print(f"  Right at {self.leader_right_ip}...")
        self.driver_right = trossen_arm.TrossenArmDriver()
        self.driver_right.configure(
            trossen_arm.Model.wxai_v0,
            trossen_arm.StandardEndEffector.wxai_v0_leader,
            self.leader_right_ip,
            False
        )
        
        num_joints_left = self.driver_left.get_num_joints()
        num_joints_right = self.driver_right.get_num_joints()
        print(f"✓ Both leaders connected")
        print(f"  Left: {num_joints_left} joints, Right: {num_joints_right} joints")
        
        onscreen_render = True
        self.cam_list = ["cam_high", "cam_low", "cam_left_wrist", "cam_right_wrist"]

        # Create Mujoco env. 
        self.env = make_sim_env(
            TransferCubeTask,
            task_name="sim_transfer_cube",
            onscreen_render=onscreen_render,
            cam_list=self.cam_list,
        )


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

    
    def move_robots_to_home(self, gripper_open=0.044):
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
    
    def push_state_to_server(self, qpos, qvel, action, reward):
        """Send robot state + action to server"""
        try:
            data = {
                'qpos': qpos.tolist(),
                'qvel': qvel.tolist(),
                'action': action.tolist(), 
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
        
        self.move_robots_to_home()
        
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
    
        

    def _teleop_step(self):
        """Single teleop step - shared between viewer and headless modes"""

        # Combine for recording (16D state + 14D actions)
        qpos = self.ts.observation["qpos"]
        qvel = self.ts.observation["qvel"]

        # Capture cameras (every step)
        images = self.capture_cameras()
        
        # Get left arm state
        left_state = self.driver_left.get_all_positions()
        left_state = np.array(left_state)  # Convert VectorDouble to numpy

        # Get right arm state
        right_state = self.driver_right.get_all_positions()
        right_state = np.array(right_state)  # Convert VectorDouble to numpy

        # Concatenate into a single vector: left arm first, then right arm
        full_state_vector = np.concatenate([left_state, [left_state[-1]], right_state, [right_state[-1]]
        ])
        action = full_state_vector.copy()  # 16D total
        print("action shape : ", full_state_vector.shape, full_state_vector)
        # Apply to MuJoCo ctrl
        self.ts = self.env.step(full_state_vector)

        # Get the reward after the stepping function. 
        reward = 0.0 if self.ts.reward is None else self.ts.reward

        set_observation_images(self.ts.observation, self.plt_imgs, self.cam_list)
        
        # Push data camera to server
        for cam_name, image in images.items():
            self.push_frame_to_server(cam_name, image) 
        
        # Send state data to server for recording
        self.push_state_to_server(qpos, qvel, action, reward) 

        # Both are pushed at the same moment so they correspond to the same image-state-action triplet. 
        # Check server recording status to detect STOP event
        try:
            resp = requests.get(f"{self.server_url}/api/status", timeout=0.2)
            if resp.status_code == 200:
                remote_rec = bool(resp.json().get('recording', False))
                # If server just transitioned from recording->not-recording, move robots to home
                if self.prev_remote_recording and not remote_rec:
                    print("🎯 Detected remote STOP - moving robots to home...")
                    try:
                        self.move_robots_to_home()
                    except Exception as e:
                        print(f"⚠️  Failed to move robots to home after STOP: {e}")
                self.prev_remote_recording = remote_rec
        except requests.exceptions.Timeout:
            # ignore short timeouts to avoid blocking teleop loop
            pass
        except Exception as e:
            print(f"⚠️  Error checking server status: {e}")

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
