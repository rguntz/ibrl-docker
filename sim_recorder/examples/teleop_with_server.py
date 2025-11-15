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

# Path to MuJoCo XML
XML_PATH_OPTIONS = [
    Path(__file__).parent.parent / "assets" / "trossen_ai_scene_joint.xml",  # sim_recorder local copy
    Path(__file__).parent.parent.parent / "trossen_sim" / "trossen_sim" / "envs" / "xmls" / "trossen_ai_scene_joint.xml",
    Path(__file__).parent.parent.parent / "trossen_arm_mujoco" / "trossen_arm_mujoco" / "assets" / "trossen_ai_scene_joint.xml",
]

XML_PATH = None
for path in XML_PATH_OPTIONS:
    if path.exists():
        XML_PATH = path
        break

if XML_PATH is None:
    print(f"❌ ERROR: MuJoCo XML not found in any of these locations:")
    for p in XML_PATH_OPTIONS:
        print(f"   - {p}")
    exit(1)


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
        
        # Load MuJoCo model
        print(f"🎮 Loading MuJoCo model: {XML_PATH.name}")
        self.mj_model = mujoco.MjModel.from_xml_path(str(XML_PATH))
        self.mj_data = mujoco.MjData(self.mj_model)
        
        # Get cube position
        cube_pos = self.mj_data.qpos[:3]
        print(f"✓ Cube at [{cube_pos[0]:.3f}, {cube_pos[1]:.3f}, {cube_pos[2]:.3f}]")
        print(f"✓ MuJoCo sim loaded")
        
        # Setup renderer for cameras
        self.renderer = mujoco.Renderer(self.mj_model, *self.camera_resolution) # Here we set the render to the camera resolution that we want. 
        
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
    
    def _randomize_cube(self):
        """Randomize cube position, keep orientation fixed, zero velocities"""
        try:
            cube_joint_id = mujoco.mj_name2id(
                self.mj_model,
                mujoco.mjtObj.mjOBJ_JOINT,
                "red_box_joint"
            )
            qpos_addr = self.mj_model.jnt_qposadr[cube_joint_id]
            qvel_addr = self.mj_model.jnt_dofadr[cube_joint_id]

            # -----------------------------
            # Randomize position (x, y), fixed z
            # -----------------------------
            x = np.random.uniform(-0.1, 0.2)
            y = np.random.uniform(-0.15, 0.15)
            z = 0.0125

            # -----------------------------
            # Fixed orientation (identity quaternion)
            # -----------------------------
            # Orientation = [1, 0, 0, 0] = no rotation
            quat = np.array([1.0, 0.0, 0.0, 0.0])

            # Write full qpos for free joint: [x, y, z, qw, qx, qy, qz]
            self.mj_data.qpos[qpos_addr:qpos_addr+7] = [x, y, z, *quat]

            # -----------------------------
            # Zero linear + angular velocity
            # -----------------------------
            self.mj_data.qvel[qvel_addr:qvel_addr+6] = 0.0

            print(f"✓ Cube moved to [{x:.3f}, {y:.3f}, {z:.3f}], orientation fixed, speed reset")

        except Exception as e:
            print(f"Warning: Could not randomize cube: {e}")
    
    def move_robots_to_home(self, gripper_open=0.04):
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
        
        # Move MuJoCo simulation
        # Left robot: ctrl 0-5 arm, 6-7 gripper
        self.mj_data.ctrl[0:6] = 0.0
        self.mj_data.ctrl[6] = gripper_open
        self.mj_data.ctrl[7] = gripper_open
        
        # Right robot: ctrl 8-13 arm, 14-15 gripper
        self.mj_data.ctrl[8:14] = 0.0
        self.mj_data.ctrl[14] = gripper_open
        self.mj_data.ctrl[15] = gripper_open

        self._randomize_cube()
        
        # Step simulation to update scene
        for _ in range(100):
            mujoco.mj_step(self.mj_model, self.mj_data)

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
        """Capture all camera images (mirror cam_high and cam_low across vertical axis)"""
        images = {}
        for cam_name in self.camera_names:
            try:
                cam_id = mujoco.mj_name2id(self.mj_model, mujoco.mjtObj.mjOBJ_CAMERA, cam_name)
                self.renderer.update_scene(self.mj_data, camera=cam_id)
                image = self.renderer.render()

                # Mirror across vertical axis (left-right flip)
                if cam_name in ('cam_high', 'cam_low'):
                    # image shape is (H, W, C) so flip along axis=1 (width)
                    image = np.flip(image, axis=1)
                    # alternative: image = image[:, ::-1, :]

                images[cam_name] = image.copy()

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
    
    def push_state_to_server(self, qpos, qvel, action):
        """Send robot state + action to server"""
        try:
            data = {
                'qpos': qpos.tolist(),
                'qvel': qvel.tolist(),
                'action': action.tolist()
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
                with mujoco.viewer.launch_passive(self.mj_model, self.mj_data) as viewer:
                    self._run_teleop_loop(viewer)
            else:
                # Run headless (no viewer)
                self._run_teleop_loop_headless()
        
        except KeyboardInterrupt:
            print("\n⚠️  Interrupted by user")
        
        finally:
            print("\n🛑 Stopping teleop...")
            self.cleanup()
    
    def _run_teleop_loop(self, viewer):
        """Main teleop loop with viewer""" 
        print("teleop  starting")
        step_count = 0
        
        while viewer.is_running():
            step_start = time.time()
            
            if not self._teleop_step():
                break
            
            # Update viewer
            viewer.sync()
            
            step_count += 1
            if step_count % 500 == 0:  # Log every 500 steps (~1 second at 500Hz)
                elapsed = time.time() - step_start
                print(f"✓ Step {step_count} (running at ~{1.0/elapsed:.0f} Hz)")
            
            # Sleep to match MuJoCo timestep (~0.002s = 500Hz) for smooth physics
            time.sleep(self.mj_model.opt.timestep)
    
    def _run_teleop_loop_headless(self):
        """Main teleop loop without viewer (for web UI only)"""
        step_count = 0
        
        try:
            while True:  # Run until interrupted
                step_start = time.time()
                
                if not self._teleop_step():
                    break
                
                step_count += 1
                if step_count % 500 == 0:  # Log every 500 steps (~1 second at 500Hz)
                    elapsed = time.time() - step_start
                    print(f"✓ Step {step_count} (running at ~{1.0/elapsed:.0f} Hz)")
                
                # Sleep to match MuJoCo timestep (~0.002s = 500Hz) for smooth physics
                time.sleep(self.mj_model.opt.timestep)
                
        except KeyboardInterrupt:
            print("\n⚠️  Headless loop interrupted")
    
    def _teleop_step(self):
        """Single teleop step - shared between viewer and headless modes"""
        try:
            # Get both leader robot states (7D each: 6 arm + 1 gripper)
            left_state = self.driver_left.get_all_positions()
            left_state = np.array(left_state)  # Convert VectorDouble to numpy
            
            right_state = self.driver_right.get_all_positions()
            right_state = np.array(right_state)  # Convert VectorDouble to numpy
            
            # Split into arm and gripper for each robot
            left_arm = left_state[:6]
            left_gripper = left_state[6]
            
            right_arm = right_state[:6]
            right_gripper = right_state[6]
            
            # Apply to MuJoCo ctrl
            # Left robot (ctrl 0-5: arm, 6-7: gripper)
            self.mj_data.ctrl[0:6] = left_arm
            self.mj_data.ctrl[6] = left_gripper   # left/right_carriage_joint
            self.mj_data.ctrl[7] = left_gripper   # left/left_carriage_joint
            
            # Right robot (ctrl 8-13: arm, 14-15: gripper)
            self.mj_data.ctrl[8:14] = right_arm
            self.mj_data.ctrl[14] = right_gripper  # right/right_carriage_joint
            self.mj_data.ctrl[15] = right_gripper  # right/left_carriage_joint
            
        except Exception as e:
            print(f"⚠️  Error reading leader positions: {e}")
            print("   Robots may have gone into error state. Stopping...")
            return False
        
        # Step simulation
        mujoco.mj_step(self.mj_model, self.mj_data)
        
        # Get robot states from MuJoCo (for recording - both robots)
        # Left robot: qpos[3:11] (skip cube, get 8 joints: 6 arm + 2 gripper)
        left_qpos = self.mj_data.qpos[3:11].copy()
        left_qvel = self.mj_data.qvel[3:11].copy()
        
        # Right robot: qpos[11:19] (8 joints after left robot)
        right_qpos = self.mj_data.qpos[11:19].copy()
        right_qvel = self.mj_data.qvel[11:19].copy()
        
        # Combine for recording (16D state + 14D actions)
        qpos = np.concatenate([left_qpos, right_qpos])
        qvel = np.concatenate([left_qvel, right_qvel])
        action = np.concatenate([left_state[:7], right_state[:7]])  # 14D total 
        # action is the left state which is what we read from the real robot. 
        
        # Capture cameras (every step)
        images = self.capture_cameras()
        
        # Push data to server
        for cam_name, image in images.items():
            self.push_frame_to_server(cam_name, image) 
        
        # TODO: Send state data to server for recording
        self.push_state_to_server(qpos, qvel, action) 

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
