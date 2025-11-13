#!/usr/bin/env python3
"""
Integrated Teleop + Recording for SERL
Single script that:
1. Runs MuJoCo sim with real leader robot teleop
2. Records camera images + robot states (qpos, qvel) + actions
3. Saves episodes in SERL-compatible format (PKL or NPZ)

This is a standalone solution - no need for separate server/camera pusher.
"""

import mujoco
import mujoco.viewer
import time
import numpy as np
import trossen_arm
from pathlib import Path
import pickle as pkl
import datetime
import json
from typing import Optional, Dict, List
import copy

# Path to MuJoCo XML - try multiple locations
XML_PATH_OPTIONS = [
    # Option 1: From trossen_sim package
    Path(__file__).parent.parent.parent / "trossen_sim" / "trossen_sim" / "envs" / "xmls" / "trossen_ai_scene_joint.xml",
    # Option 2: From trossen_arm_mujoco package
    Path(__file__).parent.parent.parent / "trossen_arm_mujoco" / "trossen_arm_mujoco" / "assets" / "trossen_ai_scene_joint.xml",
]

# Find first existing XML
XML_PATH = None
for path in XML_PATH_OPTIONS:
    if path.exists():
        XML_PATH = path
        break

if XML_PATH is None:
    # Default to first option (will error later with helpful message)
    XML_PATH = XML_PATH_OPTIONS[0]


class TeleopRecorder:
    """
    Integrated teleoperation + recording system
    Records: cameras (images) + robot states (qpos, qvel) + actions (from leader)
    """
    
    def __init__(self, 
                 leader_ip: str = '192.168.1.2',
                 model: str = 'wxai_v0',
                 save_dir: str = 'recorded_episodes',
                 camera_names: Optional[List[str]] = None,
                 camera_resolution: tuple = (128, 128),
                 enable_web_ui: bool = False,
                 web_ui_port: int = 5000):
        """
        Args:
            leader_ip: Real leader robot IP
            model: Robot model
            save_dir: Directory to save recorded episodes
            camera_names: List of camera names to record
            camera_resolution: Image resolution (H, W)
            enable_web_ui: If True, launch Flask server for web monitoring
            web_ui_port: Port for web UI server
        """
        self.leader_ip = leader_ip
        self.save_dir = Path(save_dir)
        self.save_dir.mkdir(parents=True, exist_ok=True)
        self.enable_web_ui = enable_web_ui
        self.web_ui_port = web_ui_port
        self.flask_process = None
        
        # Camera configuration
        if camera_names is None:
            self.camera_names = ['cam_high', 'cam_low', 'cam_left_wrist', 'cam_right_wrist']
        else:
            self.camera_names = camera_names
        
        self.camera_resolution = camera_resolution
        
        # Robot model
        if model == 'wxai_v0':
            self.model = trossen_arm.Model.wxai_v0
        else:
            raise ValueError(f"Unknown model: {model}")
        
        # Components
        self.driver_leader = None
        self.mj_model = None
        self.mj_data = None
        self.viewer = None
        self.renderer = None  # For offscreen rendering
        
        # Recording state
        self.recording = False
        self.current_episode_data = None
        self.episode_count = 0
        
    def initialize(self):
        """Initialize leader robot and MuJoCo sim"""
        # Initialize leader
        print(f"Connecting to leader robot at {self.leader_ip}...")
        self.driver_leader = trossen_arm.TrossenArmDriver()
        self.driver_leader.configure(
            self.model,
            trossen_arm.StandardEndEffector.wxai_v0_leader,
            self.leader_ip,
            False
        )
        print(f"✓ Leader connected ({self.driver_leader.get_num_joints()} joints)")
        
        # Load MuJoCo sim
        print(f"Loading MuJoCo model from {XML_PATH}...")
        if not XML_PATH.exists():
            raise FileNotFoundError(f"XML not found: {XML_PATH}")
        
        self.mj_model = mujoco.MjModel.from_xml_path(str(XML_PATH))
        self.mj_data = mujoco.MjData(self.mj_model)
        
        # Create offscreen renderer for cameras
        self.renderer = mujoco.Renderer(
            self.mj_model,
            height=self.camera_resolution[0],
            width=self.camera_resolution[1]
        )
        
        # Launch viewer
        self.viewer = mujoco.viewer.launch_passive(self.mj_model, self.mj_data)
        
        # Randomize cube
        self._randomize_cube()
        
        print(f"✓ MuJoCo sim loaded")
        print(f"  Cameras: {self.camera_names}")
        print(f"  Resolution: {self.camera_resolution}")
        print(f"  Timestep: {self.mj_model.opt.timestep}")
        
        # Optionally start web UI server
        if self.enable_web_ui:
            self._start_web_ui_server()
    
    def _randomize_cube(self):
        """Randomize cube position"""
        try:
            cube_joint_id = mujoco.mj_name2id(self.mj_model, mujoco.mjtObj.mjOBJ_JOINT, "red_box_joint")
            cube_qpos_addr = self.mj_model.jnt_qposadr[cube_joint_id]
            
            x = np.random.uniform(-0.1, 0.2)
            y = np.random.uniform(-0.15, 0.15)
            z = 0.0125
            
            self.mj_data.qpos[cube_qpos_addr:cube_qpos_addr+3] = [x, y, z]
            print(f"✓ Cube at [{x:.3f}, {y:.3f}, {z:.3f}]")
        except Exception as e:
            print(f"Warning: Could not randomize cube: {e}")
    
    def _start_web_ui_server(self):
        """Start Flask web UI server in background"""
        import subprocess
        import sys
        
        server_script = Path(__file__).parent.parent / "server" / "app.py"
        if not server_script.exists():
            print(f"⚠️  Web UI server script not found: {server_script}")
            return
        
        # Start server in background
        try:
            self.flask_process = subprocess.Popen(
                [sys.executable, str(server_script)],
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                cwd=str(server_script.parent)
            )
            time.sleep(2)  # Give server time to start
            
            if self.flask_process.poll() is None:
                print(f"✓ Web UI server started at http://localhost:{self.web_ui_port}")
                print(f"  Open browser to monitor cameras in real-time")
            else:
                print(f"⚠️  Web UI server failed to start")
                self.flask_process = None
        except Exception as e:
            print(f"⚠️  Could not start web UI server: {e}")
            self.flask_process = None
    
    def _push_frame_to_server(self, cam_id: int, image: np.ndarray):
        """Push camera frame to web UI server (if enabled)"""
        if not self.enable_web_ui or self.flask_process is None:
            return
        
        try:
            from PIL import Image
            import io
            import requests
            
            # Convert to JPEG
            img = Image.fromarray(image)
            buf = io.BytesIO()
            img.save(buf, format='JPEG', quality=85)
            buf.seek(0)
            
            # POST to server
            requests.post(
                f"http://localhost:{self.web_ui_port}/api/frame/{cam_id}",
                data=buf.read(),
                headers={'Content-Type': 'image/jpeg'},
                timeout=0.1
            )
        except:
            pass  # Silently fail if server not responding
    
    def move_to_home(self):
        """Move leader to home position"""
        print("Moving leader to home...")
        home_positions = np.zeros(7)
        home_positions[1] = np.pi / 2
        home_positions[2] = np.pi / 2
        
        self.driver_leader.set_all_modes(trossen_arm.Mode.position)
        self.driver_leader.set_all_positions(home_positions, 2.0, True)
        print("✓ Leader at home")
    
    def capture_cameras(self) -> Dict[str, np.ndarray]:
        """Capture all camera images"""
        images = {}
        
        for i, cam_name in enumerate(self.camera_names):
            try:
                # Render camera
                self.renderer.update_scene(self.mj_data, camera=cam_name)
                pixels = self.renderer.render()
                
                # Store as numpy array (H, W, 3) uint8
                images[cam_name] = pixels.copy()
                
                # Push to web UI server if enabled
                self._push_frame_to_server(i, pixels)
                
            except Exception as e:
                print(f"Warning: Failed to render {cam_name}: {e}")
                # Create placeholder
                images[cam_name] = np.zeros(
                    (self.camera_resolution[0], self.camera_resolution[1], 3),
                    dtype=np.uint8
                )
        
        return images
    
    def get_robot_state(self) -> Dict[str, np.ndarray]:
        """Get robot state from MuJoCo"""
        # Extract robot joint states (first 8 qpos: 6 arm + 2 gripper)
        robot_qpos = self.mj_data.qpos[:8].copy()
        robot_qvel = self.mj_data.qvel[:8].copy()
        
        return {
            'qpos': robot_qpos,
            'qvel': robot_qvel
        }
    
    def start_recording(self, episode_name: Optional[str] = None):
        """Start recording new episode"""
        if self.recording:
            print("Already recording!")
            return
        
        if episode_name is None:
            timestamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
            episode_name = f"episode_{timestamp}"
        
        self.current_episode_data = {
            'name': episode_name,
            'start_time': time.time(),
            'observations': [],  # List of obs dicts per step
            'actions': [],       # List of actions per step
        }
        
        self.recording = True
        print(f"\n🔴 RECORDING STARTED: {episode_name}")
    
    def stop_recording(self) -> Optional[str]:
        """Stop recording and save episode"""
        if not self.recording:
            print("Not currently recording")
            return None
        
        self.recording = False
        
        # Finalize episode data
        self.current_episode_data['end_time'] = time.time()
        num_steps = len(self.current_episode_data['observations'])
        
        if num_steps == 0:
            print("⚠️  No data recorded (0 steps)")
            return None
        
        # Save episode
        episode_path = self._save_episode()
        
        print(f"✅ RECORDING STOPPED")
        print(f"   Steps: {num_steps}")
        print(f"   Saved to: {episode_path}")
        
        self.episode_count += 1
        
        return episode_path
    
    def _save_episode(self) -> str:
        """Save episode to disk in SERL-compatible format"""
        episode_name = self.current_episode_data['name']
        episode_dir = self.save_dir / f"{episode_name}"
        episode_dir.mkdir(parents=True, exist_ok=True)
        
        # Convert observations list to proper format
        # Each obs in list is {camera_name: image, 'qpos': qpos, 'qvel': qvel}
        num_steps = len(self.current_episode_data['observations'])
        
        # Prepare observation arrays
        obs_dict = {}
        
        # Extract cameras
        for cam_name in self.camera_names:
            cam_images = []
            for obs in self.current_episode_data['observations']:
                if cam_name in obs:
                    cam_images.append(obs[cam_name])
            
            if cam_images:
                obs_dict[cam_name] = np.array(cam_images)  # (T, H, W, 3)
        
        # Extract robot states
        qpos_list = [obs['qpos'] for obs in self.current_episode_data['observations']]
        qvel_list = [obs['qvel'] for obs in self.current_episode_data['observations']]
        
        obs_dict['qpos'] = np.array(qpos_list)  # (T, 8)
        obs_dict['qvel'] = np.array(qvel_list)  # (T, 8)
        
        # Save observations
        np.savez_compressed(
            episode_dir / 'observations.npz',
            **obs_dict
        )
        
        # Save actions
        actions_array = np.array(self.current_episode_data['actions'])  # (T, 7)
        np.save(episode_dir / 'actions.npy', actions_array)
        
        # Create metadata
        metadata = {
            'episode_name': episode_name,
            'num_steps': num_steps,
            'start_time': self.current_episode_data['start_time'],
            'end_time': self.current_episode_data['end_time'],
            'duration': self.current_episode_data['end_time'] - self.current_episode_data['start_time'],
            'cameras': self.camera_names,
            'camera_resolution': self.camera_resolution,
            'robot_model': str(self.model),
            'leader_ip': self.leader_ip
        }
        
        with open(episode_dir / 'meta.json', 'w') as f:
            json.dump(metadata, f, indent=2)
        
        print(f"  Saved observations: {[f'{k}: {v.shape}' for k, v in obs_dict.items()]}")
        print(f"  Saved actions: {actions_array.shape}")
        
        return str(episode_dir)
    
    def run_teleop_with_recording(self, 
                                  duration: float = 60.0,
                                  auto_record: bool = False,
                                  control_freq: float = 20.0):
        """
        Run teleoperation with optional recording
        
        Args:
            duration: Duration in seconds
            auto_record: If True, start recording automatically
            control_freq: Control loop frequency (Hz)
        """
        print(f"\n{'='*60}")
        print(f"TELEOPERATION ACTIVE")
        print(f"{'='*60}")
        print(f"Duration: {duration}s")
        print(f"Control freq: {control_freq} Hz")
        print(f"")
        print(f"Controls:")
        print(f"  SPACE - Toggle recording")
        print(f"  R     - Randomize cube")
        print(f"  Q/ESC - Quit")
        print(f"{'='*60}\n")
        
        # Set leader to external effort mode (free movement)
        zero_efforts = np.zeros(7)
        self.driver_leader.set_all_modes(trossen_arm.Mode.external_effort)
        self.driver_leader.set_all_external_efforts(zero_efforts, 0.0, False)
        
        if auto_record:
            self.start_recording()
        
        dt = 1.0 / control_freq
        start_time = time.time()
        end_time = start_time + duration
        loop_count = 0
        
        try:
            while time.time() < end_time and self.viewer.is_running():
                loop_start = time.time()
                
                # 1. Read leader joint positions (action)
                action = self.driver_leader.get_all_positions()  # 7D: 6 arm + 1 gripper
                
                # Convert to numpy array
                action_array = np.array(action)
                
                # 2. Update sim with action (via ctrl)
                arm_joints = action_array[:6]
                gripper = action_array[6]
                
                self.mj_data.ctrl[0:6] = arm_joints
                self.mj_data.ctrl[6] = gripper   # left gripper
                self.mj_data.ctrl[7] = gripper   # right gripper
                
                # 3. Step physics
                mujoco.mj_step(self.mj_model, self.mj_data)
                
                # 4. Update viewer
                self.viewer.sync()
                
                # 5. If recording, capture data
                if self.recording:
                    # Capture cameras
                    camera_images = self.capture_cameras()
                    
                    # Get robot state
                    robot_state = self.get_robot_state()
                    
                    # Combine into observation
                    obs = {**camera_images, **robot_state}
                    
                    # Store step
                    self.current_episode_data['observations'].append(obs)
                    self.current_episode_data['actions'].append(action_array.copy())
                
                # 6. Sleep to maintain frequency
                elapsed = time.time() - loop_start
                sleep_time = dt - elapsed
                if sleep_time > 0:
                    time.sleep(sleep_time)
                
                loop_count += 1
                
                # Print status
                if loop_count % int(control_freq * 5) == 0:  # Every 5 seconds
                    elapsed_total = time.time() - start_time
                    remaining = duration - elapsed_total
                    rec_status = f"REC ({len(self.current_episode_data['observations'])} steps)" if self.recording else "IDLE"
                    print(f"[{remaining:.1f}s remaining] {rec_status} | {loop_count} loops")
                
        except KeyboardInterrupt:
            print("\n✗ Interrupted by user")
        finally:
            if self.recording:
                print("\nAuto-saving recording...")
                self.stop_recording()
            
            elapsed = time.time() - start_time
            actual_freq = loop_count / elapsed if elapsed > 0 else 0
            print(f"\n✓ Teleoperation complete")
            print(f"  Total loops: {loop_count}")
            print(f"  Duration: {elapsed:.1f}s")
            print(f"  Actual freq: {actual_freq:.1f} Hz")
    
    def cleanup(self):
        """Cleanup resources"""
        if self.driver_leader:
            self.driver_leader.cleanup()
        if self.viewer:
            self.viewer.close()
        if self.flask_process:
            self.flask_process.terminate()
            self.flask_process.wait(timeout=5)
            print("✓ Web UI server stopped")
        print("✓ Cleanup complete")


def main():
    import argparse
    
    parser = argparse.ArgumentParser(description='Integrated Teleop + Recording')
    parser.add_argument('--leader-ip', default='192.168.1.2', help='Leader robot IP')
    parser.add_argument('--duration', type=float, default=60.0, help='Duration (seconds)')
    parser.add_argument('--save-dir', default='recorded_episodes', help='Save directory')
    parser.add_argument('--auto-record', action='store_true', help='Start recording immediately')
    parser.add_argument('--control-freq', type=float, default=20.0, help='Control frequency (Hz)')
    parser.add_argument('--no-home', action='store_true', help='Skip home position')
    parser.add_argument('--web-ui', action='store_true', help='Enable web UI server for monitoring')
    parser.add_argument('--web-ui-port', type=int, default=5000, help='Web UI port')
    
    args = parser.parse_args()
    
    recorder = TeleopRecorder(
        leader_ip=args.leader_ip,
        save_dir=args.save_dir,
        enable_web_ui=args.web_ui,
        web_ui_port=args.web_ui_port
    )
    
    try:
        # Initialize
        recorder.initialize()
        
        # Move to home
        if not args.no_home:
            recorder.move_to_home()
            time.sleep(1)
        
        # Start teleoperation (skip input if auto-record)
        if not args.auto_record:
            print("\nPress ENTER to start teleoperation...")
            input()
        
        recorder.run_teleop_with_recording(
            duration=args.duration,
            auto_record=args.auto_record,
            control_freq=args.control_freq
        )
        
        return 0
        
    except Exception as e:
        print(f"\n✗ Error: {e}")
        import traceback
        traceback.print_exc()
        return 1
    finally:
        recorder.cleanup()


if __name__ == '__main__':
    exit(main())
