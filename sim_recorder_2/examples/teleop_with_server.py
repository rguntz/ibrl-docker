#!/usr/bin/env python3
"""
MuJoCo Teleop with ZeroMQ Data Transmission
Runs MuJoCo viewer + sends complete teleop data packets to recording server
Server controls when to record (via START/STOP buttons in web UI)
"""

import mujoco
import mujoco.viewer
import time
import numpy as np
import trossen_arm
from pathlib import Path
import zmq
import threading
from collections import deque
import base64

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


class BoundedDataQueue:
    """Thread-safe bounded queue with sampling rate control"""
    
    def __init__(self, max_size=50, sample_rate=30):
        self.queue = deque(maxlen=max_size)
        self.sample_rate = sample_rate
        self.last_sample_time = 0
        self.lock = threading.Lock()
        
    def put(self, data):
        """Add data if enough time has passed since last sample"""
        now = time.time()
        if now - self.last_sample_time >= 1.0 / self.sample_rate:
            with self.lock:
                self.queue.append(data)
                self.last_sample_time = now
            return True
        return False
    
    def get(self):
        """Get oldest item from queue"""
        with self.lock:
            return self.queue.popleft() if self.queue else None
    
    def get_all(self):
        """Get all items and clear queue"""
        with self.lock:
            items = list(self.queue)
            self.queue.clear()
            return items
    
    def size(self):
        """Get current queue size"""
        with self.lock:
            return len(self.queue)


class TeleopWithServer:
    def __init__(self, 
                 leader_left_ip='192.168.1.2',
                 leader_right_ip='192.168.1.3',
                 visualize=True,
                 camera_names=None,
                 camera_resolution=(480, 640),  # Higher resolution for teleop
                 zmq_port=5556):
        
        self.leader_left_ip = leader_left_ip
        self.leader_right_ip = leader_right_ip
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
        self.zmq_port = zmq_port
        
        # ZeroMQ setup for data transmission
        self.zmq_context = zmq.Context()
        self.zmq_socket = self.zmq_context.socket(zmq.PUB)
        self.zmq_socket.connect(f"tcp://localhost:{zmq_port}")
        
        # Data queue for async transmission
        self.data_queue = BoundedDataQueue(max_size=30, sample_rate=30)  # 30fps
        
        # Transmission thread
        self.transmission_thread = None
        self.transmission_active = False
        
        # Sequence counter for data packets
        self.sequence_counter = 0
        
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
        self.renderer = mujoco.Renderer(self.mj_model, *self.camera_resolution)
        
        # Check ZeroMQ connection to server
        print(f"🌐 Testing ZeroMQ connection to server on port {self.zmq_port}...")
        try:
            # Test ZeroMQ connection by sending a test message
            test_context = zmq.Context()
            test_socket = test_context.socket(zmq.PUB)
            test_socket.connect(f"tcp://localhost:{self.zmq_port}")
            test_socket.send_json({"test": True})
            test_socket.close()
            test_context.term()
            print(f"✓ ZeroMQ connection established")
        except Exception as e:
            print(f"⚠️  Cannot connect to server via ZeroMQ: {e}")
            print(f"   Make sure Flask server is running: cd sim_recorder/server && python app.py")
            # Don't return False - ZeroMQ might still work even if test fails
        
        print("="*60)
        print("✅ Initialization complete!")
        if self.visualize:
            print(f"🎥 MuJoCo viewer will open now")
        else:
            print(f"🎥 MuJoCo viewer disabled - running headless")
        print(f"🔴 Click START in web UI to begin recording")
        print("="*60)
        return True
    
    def capture_cameras(self):
        """Capture all camera images"""
        images = {}
        for cam_name in self.camera_names:
            try:
                cam_id = mujoco.mj_name2id(self.mj_model, mujoco.mjtObj.mjOBJ_CAMERA, cam_name)
                self.renderer.update_scene(self.mj_data, camera=cam_id)
                image = self.renderer.render()
                images[cam_name] = image.copy()
            except Exception as e:
                print(f"⚠️  Failed to capture {cam_name}: {e}")
        
        return images
    
    def start_data_transmission(self):
        """Start the async data transmission thread"""
        if not self.transmission_active:
            self.transmission_active = True
            self.transmission_thread = threading.Thread(target=self._transmission_loop, daemon=True)
            self.transmission_thread.start()
            print(f"📡 Data transmission started on port {self.zmq_port}")
    
    def stop_data_transmission(self):
        """Stop the data transmission thread"""
        self.transmission_active = False
        if self.transmission_thread:
            self.transmission_thread.join(timeout=1.0)
    
    def _transmission_loop(self):
        """Async thread that transmits queued data via ZeroMQ"""
        while self.transmission_active:
            # Get all queued data
            data_packets = self.data_queue.get_all()
            
            # Send each packet
            for packet in data_packets:
                try:
                    self.zmq_socket.send_json(packet)
                except Exception as e:
                    print(f"⚠️  ZeroMQ send failed: {e}")
            
            # Small sleep to prevent busy waiting
            time.sleep(0.01)
    
    def queue_teleop_data(self, cameras, robot_state, action):
        """Queue complete teleop data packet for transmission"""
        packet = {
            'timestamp': time.time(),
            'sequence': self.sequence_counter,
            'cameras': cameras,
            'robot_state': robot_state,
            'action': action.tolist() if hasattr(action, 'tolist') else action,
            'metadata': {
                'frequency': self.data_queue.sample_rate,
                'camera_resolution': self.camera_resolution,
                'camera_names': self.camera_names
            }
        }
        
        self.sequence_counter += 1
        return self.data_queue.put(packet)
    
    def run_teleop(self):
        """Main teleop loop with viewer"""
        if not self.initialize():
            return
        
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
        
        # Start data transmission thread
        self.start_data_transmission()
        
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
        action = np.concatenate([left_state[:7], right_state[:7]])  # 14D total. [left_arm(6) + left_gripper(1) + right_arm(6) + right_gripper(1)]
        
        # Capture cameras (every step)
        images = self.capture_cameras()
        
        # Convert images to base64 for transmission
        camera_data = {}
        for cam_name, image in images.items():
            # Convert numpy array to bytes and then base64
            img_bytes = image.tobytes()
            camera_data[cam_name] = {
                'data': base64.b64encode(img_bytes).decode('utf-8'),
                'shape': image.shape,
                'dtype': str(image.dtype)
            }
        
        # Prepare robot state data
        robot_state = {
            'qpos': qpos.tolist(),
            'qvel': qvel.tolist()
        }
        
        # Queue complete data packet for async transmission
        self.queue_teleop_data(camera_data, robot_state, action)
        
        return True
    
    def cleanup(self):
        """Cleanup resources"""
        # Stop data transmission
        self.stop_data_transmission()
        
        if self.driver_left:
            # TrossenArmDriver doesn't have disconnect, just delete the object
            self.driver_left = None
            print("✓ Left leader disconnected")
        
        if self.driver_right:
            self.driver_right = None
            print("✓ Right leader disconnected")
        
        # Close ZeroMQ
        if hasattr(self, 'zmq_socket'):
            self.zmq_socket.close()
        if hasattr(self, 'zmq_context'):
            self.zmq_context.term()
        
        if self.renderer:
            self.renderer.close()
        
        print("✓ Cleanup complete")


def main():
    import argparse
    
    parser = argparse.ArgumentParser(description='MuJoCo Dual Robot Teleop with Server Recording')
    parser.add_argument('--leader-left-ip', type=str, default='192.168.1.3',
                       help='Left leader robot IP address')
    parser.add_argument('--leader-right-ip', type=str, default='192.168.1.5',
                       help='Right leader robot IP address')
    parser.add_argument('--no-visualize', action='store_true',
                       help='Run without MuJoCo viewer (headless mode for web UI only)')
    
    args = parser.parse_args()

    print("args ip : ", args.leader_left_ip, args.leader_right_ip)
    
    teleop = TeleopWithServer(
        leader_left_ip=args.leader_left_ip,
        leader_right_ip=args.leader_right_ip,
        visualize=not args.no_visualize
    )
    
    teleop.run_teleop()


if __name__ == '__main__':
    main()
