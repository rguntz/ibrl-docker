#!/usr/bin/env python3
"""
Test script for ZeroMQ teleop data transmission and recording
Simulates teleop data without requiring real robots
"""

import mujoco
import mujoco.viewer
import time
import numpy as np
import zmq
import threading
from collections import deque
import base64
from pathlib import Path

# Path to MuJoCo XML
XML_PATH_OPTIONS = [
    Path(__file__).parent.parent / "assets" / "trossen_ai_scene_joint.xml",
    Path(__file__).parent.parent.parent / "trossen_sim" / "trossen_sim" / "envs" / "xmls" / "trossen_ai_scene_joint.xml",
]

XML_PATH = None
for path in XML_PATH_OPTIONS:
    if path.exists():
        XML_PATH = path
        break

if XML_PATH is None:
    print(f"❌ ERROR: MuJoCo XML not found")
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

    def get_all(self):
        """Get all items and clear queue"""
        with self.lock:
            items = list(self.queue)
            self.queue.clear()
            return items


class TeleopSimulator:
    """Simulates teleop data transmission"""

    def __init__(self, zmq_port=5556, duration=10):
        self.zmq_port = zmq_port
        self.duration = duration

        # Camera setup
        self.camera_names = ['cam_high', 'cam_low', 'cam_left_wrist', 'cam_right_wrist']
        self.camera_resolution = (640, 480)

        # ZeroMQ setup
        self.zmq_context = zmq.Context()
        self.zmq_socket = self.zmq_context.socket(zmq.PUB)
        self.zmq_socket.connect(f"tcp://localhost:{zmq_port}")

        # Data queue
        self.data_queue = BoundedDataQueue(max_size=30, sample_rate=30)

        # Transmission thread
        self.transmission_thread = None
        self.transmission_active = False
        self.sequence_counter = 0

        # MuJoCo setup
        self.mj_model = None
        self.mj_data = None
        self.renderer = None

    def initialize(self):
        """Initialize MuJoCo simulation"""
        print("🎮 Initializing MuJoCo simulation...")

        self.mj_model = mujoco.MjModel.from_xml_path(str(XML_PATH))
        self.mj_data = mujoco.MjData(self.mj_model)
        self.renderer = mujoco.Renderer(self.mj_model, *self.camera_resolution)

        print("✅ MuJoCo initialized")
        return True

    def capture_cameras(self):
        """Capture camera images"""
        images = {}
        for cam_name in self.camera_names:
            try:
                cam_id = mujoco.mj_name2id(self.mj_model, mujoco.mjtObj.mjOBJ_CAMERA, cam_name)
                self.renderer.update_scene(self.mj_data, camera=cam_id)
                image = self.renderer.render()
                images[cam_name] = image.copy()
            except Exception as e:
                print(f"⚠️ Failed to capture {cam_name}: {e}")
        return images

    def start_data_transmission(self):
        """Start async data transmission"""
        if not self.transmission_active:
            self.transmission_active = True
            self.transmission_thread = threading.Thread(target=self._transmission_loop, daemon=True)
            self.transmission_thread.start()
            print(f"📡 Data transmission started on port {self.zmq_port}")

    def stop_data_transmission(self):
        """Stop data transmission"""
        self.transmission_active = False
        if self.transmission_thread:
            self.transmission_thread.join(timeout=1.0)

    def _transmission_loop(self):
        """Async transmission thread"""
        while self.transmission_active:
            data_packets = self.data_queue.get_all()
            for packet in data_packets:
                try:
                    self.zmq_socket.send_json(packet)
                except Exception as e:
                    print(f"⚠️ ZeroMQ send failed: {e}")
            time.sleep(0.01)

    def queue_teleop_data(self, cameras, robot_state, action):
        """Queue data packet for transmission"""
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

    def run_simulation(self):
        """Run simulation and send data"""
        if not self.initialize():
            return

        print(f"🎯 Running simulation for {self.duration} seconds...")
        print("📡 Sending teleop data via ZeroMQ")

        self.start_data_transmission()

        start_time = time.time()
        step_count = 0

        try:
            while time.time() - start_time < self.duration:
                step_start = time.time()

                # Simulate some robot movement (sine wave)
                t = time.time() - start_time
                left_arm = 0.1 * np.sin(t) * np.ones(6)  # 6D arm
                right_arm = 0.1 * np.cos(t) * np.ones(6)
                left_gripper = 0.5 + 0.1 * np.sin(t)
                right_gripper = 0.5 + 0.1 * np.cos(t)

                # Apply to MuJoCo
                self.mj_data.ctrl[0:6] = left_arm      # left arm
                self.mj_data.ctrl[6] = left_gripper    # left gripper carriage
                self.mj_data.ctrl[7] = left_gripper    # left gripper
                self.mj_data.ctrl[8:14] = right_arm    # right arm
                self.mj_data.ctrl[14] = right_gripper  # right gripper carriage
                self.mj_data.ctrl[15] = right_gripper  # right gripper

                # Step simulation
                mujoco.mj_step(self.mj_model, self.mj_data)

                # Get robot states
                left_qpos = self.mj_data.qpos[3:11].copy()  # Skip cube, get 8 joints
                right_qpos = self.mj_data.qpos[11:19].copy()
                qpos = np.concatenate([left_qpos, right_qpos])

                left_qvel = self.mj_data.qvel[3:11].copy()
                right_qvel = self.mj_data.qvel[11:19].copy()
                qvel = np.concatenate([left_qvel, right_qvel])

                # Create action (14D: left 7 + right 7)
                action = np.concatenate([
                    left_arm, [left_gripper],
                    right_arm, [right_gripper]
                ])

                # Capture cameras
                images = self.capture_cameras()

                # Convert to base64
                camera_data = {}
                for cam_name, image in images.items():
                    img_bytes = image.tobytes()
                    camera_data[cam_name] = {
                        'data': base64.b64encode(img_bytes).decode('utf-8'),
                        'shape': image.shape,
                        'dtype': str(image.dtype)
                    }

                robot_state = {
                    'qpos': qpos.tolist(),
                    'qvel': qvel.tolist()
                }

                # Queue data packet
                self.queue_teleop_data(camera_data, robot_state, action)

                step_count += 1
                if step_count % 100 == 0:
                    elapsed = time.time() - step_start
                    print(f"✓ Step {step_count} ({1.0/elapsed:.0f} Hz)")

                # Sleep to maintain ~500Hz
                time.sleep(0.002)

        except KeyboardInterrupt:
            print("\n⚠️ Interrupted by user")

        finally:
            print("\n🛑 Stopping simulation...")
            self.stop_data_transmission()

            # Close ZeroMQ
            self.zmq_socket.close()
            self.zmq_context.term()

            if self.renderer:
                self.renderer.close()

        print(f"✅ Simulation complete: {step_count} steps in {time.time() - start_time:.1f}s")


def main():
    import argparse
    parser = argparse.ArgumentParser(description='Test ZeroMQ teleop data transmission')
    parser.add_argument('--duration', type=int, default=10, help='Simulation duration in seconds')
    parser.add_argument('--zmq-port', type=int, default=5556, help='ZeroMQ port')

    args = parser.parse_args()

    simulator = TeleopSimulator(duration=args.duration, zmq_port=args.zmq_port)
    simulator.run_simulation()


if __name__ == '__main__':
    main()