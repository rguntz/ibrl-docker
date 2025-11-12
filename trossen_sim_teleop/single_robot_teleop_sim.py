#!/usr/bin/env python3
"""
Single Robot Teleoperation to MuJoCo Sim
Real leader robot → MuJoCo follower robot

Uses trossen_arm SDK to read joint states from leader robot
and updates MuJoCo sim robot directly using qpos injection.
"""

import mujoco
import mujoco.viewer
import time
import numpy as np
import trossen_arm
from pathlib import Path

# Path to your MuJoCo XML
XML_PATH = Path(__file__).parent.parent / "trossen_sim" / "trossen_sim" / "envs" / "xmls" / "trossen_ai_scene_joint.xml"


class SingleRobotTeleopSim:
    def __init__(self, leader_ip: str = '192.168.1.2', model: str = 'wxai_v0'):
        """
        Initialize single robot teleop to sim.
        
        Args:
            leader_ip: IP of physical leader robot
            model: Robot model name
        """
        self.leader_ip = leader_ip
        
        # Set model
        if model == 'wxai_v0':
            self.model = trossen_arm.Model.wxai_v0
        else:
            raise ValueError(f"Unknown model: {model}")
        
        # Initialize leader driver
        self.driver_leader = None
        
        # MuJoCo sim components
        self.mj_model = None
        self.mj_data = None
        self.viewer = None
        
        # Joint indices in MuJoCo (adjust based on your XML)
        # Assuming left robot joints are first in qpos
        self.sim_joint_start_idx = 0  # Starting index in qpos
        self.num_joints = 7  # 6 arm joints + 1 gripper (trossen_arm expects 7 total)
        
    def initialize_leader(self):
        """Initialize physical leader robot"""
        print(f"Connecting to leader robot at {self.leader_ip}...")
        
        self.driver_leader = trossen_arm.TrossenArmDriver()
        self.driver_leader.configure(
            self.model,
            trossen_arm.StandardEndEffector.wxai_v0_leader,
            self.leader_ip,
            False
        )
        
        print(f"✓ Leader connected")
        print(f"  Num joints: {self.driver_leader.get_num_joints()}")
        print(f"  Driver version: {self.driver_leader.get_driver_version()}")
    
    def initialize_sim(self):
        """Initialize MuJoCo simulation"""
        print(f"Loading MuJoCo model from {XML_PATH}...")
        
        if not XML_PATH.exists():
            raise FileNotFoundError(f"XML file not found: {XML_PATH}")
        
        # Load MuJoCo model
        self.mj_model = mujoco.MjModel.from_xml_path(str(XML_PATH))
        self.mj_data = mujoco.MjData(self.mj_model)
        
        # Spawn cube at random position
        self._randomize_cube()
        
        # Launch passive viewer (non-blocking)
        self.viewer = mujoco.viewer.launch_passive(self.mj_model, self.mj_data)
        
        # Set initial camera to left wrist for better view
        self._set_initial_camera()
        
        print("✓ MuJoCo sim loaded")
        print(f"  Model name: {self.mj_model.names}")
        print(f"  Num qpos: {self.mj_model.nq}")
        print(f"  Timestep: {self.mj_model.opt.timestep}")

        # Print camera controls
        self._print_camera_info()
        
        # Print actuator info for debugging
        self._print_actuator_info()
    
    def _print_actuator_info(self):
        """Print actuator configuration for debugging"""
        print("\n--- Actuator Configuration ---")
        for i in range(min(8, self.mj_model.nu)):  # Print first 8 (left robot)
            act_name = mujoco.mj_id2name(self.mj_model, mujoco.mjtObj.mjOBJ_ACTUATOR, i)
            kp = self.mj_model.actuator_gainprm[i][0] if self.mj_model.actuator_gainprm[i][0] != 0 else "default"
            kv = self.mj_model.actuator_gainprm[i][1] if self.mj_model.actuator_gainprm[i][1] != 0 else "default"
            print(f"  {act_name}: kp={kp}, kv={kv}")
        print("--- End Actuator Info ---\n")

    def _set_initial_camera(self):
        """Set initial camera to wrist view for better manipulation visibility"""
        try:
            # Try to set to left wrist camera (better for picking)
            cam_id = mujoco.mj_name2id(self.mj_model, mujoco.mjtObj.mjOBJ_CAMERA, "cam_left_wrist")
            if cam_id >= 0:
                self.viewer.cam.type = mujoco.mjtCamera.mjCAMERA_FIXED
                self.viewer.cam.fixedcamid = cam_id
                print("✓ Started with left wrist camera view")
        except:
            print("ℹ️  Using default free camera (use [ ] to switch cameras)")
    
    def _print_camera_info(self):
        """Print available cameras and controls"""
        print("\n--- Available Cameras ---")
        print("Camera Controls:")
        print("  [ ] - Switch between cameras (left/right bracket)")
        print("  0-9 - Jump to camera by index")
        print("  ESC - Toggle free camera (mouse control)")
        print("\nCameras in scene:")
        
        for i in range(self.mj_model.ncam):
            cam_name = mujoco.mj_id2name(self.mj_model, mujoco.mjtObj.mjOBJ_CAMERA, i)
            if cam_name:
                print(f"  [{i}] {cam_name}")
        
        print("--- End Camera Info ---\n")

    def _randomize_cube(self):
        """Randomize cube position in sim"""
        # Find cube joint (assuming it's named 'cube_joint' or similar)
        # Adjust based on your XML structure
        try:
            # Try to find cube joint by name
            cube_joint_id = mujoco.mj_name2id(self.mj_model, mujoco.mjtObj.mjOBJ_JOINT, "cube_joint")
            cube_qpos_addr = self.mj_model.jnt_qposadr[cube_joint_id]
            
            # Random position near left arm
            x = np.random.uniform(-0.1, 0.2)
            y = np.random.uniform(-0.15, 0.15)
            z = 0.0125  # Table height
            
            self.mj_data.qpos[cube_qpos_addr:cube_qpos_addr+3] = [x, y, z]
            print(f"✓ Cube spawned at [{x:.3f}, {y:.3f}, {z:.3f}]")
            
        except Exception as e:
            print(f"Warning: Could not randomize cube position: {e}")
    
    def move_to_home(self):
        """Move leader to home position"""
        print("Moving leader to home position...")
        
        # Create 7D home position (6 arm joints + 1 gripper)
        home_positions = np.zeros(7)
        home_positions[1] = np.pi / 2
        home_positions[2] = np.pi / 2
        # home_positions[6] = 0.044  # Gripper open
        
        self.driver_leader.set_all_modes(trossen_arm.Mode.position)
        self.driver_leader.set_all_positions(home_positions, 2.0, True)
        
        print("✓ Leader at home")
    
    def start_teleoperation(self, duration: float = 60.0):
        """
        Start teleoperation: leader → sim follower.
        
        Args:
            duration: Duration in seconds
        """
        print(f"\n{'='*60}")
        print(f"TELEOPERATION ACTIVE - {duration}s")
        print(f"{'='*60}")
        print("Move your leader robot to control sim robot")
        print("Press Ctrl+C to stop early\n")
        
        # Set leader to external effort mode (free movement)
        self.driver_leader.set_all_modes(trossen_arm.Mode.external_effort)
        self.driver_leader.set_all_external_efforts(np.zeros(7), 0.0, False)
        
        start_time = time.time()
        end_time = start_time + duration
        loop_count = 0
        
        print("ℹ️  Teleoperation will run for {:.0f} seconds".format(duration))
        print("    Press Ctrl+C to stop early\n")
        
        try:
            while time.time() < end_time and self.viewer.is_running():
                # 1. Read joint states from leader (returns 7D: 6 joints + 1 gripper)
                leader_state = self.driver_leader.get_all_positions()  # 7D array
                
                # Split into arm joints and gripper
                leader_arm_joints = leader_state[:6]   # First 6 are arm joints
                leader_gripper = leader_state[6]       # 7th is gripper
                
                # 2. Inject into sim qpos
                # Map leader joints to sim joints
                self.mj_data.ctrl[self.sim_joint_start_idx:self.sim_joint_start_idx+6] = leader_arm_joints
                
                # Set gripper position (in sim, gripper is split into 2 joints)
                # Index 6 = left/right_carriage_joint, index 7 = left/left_carriage_joint
                self.mj_data.ctrl[6] = leader_gripper  # right carriage
                self.mj_data.ctrl[7] = leader_gripper  # left carriage
                
                # # Zero out velocities (for smooth visualization)
                # self.mj_data.qvel[self.sim_joint_start_idx:self.sim_joint_start_idx+7] = 0
                
                # # 3. Forward kinematics to update sim state
                # mujoco.mj_forward(self.mj_model, self.mj_data)

                # 3. Step physics forward (this applies forces, respects collisions)
                mujoco.mj_step(self.mj_model, self.mj_data)
                
                # 4. Update viewer
                self.viewer.sync()

                # Optional: Print contact info when gripper is closing
                if loop_count % 100 == 0:
                    self._print_contact_debug()
                
                # Sleep to match sim timestep
                time.sleep(self.mj_model.opt.timestep)
                
                loop_count += 1
                
                # Log every 5 seconds
                if loop_count % 500 == 0:
                    elapsed = time.time() - start_time
                    remaining = duration - elapsed
                    print(f"Time remaining: {remaining:.1f}s | Loops: {loop_count}")
        
        except KeyboardInterrupt:
            print("\n✗ Interrupted by user!")
        except Exception as e:
            print(f"\n✗ Error during teleoperation: {e}")
            import traceback
            traceback.print_exc()
        finally:
            elapsed = time.time() - start_time
            print(f"\n✓ Teleoperation complete ({loop_count} loops in {elapsed:.1f}s)")

    def _print_contact_debug(self):
        """Print contact information for debugging gripper physics"""
        if self.mj_data.ncon > 0:
            cube_contacts = []
            for i in range(self.mj_data.ncon):
                contact = self.mj_data.contact[i]
                try:
                    geom1_name = mujoco.mj_id2name(self.mj_model, mujoco.mjtObj.mjOBJ_GEOM, contact.geom1) or f"geom{contact.geom1}"
                    geom2_name = mujoco.mj_id2name(self.mj_model, mujoco.mjtObj.mjOBJ_GEOM, contact.geom2) or f"geom{contact.geom2}"
                    
                    # Only print contacts involving cube or grippers
                    if 'box' in geom1_name or 'box' in geom2_name or \
                       'carriage' in geom1_name or 'carriage' in geom2_name or \
                       'finger' in geom1_name or 'finger' in geom2_name:
                        cube_contacts.append(f"{geom1_name} <-> {geom2_name}")
                except:
                    pass
            
            if cube_contacts:
                print(f"Contacts ({len(cube_contacts)}): {', '.join(cube_contacts[:3])}")  # Print first 3
    

    def cleanup(self):
        """Cleanup resources"""
        print("Cleaning up...")
        
        if self.driver_leader:
            self.driver_leader.cleanup()
        
        if self.viewer:
            self.viewer.close()
        
        print("✓ Cleanup complete")


def main():
    import argparse
    
    parser = argparse.ArgumentParser(description='Single Robot Teleop to MuJoCo Sim')
    parser.add_argument('--leader-ip', default='192.168.1.2', help='Leader robot IP')
    parser.add_argument('--duration', type=float, default=60.0, help='Teleop duration (seconds)')
    parser.add_argument('--no-home', action='store_true', help='Skip home position')
    
    args = parser.parse_args()
    
    teleop = SingleRobotTeleopSim(leader_ip=args.leader_ip)
    
    try:
        # Initialize
        teleop.initialize_leader()
        teleop.initialize_sim()
        
        # Move to home
        if not args.no_home:
            teleop.move_to_home()
            time.sleep(1)
        
        # Start teleoperation
        input("Press ENTER to start teleoperation...")
        teleop.start_teleoperation(duration=args.duration)
        
        return 0
        
    except KeyboardInterrupt:
        print("\n✗ Interrupted!")
        return 1
    except Exception as e:
        print(f"\n✗ Error: {e}")
        import traceback
        traceback.print_exc()
        return 1
    finally:
        teleop.cleanup()


if __name__ == '__main__':
    exit(main())
