#!/usr/bin/env python3
"""
Dual Robot Teleoperation to MuJoCo Sim
2 Real leader robots → 2 MuJoCo follower robots

Reads joint states from 2 physical leader robots and updates
2 sim robots in MuJoCo using direct qpos injection.
"""

import mujoco
import mujoco.viewer
import time
import numpy as np
import trossen_arm
from pathlib import Path

# Path to your MuJoCo XML with dual robots
XML_PATH = Path(__file__).parent.parent / "trossen_sim" / "trossen_sim" / "envs" / "xmls" / "trossen_ai_scene_joint.xml"


class DualRobotTeleopSim:
    def __init__(self, 
                 leader_left_ip: str = '192.168.1.2',
                 leader_right_ip: str = '192.168.1.4',
                 model: str = 'wxai_v0'):
        """
        Initialize dual robot teleop to sim.
        
        Args:
            leader_left_ip: IP of left leader robot
            leader_right_ip: IP of right leader robot
            model: Robot model name
        """
        self.leader_left_ip = leader_left_ip
        self.leader_right_ip = leader_right_ip
        
        # Set model
        if model == 'wxai_v0':
            self.model = trossen_arm.Model.wxai_v0
        else:
            raise ValueError(f"Unknown model: {model}")
        
        # Initialize leader drivers
        self.driver_left = None
        self.driver_right = None
        
        # MuJoCo sim components
        self.mj_model = None
        self.mj_data = None
        self.viewer = None
        
        # Joint indices in MuJoCo
        # Adjust these based on your XML joint order
        self.left_joint_start_idx = 0   # Left robot starts at qpos[0]
        self.right_joint_start_idx = 8  # Right robot starts at qpos[8] (6 joints + 2 gripper joints)
        self.num_joints = 7  # 6 arm joints + 1 gripper (trossen_arm format)
        
    def initialize_leaders(self):
        """Initialize both physical leader robots"""
        print(f"Connecting to leader robots...")
        
        # Left leader
        print(f"  Left at {self.leader_left_ip}...")
        self.driver_left = trossen_arm.TrossenArmDriver()
        self.driver_left.configure(
            self.model,
            trossen_arm.StandardEndEffector.wxai_v0_leader,
            self.leader_left_ip,
            False
        )
        
        # Right leader
        print(f"  Right at {self.leader_right_ip}...")
        self.driver_right = trossen_arm.TrossenArmDriver()
        self.driver_right.configure(
            self.model,
            trossen_arm.StandardEndEffector.wxai_v0_leader,
            self.leader_right_ip,
            False
        )
        
        print(f"✓ Both leaders connected")
        print(f"  Left joints: {self.driver_left.get_num_joints()}")
        print(f"  Right joints: {self.driver_right.get_num_joints()}")
    
    def initialize_sim(self):
        """Initialize MuJoCo simulation with dual robots"""
        print(f"Loading MuJoCo model from {XML_PATH}...")
        
        if not XML_PATH.exists():
            raise FileNotFoundError(f"XML file not found: {XML_PATH}")
        
        # Load MuJoCo model
        self.mj_model = mujoco.MjModel.from_xml_path(str(XML_PATH)) # XML_PATH points to a MuJoCo XML file. This file describes your entire simulation scene, including:
        self.mj_data = mujoco.MjData(self.mj_model) # MjData is the dynamic state of the simulation for that model.

        # Spawn cube at random position
        self._randomize_cube()
        
        # Launch passive viewer
        self.viewer = mujoco.viewer.launch_passive(self.mj_model, self.mj_data)
        
        # Set initial camera to left wrist for better view
        self._set_initial_camera()
        
        print("✓ MuJoCo sim loaded")
        print(f"  Num qpos: {self.mj_model.nq}")
        print(f"  Num ctrl: {self.mj_model.nu}")
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
        """Randomize cube position"""
        try:
            cube_joint_id = mujoco.mj_name2id(self.mj_model, mujoco.mjtObj.mjOBJ_JOINT, "cube_joint")
            cube_qpos_addr = self.mj_model.jnt_qposadr[cube_joint_id]
            
            # Random position in center
            x = np.random.uniform(-0.1, 0.2)
            y = np.random.uniform(-0.15, 0.15)
            z = 0.0125  # Table height
            
            self.mj_data.qpos[cube_qpos_addr:cube_qpos_addr+3] = [x, y, z]
            print(f"✓ Cube spawned at [{x:.3f}, {y:.3f}, {z:.3f}]")
            
        except Exception as e:
            print(f"Warning: Could not randomize cube: {e}")
    
    def move_to_home(self):
        """Move both leaders to home position"""
        print("Moving leaders to home...")
        
        # Create 7D home position (6 arm + 1 gripper)
        home_positions = np.zeros(7)
        home_positions[1] = np.pi / 2
        home_positions[2] = np.pi / 2
        
        # Set to position mode
        self.driver_left.set_all_modes(trossen_arm.Mode.position)
        self.driver_right.set_all_modes(trossen_arm.Mode.position)
        
        # Move to home
        self.driver_left.set_all_positions(home_positions, 2.0, True)
        self.driver_right.set_all_positions(home_positions, 2.0, True)
        
        print("✓ Both leaders at home")
    
    def start_teleoperation(self, duration: float = 60.0):
        """
        Start dual robot teleoperation: 2 leaders → 2 sim followers.
        
        Args:
            duration: Duration in seconds
        """
        print(f"\n{'='*60}")
        print(f"DUAL ROBOT TELEOPERATION ACTIVE - {duration}s")
        print(f"{'='*60}")
        print("Move both leader robots to control sim robots")
        print("Press Ctrl+C to stop early\n")
        
        # Set leaders to external effort mode (free movement)
        zero_efforts = np.zeros(7)
        self.driver_left.set_all_modes(trossen_arm.Mode.external_effort)
        self.driver_left.set_all_external_efforts(zero_efforts, 0.0, False)
        
        self.driver_right.set_all_modes(trossen_arm.Mode.external_effort)
        self.driver_right.set_all_external_efforts(zero_efforts, 0.0, False)
        
        start_time = time.time()
        end_time = start_time + duration
        loop_count = 0

        step = 0
        
        print("ℹ️  Teleoperation will run for {:.0f} seconds".format(duration))
        print("    Press Ctrl+C to stop early\n")
        
        try:
            while time.time() < end_time and self.viewer.is_running():
                # 1. Read joint states from both leaders (7D each: 6 joints + 1 gripper)
                left_state = self.driver_left.get_all_positions()    # 7D
                right_state = self.driver_right.get_all_positions()  # 7D
                
                # Split into arm joints and gripper
                left_arm = left_state[:6]
                left_gripper = left_state[6]
                
                right_arm = right_state[:6]
                right_gripper = right_state[6]
                
                # 2. Inject into sim qpos
                # Left robot (qpos 0-5: arm, 6-7: gripper). NOTE: qpos is causing physics to break.
                self.mj_data.ctrl[0:6] = left_arm
                self.mj_data.ctrl[6] = left_gripper   # left/right_carriage_joint
                self.mj_data.ctrl[7] = left_gripper   # left/left_carriage_joint
                
                # Right robot (qpos 8-13: arm, 14-15: gripper). NOTE: qpos is causing physics to break.
                self.mj_data.ctrl[8:14] = right_arm
                self.mj_data.ctrl[14] = right_gripper  # right/right_carriage_joint
                self.mj_data.ctrl[15] = right_gripper  # right/left_carriage_joint
                
                # # Zero velocities for smooth visualization
                # self.mj_data.qvel[:] = 0
                
                # # 3. Forward kinematics
                # mujoco.mj_forward(self.mj_model, self.mj_data)
                

                # 3. Step physics forward (this applies forces, respects collisions)
                mujoco.mj_step(self.mj_model, self.mj_data)
                
                # 4. Update viewer
                self.viewer.sync()
                
                # Optional: Print contact info when gripper is closing
                #if loop_count % 100 == 0:
                    #self._print_contact_debug()
                
                # Sleep to match timestep
                time.sleep(self.mj_model.opt.timestep)
                
                loop_count += 1
                
                # Log every 5 seconds
                if step % 500 == 0:
                    elapsed = time.time() - start_time
                    remaining = duration - elapsed
        
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
        
        if self.driver_left:
            self.driver_left.cleanup()
        
        if self.driver_right:
            self.driver_right.cleanup()
        
        if self.viewer:
            self.viewer.close()
        
        print("✓ Cleanup complete")


def main():
    import argparse
    
    parser = argparse.ArgumentParser(description='Dual Robot Teleop to MuJoCo Sim')
    parser.add_argument('--left-ip', default='192.168.1.2', help='Left leader IP')
    parser.add_argument('--right-ip', default='192.168.1.4', help='Right leader IP')
    parser.add_argument('--duration', type=float, default=60.0, help='Duration (seconds)')
    parser.add_argument('--no-home', action='store_true', help='Skip home position')
    
    args = parser.parse_args()
    
    teleop = DualRobotTeleopSim(
        leader_left_ip=args.left_ip,
        leader_right_ip=args.right_ip
    )
    
    try:
        # Initialize
        teleop.initialize_leaders()
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
