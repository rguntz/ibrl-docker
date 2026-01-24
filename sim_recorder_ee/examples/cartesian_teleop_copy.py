#!/usr/bin/env python3
"""
Cartesian Space Teleoperation for Trossen Arms

Reads EE poses from leader(s) and controls follower(s) in Cartesian space.
Supports single or dual arm configuration.

Usage:
    # Single arm
    python cartesian_teleop.py --mode single --leader-ip 192.168.1.2 --follower-ip 192.168.1.3
    
    # Dual arm
    python cartesian_teleop.py --mode dual \
        --leader-left-ip 192.168.1.2 --leader-right-ip 192.168.1.3 \
        --follower-left-ip 192.168.1.4 --follower-right-ip 192.168.1.5
    
    # BC policy compatible - reads from leader, sends to follower
    python cartesian_teleop.py --mode single --leader-ip 192.168.1.2 --follower-ip 192.168.1.3 --frequency 20
"""

import argparse
import time
import numpy as np
import trossen_arm
from typing import Optional, Tuple


class CartesianTeleopController:
    """
    Smooth Cartesian space teleoperation controller.
    
    Features:
    - Velocity feedforward for smooth motion
    - Configurable control frequency
    - Dual or single arm support
    - Low-latency updates
    """
    
    def __init__(
        self,
        model: str = 'wxai_v0',
        control_frequency: float = 50.0,  # Hz
        interpolation_space: str = 'cartesian',  # 'cartesian' or 'joint'
        use_velocity_feedforward: bool = True,
        velocity_filter_alpha: float = 0.3,  # Exponential smoothing for velocity
    ):
        """
        Initialize the Cartesian teleop controller.
        
        Args:
            model: Robot model ('wxai_v0', 'vxai_v0', etc.)
            control_frequency: Control loop frequency in Hz
            interpolation_space: 'cartesian' for task-space, 'joint' for joint-space interpolation
            use_velocity_feedforward: Enable velocity feedforward for smoother tracking
            velocity_filter_alpha: Smoothing factor for velocity estimation (0-1, higher=less filtering)
        """
        self.model = self._parse_model(model)
        self.control_dt = 1.0 / control_frequency
        self.control_frequency = control_frequency
        
        # Set interpolation mode
        if interpolation_space == 'cartesian':
            self.interpolation_mode = trossen_arm.InterpolationSpace.cartesian
        else:
            self.interpolation_mode = trossen_arm.InterpolationSpace.joint
            
        self.use_velocity_feedforward = use_velocity_feedforward
        self.velocity_filter_alpha = velocity_filter_alpha
        
        # State tracking for velocity estimation
        self.prev_poses = {}
        self.filtered_velocities = {}
        
        print(f"CartesianTeleopController initialized:")
        print(f"  Model: {model}")
        print(f"  Control frequency: {control_frequency} Hz")
        print(f"  Interpolation: {interpolation_space}")
        print(f"  Velocity feedforward: {use_velocity_feedforward}")
    
    def _parse_model(self, model_str: str):
        """Parse model string to trossen_arm.Model enum."""
        if model_str == 'wxai_v0':
            return trossen_arm.Model.wxai_v0
        elif model_str == 'vxai_v0':
            return trossen_arm.Model.vxai_v0
        else:
            raise ValueError(f"Unknown model: {model_str}")
    
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
        
        driver.configure(self.model, ee_type, ip, False)
        
        # Set appropriate mode based on robot role
        if end_effector == 'leader':
            # Leaders: external effort mode (free movement, like joint teleop)
            # Don't set mode here - will be set when starting teleop
            print(f"✓ {robot_id} connected ({driver.get_num_joints()} joints)")
            print(f"  Role: LEADER (will be set to external effort mode)")
        else: # follower case
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
    
    def run_single_arm_teleop(
        self,
        leader_ip: str,
        follower_ip: str,
        duration: Optional[float] = None
    ):
        """
        Run single arm Cartesian teleoperation.
        
        Args:
            leader_ip: Leader robot IP
            follower_ip: Follower robot IP
            duration: Optional duration in seconds (None = run forever)
        """
        print("\n" + "="*60)
        print("SINGLE ARM CARTESIAN TELEOPERATION")
        print("="*60)
        
        # Initialize robots
        leader = self.initialize_robot(leader_ip, 'leader', 'leader')
        follower = self.initialize_robot(follower_ip, 'follower', 'follower')
        
        # Set leader to external effort mode (like joint teleop does)
        print("\n⚙ Setting leader to external effort mode (free movement)...")
        leader.set_all_modes(trossen_arm.Mode.external_effort)
        leader.set_all_external_efforts(np.zeros(7), 0.0, False)
        print("✓ Leader is now freely movable\n")
        
        print("✓ Starting teleoperation...")
        print(f"  Control frequency: {self.control_frequency} Hz")
        print(f"  Press Ctrl+C to stop\n")
        
        start_time = time.time()
        loop_count = 0
        
        try:
            while True:
                loop_start = time.time()
                
                # Read leader state (pose + gripper)
                leader_pose = np.array(leader.get_cartesian_positions())
                leader_gripper = leader.get_gripper_position()
                
                # Send to follower with velocity feedforward
                self.send_cartesian_command(
                    follower,
                    leader_pose,
                    leader_gripper,
                    robot_id='follower'
                )
                
                # Print status periodically
                if loop_count % (self.control_frequency * 2) == 0:  # Every 2 seconds
                    elapsed = time.time() - start_time
                    actual_freq = loop_count / elapsed if elapsed > 0 else 0
                    print(f"[{elapsed:.1f}s] Pose: [{leader_pose[0]:.3f}, {leader_pose[1]:.3f}, {leader_pose[2]:.3f}] | "
                          f"Freq: {actual_freq:.1f} Hz")
                
                # Check duration
                if duration and (time.time() - start_time) >= duration:
                    print(f"\n✓ Duration {duration}s reached")
                    break
                
                # Sleep to maintain control frequency
                loop_time = time.time() - loop_start
                sleep_time = max(0, self.control_dt - loop_time)
                if sleep_time > 0:
                    time.sleep(sleep_time)
                
                loop_count += 1
                
        except KeyboardInterrupt:
            print("\n\n✓ Teleoperation stopped by user")
        
        elapsed = time.time() - start_time
        avg_freq = loop_count / elapsed if elapsed > 0 else 0
        print(f"\nStatistics:")
        print(f"  Total time: {elapsed:.2f}s")
        print(f"  Total loops: {loop_count}")
        print(f"  Average frequency: {avg_freq:.1f} Hz")
    
    def run_dual_arm_teleop(
        self,
        leader_left_ip: str,
        leader_right_ip: str,
        follower_left_ip: str,
        follower_right_ip: str,
        duration: Optional[float] = None
    ):
        """
        Run dual arm Cartesian teleoperation.
        
        Args:
            leader_left_ip: Left leader robot IP
            leader_right_ip: Right leader robot IP
            follower_left_ip: Left follower robot IP
            follower_right_ip: Right follower robot IP
            duration: Optional duration in seconds
        """
        print("\n" + "="*60)
        print("DUAL ARM CARTESIAN TELEOPERATION")
        print("="*60)
        
        # Initialize robots
        leader_left = self.initialize_robot(leader_left_ip, 'leader', 'leader_left')
        leader_right = self.initialize_robot(leader_right_ip, 'leader', 'leader_right')
        follower_left = self.initialize_robot(follower_left_ip, 'follower', 'follower_left')
        follower_right = self.initialize_robot(follower_right_ip, 'follower', 'follower_right')
        
        # Set leaders to external effort mode (like joint teleop does)
        print("\n⚙ Setting leaders to external effort mode (free movement)...")
        leader_left.set_all_modes(trossen_arm.Mode.external_effort)
        leader_left.set_all_external_efforts(np.zeros(7), 0.0, False)
        leader_right.set_all_modes(trossen_arm.Mode.external_effort)
        leader_right.set_all_external_efforts(np.zeros(7), 0.0, False)
        print("✓ Both leaders are now freely movable\n")
        
        print("✓ Starting dual arm teleoperation...")
        print(f"  Control frequency: {self.control_frequency} Hz")
        print(f"  Press Ctrl+C to stop\n")
        
        start_time = time.time()
        loop_count = 0
        
        try:
            while True:
                loop_start = time.time()
                
                # Read leader poses and grippers
                left_pose = np.array(leader_left.get_cartesian_positions())
                right_pose = np.array(leader_right.get_cartesian_positions())
                left_gripper = leader_left.get_gripper_position()
                right_gripper = leader_right.get_gripper_position()
                
                # Send to followers
                self.send_cartesian_command(follower_left, left_pose, left_gripper, robot_id='follower_left')
                self.send_cartesian_command(follower_right, right_pose, right_gripper, robot_id='follower_right')
                
                # Print status periodically
                if loop_count % (self.control_frequency * 2) == 0:
                    elapsed = time.time() - start_time
                    actual_freq = loop_count / elapsed if elapsed > 0 else 0
                    print(f"[{elapsed:.1f}s] Left: [{left_pose[0]:.3f}, {left_pose[1]:.3f}, {left_pose[2]:.3f}] | "
                          f"Right: [{right_pose[0]:.3f}, {right_pose[1]:.3f}, {right_pose[2]:.3f}] | "
                          f"Freq: {actual_freq:.1f} Hz")
                
                # Check duration
                if duration and (time.time() - start_time) >= duration:
                    print(f"\n✓ Duration {duration}s reached")
                    break
                
                # Sleep to maintain control frequency
                loop_time = time.time() - loop_start
                sleep_time = max(0, self.control_dt - loop_time)
                if sleep_time > 0:
                    time.sleep(sleep_time)
                
                loop_count += 1
                
        except KeyboardInterrupt:
            print("\n\n✓ Teleoperation stopped by user")
        
        elapsed = time.time() - start_time
        avg_freq = loop_count / elapsed if elapsed > 0 else 0
        print(f"\nStatistics:")
        print(f"  Total time: {elapsed:.2f}s")
        print(f"  Total loops: {loop_count}")
        print(f"  Average frequency: {avg_freq:.1f} Hz")


def main():
    parser = argparse.ArgumentParser(
        description='Cartesian space teleoperation for Trossen arms',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  # Single arm teleop at 50Hz
  python cartesian_teleop.py --mode single --leader-ip 192.168.1.2 --follower-ip 192.168.1.3
  
  # Dual arm teleop at 30Hz for 60 seconds
  python cartesian_teleop.py --mode dual --frequency 30 --duration 60 \\
      --leader-left-ip 192.168.1.2 --leader-right-ip 192.168.1.3 \\
      --follower-left-ip 192.168.1.4 --follower-right-ip 192.168.1.5
  
  # Use joint-space interpolation (faster, may be less smooth in task space)
  python cartesian_teleop.py --mode single --interpolation joint \\
      --leader-ip 192.168.1.2 --follower-ip 192.168.1.3
        """
    )
    
    # Mode selection
    parser.add_argument('--mode', choices=['single', 'dual'], required=False,
                        help='Single or dual arm teleoperation', default='dual')
    
    # Single arm IPs
    parser.add_argument('--leader-ip', help='Leader robot IP (single arm mode)', default='192.168.1.2')
    parser.add_argument('--follower-ip', help='Follower robot IP (single arm mode)', default='192.168.1.3')
    
    # Dual arm IPs
    parser.add_argument('--leader-left-ip', help='Left leader robot IP (dual arm mode)', default='192.168.1.2')
    parser.add_argument('--leader-right-ip', help='Right leader robot IP (dual arm mode)',default='192.168.1.4')
    parser.add_argument('--follower-left-ip', help='Left follower robot IP (dual arm mode)', default='192.168.1.3')
    parser.add_argument('--follower-right-ip', help='Right follower robot IP (dual arm mode)', default='192.168.1.5')
    
    # Control parameters
    parser.add_argument('--model', default='wxai_v0',
                        choices=['wxai_v0', 'vxai_v0'],
                        help='Robot model (default: wxai_v0)')
    parser.add_argument('--frequency', type=float, default=50.0,
                        help='Control frequency in Hz (default: 50)')
    parser.add_argument('--interpolation', choices=['cartesian', 'joint'], default='cartesian',
                        help='Interpolation space (default: cartesian)')
    parser.add_argument('--no-velocity-feedforward', action='store_true',
                        help='Disable velocity feedforward (may be jerkier)')
    parser.add_argument('--velocity-filter', type=float, default=0.3,
                        help='Velocity filter alpha 0-1, higher=less filtering (default: 0.3)')
    parser.add_argument('--duration', type=float,
                        help='Duration in seconds (default: run forever)')
    
    args = parser.parse_args()
    
    # Validate IP arguments
    if args.mode == 'single':
        if not args.leader_ip or not args.follower_ip:
            parser.error("Single arm mode requires --leader-ip and --follower-ip")
    elif args.mode == 'dual':
        if not all([args.leader_left_ip, args.leader_right_ip, 
                   args.follower_left_ip, args.follower_right_ip]):
            parser.error("Dual arm mode requires all four IP addresses")
    
    # Create controller
    controller = CartesianTeleopController(
        model=args.model,
        control_frequency=args.frequency,
        interpolation_space=args.interpolation,
        use_velocity_feedforward=not args.no_velocity_feedforward,
        velocity_filter_alpha=args.velocity_filter
    )
    
    # Run teleoperation
    if args.mode == 'single':
        controller.run_single_arm_teleop(
            args.leader_ip,
            args.follower_ip,
            args.duration
        )
    else:
        controller.run_dual_arm_teleop(
            args.leader_left_ip,
            args.leader_right_ip,
            args.follower_left_ip,
            args.follower_right_ip,
            args.duration
        )


if __name__ == '__main__':
    main()
