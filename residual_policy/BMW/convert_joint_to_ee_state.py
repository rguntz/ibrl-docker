#!/usr/bin/env python3
"""
Convert joint states to end-effector poses in parquet files.

This script processes all parquet files in a directory, converts joint states
(state and joint_states_2) to end-effector poses, and adds them as new columns
while preserving all existing data.
"""

import os
import numpy as np
import pandas as pd
import pinocchio as pin
from pinocchio.robot_wrapper import RobotWrapper
from scipy.spatial.transform import Rotation as R
from pathlib import Path
from tqdm import tqdm


# ==================================================
# CONFIGURATION
# ==================================================
class Config:
    """Configuration for joint to EE conversion."""
    
    # URDF paths
    URDF_LEFT = "/home/qtf5422/Desktop/AIRE/ibrl/trossen_arm_description/urdf/generated/wxai/wxai_follower.urdf"
    URDF_RIGHT = "/home/qtf5422/Desktop/AIRE/ibrl/trossen_arm_description/urdf/generated/wxai/wxai_follower.urdf"
    
    # Package directories for URDF loading
    PACKAGE_DIRS = ["/home/qtf5422/Desktop/AIRE/ibrl"]
    
    # Input directory containing parquet files
    INPUT_DIR = "/home/qtf5422/Desktop/AIRE/residual-offpolicy-rl/arm=trossen_dual/task=entire_ecu_assembly/data/chunk-000"
    
    # Column names in parquet file
    LEFT_JOINT_COL = "joint_states_2"
    RIGHT_JOINT_COL = "state"
    
    # Output column names (new columns to add)
    LEFT_EE_COL = "state_end_effector_2"
    RIGHT_EE_COL = "state_end_effector"
    
    # Expected number of joints (7 arm + 1 gripper)
    NUM_JOINTS = 7


# ==================================================
# ROBOT INITIALIZATION
# ==================================================
class RobotFK:
    """Forward kinematics calculator for a robot arm."""
    
    def __init__(self, urdf_path, package_dirs, arm_name="arm"):
        """
        Initialize robot FK calculator.
        
        Args:
            urdf_path: Path to URDF file
            package_dirs: List of package directories
            arm_name: Name for logging (e.g., "left", "right")
        """
        self.arm_name = arm_name
        self.robot = RobotWrapper.BuildFromURDF(urdf_path, package_dirs=package_dirs)
        self.model = self.robot.model
        self.data = self.robot.data
        
        # Validate model
        assert self.model.nq == 8, f"{arm_name} model nq={self.model.nq}, expected 8"
        
        # Find end-effector frame
        self.ee_frame_name = self._find_ee_frame()
        self.ee_frame_id = self.model.getFrameId(self.ee_frame_name)
        
        print(f"[{self.arm_name}] EE frame: {self.ee_frame_name}")
    
    def _find_ee_frame(self):
        """Find the end-effector frame (excluding gripper)."""
        preferred = ["tool", "tcp", "wrist", "ee"]
        for frame in reversed(self.model.frames):
            name = frame.name.lower()
            if any(p in name for p in preferred):
                return frame.name
        # Fallback: last frame
        return self.model.frames[-1].name
    
    def _build_q(self, q_arm):
        """
        Build full configuration vector from 7 arm joints.
        
        Args:
            q_arm: Array of shape (7,) with [joint1...joint6, gripper]
        
        Returns:
            q: Array of shape (8,) with duplicated gripper
        """
        q = np.zeros(self.model.nq)
        q[:6] = q_arm[:6]  # First 6 arm joints
        q[6] = q_arm[6]    # Gripper 1
        q[7] = q_arm[6]    # Gripper 2 (duplicated)
        return q
    
    def compute_ee(self, q_arm):
        """
        Compute end-effector pose from joint configuration.
        
        Args:
            q_arm: Array of shape (7,) with joint angles
        
        Returns:
            ee_pose: Array of shape (7,) with [x, y, z, rotvec_x, rotvec_y, rotvec_z, gripper]
        """
        q = self._build_q(q_arm)
        
        # Forward kinematics
        pin.forwardKinematics(self.model, self.data, q)
        pin.updateFramePlacements(self.model, self.data)
        
        # Extract transform
        T = self.data.oMf[self.ee_frame_id]
        pos = T.translation # save T.rotation for mike. 
        rotvec = R.from_matrix(T.rotation).as_rotvec()
        
        # Build output
        ee_pose = np.zeros(7)
        ee_pose[0:3] = pos
        ee_pose[3:6] = rotvec
        ee_pose[6] = q_arm[6]  # Gripper state
        
        return ee_pose
    
    def compute_ee_batch(self, Q_arm):
        """
        Compute end-effector poses for batch of joint configurations.
        
        Args:
            Q_arm: Array of shape (T, 7) with joint angles
        
        Returns:
            ee_poses: Array of shape (T, 7) with EE poses
        """
        T_steps = Q_arm.shape[0]
        ee_poses = np.zeros((T_steps, 7))
        
        for t in range(T_steps):
            ee_poses[t] = self.compute_ee(Q_arm[t])
        
        return ee_poses


# ==================================================
# PARQUET PROCESSING
# ==================================================
def process_parquet_file(
    filepath,
    robot_left,
    robot_right,
    left_joint_col,
    right_joint_col,
    left_ee_col,
    right_ee_col
):
    """
    Process a single parquet file: load, convert joints to EE, save.
    
    Args:
        filepath: Path to parquet file
        robot_left: RobotFK instance for left arm
        robot_right: RobotFK instance for right arm
        left_joint_col: Column name for left joint states
        right_joint_col: Column name for right joint states
        left_ee_col: Column name for left EE output
        right_ee_col: Column name for right EE output
    
    Returns:
        bool: True if successful, False otherwise
    """
    try:
        # Load parquet file
        df = pd.read_parquet(filepath)
        
        # Check if columns exist
        if left_joint_col not in df.columns or right_joint_col not in df.columns:
            print(f"  ⚠️  Skipping {filepath.name}: missing joint columns")
            return False
        
        # Extract joint states
        Q_left = np.stack(df[left_joint_col].values)   # (T, 7)
        Q_right = np.stack(df[right_joint_col].values) # (T, 7)
        
        # Validate shape
        if Q_left.shape[1] != 7 or Q_right.shape[1] != 7:
            print(f"  ⚠️  Skipping {filepath.name}: unexpected joint shape")
            return False
        
        # Convert to end-effector poses
        ee_left = robot_left.compute_ee_batch(Q_left)
        ee_right = robot_right.compute_ee_batch(Q_right)
        
        # Add new columns to dataframe (as list of arrays)
        df[left_ee_col] = list(ee_left)
        df[right_ee_col] = list(ee_right)
        
        # Save back to parquet (overwrite)
        df.to_parquet(filepath, engine='pyarrow', compression='snappy')
        
        return True
        
    except Exception as e:
        print(f"  ❌ Error processing {filepath.name}: {e}")
        return False


def process_directory(input_dir, config):
    """
    Process all parquet files in a directory.
    
    Args:
        input_dir: Directory containing parquet files
        config: Config object with settings
    """
    input_path = Path(input_dir)
    
    if not input_path.exists():
        print(f"❌ Directory not found: {input_dir}")
        return
    
    # Find all parquet files
    parquet_files = sorted(input_path.glob("*.parquet"))
    
    if not parquet_files:
        print(f"❌ No parquet files found in: {input_dir}")
        return
    
    print(f"Found {len(parquet_files)} parquet files")
    print("=" * 60)
    
    # Initialize robots
    print("Initializing robot models...")
    robot_left = RobotFK(config.URDF_LEFT, config.PACKAGE_DIRS, arm_name="LEFT")
    robot_right = RobotFK(config.URDF_RIGHT, config.PACKAGE_DIRS, arm_name="RIGHT")
    print("=" * 60)
    
    # Process each file
    success_count = 0
    for filepath in tqdm(parquet_files, desc="Processing files"):
        success = process_parquet_file(
            filepath,
            robot_left,
            robot_right,
            config.LEFT_JOINT_COL,
            config.RIGHT_JOINT_COL,
            config.LEFT_EE_COL,
            config.RIGHT_EE_COL
        )
        if success:
            success_count += 1
    
    # Summary
    print("=" * 60)
    print(f"✅ Successfully processed: {success_count}/{len(parquet_files)} files")
    print("=" * 60)


# ==================================================
# MAIN
# ==================================================
def main():
    """Main entry point."""
    config = Config()
    
    print("=" * 60)
    print("JOINT TO END-EFFECTOR CONVERSION")
    print("=" * 60)
    print(f"Input directory: {config.INPUT_DIR}")
    print(f"Left joints:  {config.LEFT_JOINT_COL}  → {config.LEFT_EE_COL}")
    print(f"Right joints: {config.RIGHT_JOINT_COL} → {config.RIGHT_EE_COL}")
    print("=" * 60)
    
    process_directory(config.INPUT_DIR, config)


if __name__ == "__main__":
    main()