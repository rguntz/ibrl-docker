    def _get_observation(self):

        """Create a dummy observation dict with the current action as state."""
        observation = OrderedDict()

        # assuming you know the number of joints
        follower_left_qpos = np.array([self.follower_left.get_joint_position(i) for i in range(self.num_joints)])

        follower_right_qpos = np.array([self.follower_right.get_joint_position(i) for i in range(self.num_joints)])

        # Get joint velocities
        follower_left_qvel = np.array([self.follower_left.get_joint_velocity(i) for i in range(self.num_joints)])
        follower_right_qvel = np.array([self.follower_right.get_joint_velocity(i) for i in range(self.num_joints)])

        # Joint states (16D)
        observation['qpos'] = np.array([follower_left_qpos, follower_right_qpos])
        observation['qvel'] = np.array([follower_left_qvel, follower_right_qvel])

        follower_left_pose = np.array(self.follower_left.get_cartesian_positions())
        follower_right_pose = np.array(self.follower_right.get_cartesian_positions())

        follower_left_gripper = np.array(self.follower_left.get_gripper_position())
        follower_right_gripper = np.array(self.follower_right.get_gripper_position())

        # End-effector position (6D: 3D per arm)
        observation['robot0_eef_pos'] = np.array([follower_left_pose[:3], follower_right_pose[:3]])
        
        # End-effector orientation (8D: 4D quat per arm, using identity as dummy)
        observation['robot0_eef_quat'] = np.array([self.angle_axis_to_quaternion(follower_left_pose[3:]), 
                                                   self.angle_axis_to_quaternion(follower_right_pose[3:])])
        
        # Gripper positions (2D)
        observation['robot0_gripper_qpos'] = np.array([follower_left_gripper, follower_right_gripper])
        
        # Camera images (get latest from background thread - non-blocking)
        observation['images'] = self.get_latest_images()

        return observation
