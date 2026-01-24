import numpy as np
from scipy.spatial.transform import Rotation as R
from typing import Tuple, List

class SmoothTrajectoryGenerator:
    def __init__(self, duration: float = 1.0, num_steps: int = 20):
        """
        Generate smooth bimanual EE trajectories using quintic polynomials.
        
        Args:
            duration: Total trajectory time T (seconds)
            num_steps: Number of interpolated steps (N)
        """
        self.T = duration
        self.N = num_steps
        self.timesteps = np.linspace(0, self.T, self.N)
    
    def _solve_quintic_coeffs(self, p0, v0, a0, pf, vf, af):
        """
        Solve for coefficients [a0, a1, ..., a5] of quintic polynomial:
            p(t) = a0 + a1*t + a2*t^2 + a3*t^3 + a4*t^4 + a5*t^5
        
        Boundary conditions at t=0 and t=T.
        Returns: (6,) coefficient vector
        """
        T = self.T
        # Construct linear system A x = b
        A = np.array([
            [1, 0, 0, 0, 0, 0],                      # p(0) = p0
            [0, 1, 0, 0, 0, 0],                      # p'(0) = v0
            [0, 0, 2, 0, 0, 0],                      # p''(0) = a0
            [1, T, T**2, T**3, T**4, T**5],         # p(T) = pf
            [0, 1, 2*T, 3*T**2, 4*T**3, 5*T**4],   # p'(T) = vf
            [0, 0, 2, 6*T, 12*T**2, 20*T**3]       # p''(T) = af
        ])
        b = np.array([p0, v0, a0, pf, vf, af])
        coeffs = np.linalg.solve(A, b)
        return coeffs
    
    def _evaluate_quintic(self, coeffs, t):
        """Evaluate polynomial at time t."""
        t_vec = np.array([1, t, t**2, t**3, t**4, t**5])
        pos = np.dot(coeffs, t_vec)
        return pos
    
    def _interpolate_orientation(self, quat_start, quat_end, ang_vel_start, t_vals):
        """
        Interpolate orientation using quintic polynomial in angle-axis space.
        
        Args:
            quat_start, quat_end: (4,) quaternions in [w, x, y, z] (MuJoCo convention)
            ang_vel_start: (3,) angular velocity at start in rad/s
            t_vals: array of shape (N,) with time values from 0 to T
        
        Returns:
            aa_array: (N, 3) angle-axis representations
        """
        # Convert quaternions to angle-axis representation
        # MuJoCo [w,x,y,z] -> SciPy [x,y,z,w]
        q_start_xyzw = np.array([quat_start[1], quat_start[2], quat_start[3], quat_start[0]])
        q_end_xyzw = np.array([quat_end[1], quat_end[2], quat_end[3], quat_end[0]])
        
        # Convert to angle-axis (rotation vector)
        aa_start = R.from_quat(q_start_xyzw).as_rotvec()  # (3,)
        aa_end = R.from_quat(q_end_xyzw).as_rotvec()      # (3,)
        
        # Use quintic interpolation for each axis of the rotation vector
        aa_array = []
        for dim in range(3):
            coeffs = self._solve_quintic_coeffs(
                p0=aa_start[dim],
                v0=ang_vel_start[dim],  # Initial angular velocity
                a0=0.0,                  # Zero initial angular acceleration
                pf=aa_end[dim],
                vf=0.0,                  # Zero final angular velocity
                af=0.0                   # Zero final angular acceleration
            )
            aa_dim_traj = [self._evaluate_quintic(coeffs, t) for t in t_vals]
            aa_array.append(aa_dim_traj)
        
        aa_array = np.array(aa_array).T  # (N, 3)
        return aa_array
    
    def generate_trajectory(
        self,
        current_state: Tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray],
        target_action_14d: np.ndarray, 
        velocities, 
    ) -> List[np.ndarray]:
        """
        Generate N-step smooth trajectory from current EE state to target action.
        
        Args:
            current_state: (left_pos, left_quat, right_pos, right_quat)
                - each pos: (3,), quat: (4,) in [w,x,y,z]
            target_action_14d: (14,) = [posL(3), aaL(3), gripL(1), posR(3), aaR(3), gripR(1)]
            velocities: (12,) = [velL(3), ang_velL(3), velR(3), ang_velR(3)]
        
        Returns:
            List of N actions, each of shape (14,)
        """
        left_pos_curr, left_quat_curr, right_pos_curr, right_quat_curr = current_state
        velocities_left = velocities[:6]
        velocities_right = velocities[6:]
        
        # Extract linear and angular velocities
        lin_vel_left = velocities_left[:3]
        ang_vel_left = velocities_left[3:6]
        lin_vel_right = velocities_right[:3]
        ang_vel_right = velocities_right[3:6]
        
        # Extract target positions and grippers
        target_left_pos = target_action_14d[0:3]
        target_right_pos = target_action_14d[7:10]
        grip_left = target_action_14d[6]
        grip_right = target_action_14d[13]
        
        # Convert target angle-axis to quaternions
        target_left_aa = target_action_14d[3:6]
        target_right_aa = target_action_14d[10:13]
        target_left_quat = R.from_rotvec(target_left_aa).as_quat()  # xyzw
        target_right_quat = R.from_rotvec(target_right_aa).as_quat()
        
        # Reorder to wxyz (MuJoCo convention)
        target_left_quat_wxyz = np.array([target_left_quat[3], target_left_quat[0], 
                                          target_left_quat[1], target_left_quat[2]])
        target_right_quat_wxyz = np.array([target_right_quat[3], target_right_quat[0], 
                                           target_right_quat[1], target_right_quat[2]])
        
        # === Position interpolation (quintic) ===
        pos_traj_left = []
        pos_traj_right = []
        for dim in range(3):
            coeffs_l = self._solve_quintic_coeffs(
                p0=left_pos_curr[dim], v0=lin_vel_left[dim], a0=0.0,
                pf=target_left_pos[dim], vf=0.0, af=0.0
            )
            coeffs_r = self._solve_quintic_coeffs(
                p0=right_pos_curr[dim], v0=lin_vel_right[dim], a0=0.0,
                pf=target_right_pos[dim], vf=0.0, af=0.0
            )
            pos_traj_left.append([self._evaluate_quintic(coeffs_l, t) for t in self.timesteps])
            pos_traj_right.append([self._evaluate_quintic(coeffs_r, t) for t in self.timesteps])
        
        pos_traj_left = np.array(pos_traj_left).T  # (N, 3)
        pos_traj_right = np.array(pos_traj_right).T  # (N, 3)
        
        # === Orientation interpolation (quintic in angle-axis space) ===
        aa_traj_left = self._interpolate_orientation(
            left_quat_curr, target_left_quat_wxyz, ang_vel_left, self.timesteps
        )
        aa_traj_right = self._interpolate_orientation(
            right_quat_curr, target_right_quat_wxyz, ang_vel_right, self.timesteps
        )
        
        # === Assemble full 14D actions ===
        trajectory = []
        for i in range(self.N):
            action = np.concatenate([
                pos_traj_left[i],      # 3
                aa_traj_left[i],       # 3
                [grip_left],           # 1
                pos_traj_right[i],     # 3
                aa_traj_right[i],      # 3
                [grip_right]           # 1
            ])  # total 14
            trajectory.append(action.astype(np.float64))
        
        return trajectory