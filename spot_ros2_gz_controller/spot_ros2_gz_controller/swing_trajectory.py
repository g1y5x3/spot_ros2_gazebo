import numpy as np
from pydrake.trajectories import PiecewisePolynomial

from .robot_state import RobotState
from .gait_scheduler import GaitScheduler

class SwingTrajectory():
    def __init__(self, swing_height = 0.1):
        
        self.swing_height = swing_height

        self.current_positions = np.zeros((4,3))
        self.target_positions  = np.zeros((4,3))

        self.K_p = np.zeros((3,3))
        self.K_d = np.zeros((3,3))
        self.tau_ff = 0
        self.p_des = np.zeros((4,3))

    def update_swingfoot_trajectory(self, robot_state: RobotState, gait_schedule: GaitScheduler):
        # obtain current foot position and velocity, hip position, and CoM velocity
        foot_position = robot_state.foot_pos[:,0]
        foot_velocity = robot_state.foot_vel[:,0]
        hip_position = robot_state.hip_pos[:,0]
        com_vel = robot_state.p_dot
        
        # print(f"foot position {foot_position} {np.size(foot_position)}")
        # print(f"foot velocity {foot_velocity} {np.size(foot_velocity)}")
        # print(f"hip position {hip_position} {np.size(hip_position)}")
        # print(f"com velocity {com_vel} {np.size(com_vel)}")

    def foot_planner(self, t_stance, H_w_base, p, pdot, pdot_d, foot_position):
        pass

