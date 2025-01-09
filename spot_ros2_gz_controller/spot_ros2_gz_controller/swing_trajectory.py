import numpy as np
from pydrake.trajectories import PiecewisePolynomial

class SwingTrajectory():
    def __init__(self, swing_height = 0.1):
        
        self.swing_height = swing_height

        self.current_positions = np.zeros((4,3))
        self.target_positions  = np.zeros((4,3))

    def generate_swingfoot_trajectory(self, total_swing_time, current_time):

        break_points = np.array([[0.], []])
        
        pass