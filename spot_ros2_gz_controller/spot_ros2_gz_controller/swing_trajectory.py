class SwingTrajectory():
    def __init__(self):
        
        self.swing_height = 0.1
        self.ground_clearance = 0.02

        self.current_positions = np.zeros((4,3))
        self.target_positions = np.zeros((4,3))