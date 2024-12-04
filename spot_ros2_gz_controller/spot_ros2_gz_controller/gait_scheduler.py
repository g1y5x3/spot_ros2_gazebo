class GaitScheduler:
    def __init__(self, gait_cycle=0.5, start_time=0):
        self.gait_cycle = gait_cycle    # total time for one complete cycle
        self.duty_factor = 0.5  # portion of cycle spent in stance
        self.phase_offset = {
                  #  FL   FR   RL   RR
            'trot': [0.0, 0.5, 0.5, 0.0]     
        }

        self.start_time = start_time
        self.current_phase = 0.0    # current_phase /in [0,1)
        self.current_gait = 'trot'

    def update_phase(self, current_time):
        elapsed = (current_time - self.start_time).nanoseconds / 1e9
        self.current_phase = (elapsed % self.gait_cycle) / self.gait_cycle

    def update_leg_state(self, type='trot'):
        leg_phase = self.current_phase + self.phase_offset
        pass