import numpy as np
import matplotlib.pyplot as plt

class GaitScheduler:
    def __init__(self, gait_cycle=0.5, start_time=0, timesteps=16):
        self.gait_cycle = gait_cycle    # in sec
        self.duty_factor = 0.5  # portion of cycle spent in stance
        self.t_start = start_time
        self.timesteps  = timesteps

        self.current_gait = 'trot'
        self.phase_offset = {
                  #  FL   FR   RL   RR
            'trot': [0.0, 0.5, 0.5, 0.0]     
        }

        self.current_phase = 0.0    # current_phase /in [0,1)
        self.phase_map = [0.0, 0.0, 0.0, 0.0]   # FL, FR, RL, RR

    def update_phase(self, t):
        dt = (t - self.t_start).nanoseconds / 1e9
        self.current_phase = (dt % self.gait_cycle) / self.gait_cycle

        for leg in range(4):
            stance_start = self.phase_offset[self.current_gait][leg]
            self.phase_map[leg] = (self.current_phase - stance_start) % 1.0
        print(f"phase map {self.phase_map}")

    def get_leg_state(self, leg_idx):
        return "stance" if self.phase_map[leg_idx] < self.duty_factor else "swing"

    def get_contact_schedule(self, horizon_steps):
        """
        Generate contact schedule for the full MPC horizon.
        The schedule always looks ahead for the full horizon,
        regardless of current phase.
            
        Args:
            horizon_steps: Number of timesteps to plan ahead
        """
        contact_schedule = np.zeros((4, horizon_steps))
    
        for timestep in range(horizon_steps):
            future_phase = (self.current_phase + timestep/horizon_steps) % 1.0

            for leg in range(4):
                stance_start = self.phase_offset[self.current_gait][leg]
                leg_phase = (future_phase - stance_start) % 1.0
                contact_schedule[leg, timestep] = 1.0 if leg_phase < self.duty_factor else 0.0
    
        return contact_schedule


if __name__ == "__main__":

    total_steps = 20
    scheduler = GaitScheduler(gait_cycle=0.5)
        
    plt.figure(figsize=(12, 6))
    contact_pattern = scheduler.get_contact_schedule(total_steps)
    leg_names = ['FL', 'FR', 'RL', 'RR']
    colors = ['lightgreen', 'darkgreen', 'forestgreen', 'seagreen']

    for leg in range(4):
        time_points = np.linspace(0, 1, total_steps)
        
        # Plot the contact pattern
        plt.plot(time_points, contact_pattern[leg] * (4-leg) + leg, 
                drawstyle='steps-post', color=colors[leg], linewidth=2,
                label=leg_names[leg])
        
        for step in range(total_steps-1):
            if contact_pattern[leg, step] == 1:
                plt.fill_between(time_points[step:step+2], 
                               [leg, leg], 
                               [leg+1, leg+1], 
                               color=colors[leg], alpha=0.3)
        
    plt.title(f'{scheduler.current_gait.capitalize()} Gait Pattern')
    plt.xlabel('Gait Cycles')
    plt.ylabel('Leg')
    plt.yticks(range(4), leg_names)
    plt.grid(True, alpha=0.3)
    plt.xlim(0, 1)
    plt.ylim(-0.2, 4.2)
    plt.tight_layout()
    plt.show()
