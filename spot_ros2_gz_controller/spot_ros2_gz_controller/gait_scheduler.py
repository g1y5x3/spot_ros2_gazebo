import numpy as np
import matplotlib.pyplot as plt

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

    # Computes the ground contact schedule for each leg over the prediction 
    # horizon
    def get_contact_schedule(self, horizon_steps):
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
