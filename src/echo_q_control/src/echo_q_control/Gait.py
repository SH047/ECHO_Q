import numpy as np

class GaitController:
    def __init__(self, config):
        self.config = config
        self.phase_length = config.overlap_ticks + config.swing_ticks
        self.last_phase_change_time = 0
        self.time_in_phase = 0
        self.current_phase_index = 0
        
        # The contact phases are defined in Config.py (e.g., [[1,1,1,0], ...])
        # 1 = Stance (Foot on ground)
        # 0 = Swing (Foot in air)
        self.contacts = config.contact_phases[0]

    def update(self, current_time):
        """
        Updates the contact state of the robot based on the current time.
        """
        # Calculate how much time has passed since the last update
        self.time_in_phase += (current_time - self.last_phase_change_time)
        self.last_phase_change_time = current_time
        
        # Calculate the duration of one "tick" in seconds
        tick_duration = self.config.dt
        
        # Convert elapsed time to "ticks"
        ticks_passed = int(self.time_in_phase / tick_duration)
        
        # Check if it is time to switch to the next row in the gait matrix
        phase_duration_ticks = self.config.phase_ticks[self.current_phase_index]
        
        if ticks_passed > phase_duration_ticks:
            # Reset the counter
            self.time_in_phase = 0
            
            # Move to the next phase index (loop back to 0 if at the end)
            self.current_phase_index = (self.current_phase_index + 1) % self.config.num_phases
            
            # Update the contact array (this tells the Controller which feet are down)
            self.contacts = self.config.contact_phases[self.current_phase_index]

    def reset(self):
        """Resets the gait cycle to the beginning."""
        self.current_phase_index = 0
        self.time_in_phase = 0
        self.contacts = self.config.contact_phases[0]
