import numpy as np

class Command:
    """Stores movement commands from the joystick or keyboard."""
    def __init__(self):
        # Velocity Commands (x, y)
        self.horizontal_velocity = np.array([0.0, 0.0])
        self.yaw_rate = 0.0

        # Pose Commands
        self.height = -0.16
        self.pitch = 0.0
        self.roll = 0.0

        # Activation Toggles (0 or 1)
        self.trot_event = 0
        self.hop_event = 0
        self.joystick_control_event = 0

        # Multipliers for input sensitivity
        self.height_movement = 0.0
        self.roll_movement = 0.0
