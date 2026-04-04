#!/usr/bin/env python3
import numpy as np

class Command:
    def __init__(self):
        self.horizontal_velocity = np.zeros(2)
        self.yaw_rate            = 0.0
        self.height              = -0.16
        self.pitch               = 0.0
        self.roll                = 0.0
        self.height_movement     = 0.0
        self.roll_movement       = 0.0
        self.trot_event             = 0
        self.hop_event              = 0
        self.joystick_control_event = 0
        self.navigate_event         = 0
