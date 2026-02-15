import numpy as np
from enum import Enum

class BehaviorState(Enum):
    REST = 0
    TROT = 1
    HOP = 2
    FINISH = 3

class State:
    def __init__(self):
        # Default height from Config (-0.16m or -0.15m depending on tuning)
        self.height = -0.16 

        # Orientation
        self.pitch = 0.0
        self.roll = 0.0
        self.yaw = 0.0

        # Velocity
        self.velocity = np.array([0.0, 0.0])
        self.yaw_rate = 0.0

        # Foot locations (Relative to body center)
        # Will be updated by the Controller
        self.foot_locations = np.zeros((3, 4))

        # Joint angles (Hip, Thigh, Calf)
        self.joint_angles = np.zeros((3, 4))

        # Behavior Mode
        self.behavior_state = BehaviorState.REST

        # Timing
        self.ticks = 0
