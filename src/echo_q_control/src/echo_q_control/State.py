#!/usr/bin/env python3
import numpy as np
from enum import Enum

class BehaviorState(Enum):
    REST     = 0
    TROT     = 1
    HOP      = 2
    NAVIGATE = 3
    FINISH   = 4

class State:
    def __init__(self):
        self.behavior_state  = BehaviorState.REST
        self.height          = -0.16
        self.pitch           = 0.0
        self.roll            = 0.0
        self.yaw             = 0.0
        self.velocity        = np.zeros(2)
        self.yaw_rate        = 0.0
        self.foot_locations  = np.zeros((3, 4))
        self.foot_contacts   = np.ones(4, dtype=bool)
        self.joint_angles    = np.zeros((3, 4))
        self.ticks           = 0
        self.cmd_vel_linear  = np.zeros(2)
        self.cmd_vel_angular = 0.0
