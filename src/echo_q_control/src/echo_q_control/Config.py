#!/usr/bin/env python3
"""ECHO-Q Configuration — single source of truth for all parameters."""
import numpy as np, math as m

class Configuration:
    def __init__(self):
        self.max_x_velocity  = 0.80
        self.max_y_velocity  = 0.45
        self.max_yaw_rate    = 1.25
        self.velocity_deadband   = 0.25
        self.yaw_deadband        = 0.25
        self.x_time_constant     = 0.25
        self.y_time_constant     = 0.25
        self.yaw_time_constant   = 0.30
        self.z_speed           = 0.06
        self.max_pitch         = m.radians(30.0)
        self.max_pitch_rate    = 0.30
        self.pitch_deadband    = 0.05
        self.pitch_time_constant = 0.25
        self.roll_speed        = 0.10
        self.delta_x              = 0.09
        self.rear_leg_x_shift     = -0.04
        self.front_leg_x_shift    = 0.00
        self.delta_y              = 0.085
        self.default_z_ref        = -0.15
        self.z_clearance  = 0.06
        self.dt           = 0.01
        self.num_phases   = 4
        self.contact_phases = np.array([[1,1,1,0],[1,0,1,1],[1,0,1,1],[1,1,1,0]])
        self.overlap_time   = 0.04
        self.swing_time     = 0.07
        self.LEG_FB = 0.170
        self.LEG_LR = 0.0975
        self.LEG_ORIGINS = np.array([
            [ self.LEG_FB,  self.LEG_FB, -self.LEG_FB, -self.LEG_FB],
            [-self.LEG_LR,  self.LEG_LR, -self.LEG_LR,  self.LEG_LR],
            [0.0, 0.0, 0.0, 0.0]])
        self.L1  = 0.05162
        self.L2  = 0.11976
        self.L3  = 0.12518
        self.nav_x_scale    = 1.0
        self.nav_y_scale    = 0.5
        self.nav_yaw_scale  = 1.0

    @property
    def default_stance(self):
        return np.array([
            [self.delta_x + self.front_leg_x_shift,
             self.delta_x + self.front_leg_x_shift,
            -self.delta_x + self.rear_leg_x_shift,
            -self.delta_x + self.rear_leg_x_shift],
            [-self.delta_y, self.delta_y, -self.delta_y, self.delta_y],
            [self.default_z_ref]*4])

    @property
    def overlap_ticks(self): return int(self.overlap_time / self.dt)
    @property
    def swing_ticks(self):   return int(self.swing_time   / self.dt)
    @property
    def stance_ticks(self):  return 2 * self.overlap_ticks + self.swing_ticks
    @property
    def phase_ticks(self):
        return np.array([self.overlap_ticks, self.swing_ticks,
                         self.overlap_ticks, self.swing_ticks])
    @property
    def phase_length(self):  return 2 * self.overlap_ticks + 2 * self.swing_ticks


class Leg_linkage:
    def __init__(self, configuration):
        self.a = 27.0; self.b = 100.0; self.c = 26.0; self.d = 100.0
        self.h = 43.0; self.e = 67.1
        self.upper_leg_length = configuration.L2 * 1000.0
        self.lower_leg_length = configuration.L3 * 1000.0
        self.hip_width = configuration.L1 * 1000.0
