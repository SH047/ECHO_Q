import numpy as np
from enum import Enum
import math as m

# NOTE: You will need the input package for this import to work later
# from echo_q_input_interfacing.HardwareConfig import PS4_COLOR, PS4_DEACTIVATED_COLOR

class Configuration:
    def __init__(self):
        self.ps4_color = {"r": 255, "g": 0, "b": 255}
        self.ps4_deactivated_color = {"r": 0, "g": 0, "b": 0}
        
        # --- COMMANDS ---
        self.max_x_velocity = 0.50 
        self.max_y_velocity = 0.25
        self.max_yaw_rate = 1.0
        self.max_pitch = 30.0 * np.pi / 180.0

        # --- MOVEMENT PARAMS ---
        self.z_time_constant = 0.02
        self.z_speed = 0.06
        self.pitch_deadband = 0.05
        self.pitch_time_constant = 0.25
        self.max_pitch_rate = 0.3
        self.roll_speed = 0.1
        self.yaw_time_constant = 0.3
        self.max_stance_yaw = 1.2
        self.max_stance_yaw_rate = 1.0
        
        # --- INPUT SETTINGS ---
        self.x_time_constant = 0.25 
        self.y_time_constant = 0.25
        self.yaw_time_constant = 0.30
        self.velocity_deadband = 0.15 
        self.yaw_deadband = 0.15

        # --- STANCE ---
        self.delta_x = 0.09      
        self.rear_leg_x_shift = -0.02
        self.front_leg_x_shift = 0.02
        self.delta_y = 0.085     
        self.default_z_ref = -0.16 

        # --- SWING ---
        self.z_coeffs = None
        self.__z_clearance = 0.06 
        self.alpha = 0.5
        self.beta = 0.5

        # --- GAIT ---
        self.dt = 0.01
        self.num_phases = 4
        self.contact_phases = np.array(
            [[1, 1, 1, 0],
             [1, 0, 1, 1],
             [1, 0, 1, 1],
             [1, 1, 1, 0]]
        )
        self.overlap_time = 0.04 
        self.swing_time = 0.07 

        # --- GEOMETRY ---
        self.LEG_FB = 0.170
        self.LEG_LR = 0.0975
        self.LEG_ORIGINS = np.array([
            [self.LEG_FB,  self.LEG_FB,  -self.LEG_FB, -self.LEG_FB],
            [-self.LEG_LR, self.LEG_LR,  -self.LEG_LR, self.LEG_LR],
            [0.0, 0.0, 0.0, 0.0],
        ])
        self.L1 = 0.05162024721 
        self.L2 = 0.11976
        self.L3 = 0.12518
        self.phi = m.radians(73.91738698)
        self.FRAME_MASS = 0.560
        self.MODULE_MASS = 0.080
        self.LEG_MASS = 0.030
        self.MASS = self.FRAME_MASS + (self.MODULE_MASS + self.LEG_MASS) * 4
        self.FRAME_INERTIA = tuple(map(lambda x: 3.0 * x, (1.844e-4, 1.254e-3, 1.337e-3)))
        self.MODULE_INERTIA = (3.698e-5, 7.127e-6, 4.075e-5)
        leg_z = 1e-6
        leg_mass = 0.010
        leg_x = (1.0 / 12.0) * self.L2 ** 2 * leg_mass
        leg_y = leg_x
        self.LEG_INERTIA = (leg_x, leg_y, leg_z)

    @property
    def default_stance(self):
        return np.array([
            [self.delta_x + self.front_leg_x_shift, self.delta_x + self.front_leg_x_shift, -self.delta_x + self.rear_leg_x_shift, -self.delta_x + self.rear_leg_x_shift],
            [-self.delta_y, self.delta_y, -self.delta_y, self.delta_y],
            [self.default_z_ref, self.default_z_ref, self.default_z_ref, self.default_z_ref]
        ])

    @property
    def z_clearance(self):
        return self.__z_clearance

    @z_clearance.setter
    def z_clearance(self, z):
        if z is None:
            self.__z_clearance = 0.05
        else:
            self.__z_clearance = max(0.0, min(0.12, float(z)))

    @property
    def overlap_ticks(self):
        return int(self.overlap_time / self.dt)

    @property
    def swing_ticks(self):
        return int(self.swing_time / self.dt)

    @property
    def stance_ticks(self):
        return 2 * self.overlap_ticks + self.swing_ticks

    @property
    def phase_ticks(self):
        return np.array([self.overlap_ticks, self.swing_ticks, self.overlap_ticks, self.swing_ticks])

    @property
    def phase_length(self):
        return 2 * self.overlap_ticks + 2 * self.swing_ticks
