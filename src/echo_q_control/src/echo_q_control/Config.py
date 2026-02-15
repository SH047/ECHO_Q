import numpy as np
from enum import Enum
import math as m

# NOTE: Input config import is commented out until the input package is fully set up
# from echo_q_input_interfacing.HardwareConfig import PS4_COLOR, PS4_DEACTIVATED_COLOR

class Configuration:
    def __init__(self):
        self.ps4_color = {"r": 255, "g": 0, "b": 255}
        self.ps4_deactivated_color = {"r": 0, "g": 0, "b": 0}
        
        #################### COMMANDS ####################
        self.max_x_velocity = 0.80  # INCREASED SPEED (Was 0.50)
        self.max_y_velocity = 0.45
        self.max_yaw_rate = 1.25
        self.max_pitch = 30.0 * np.pi / 180.0

        #################### MOVEMENT PARAMS ####################
        self.z_time_constant = 0.02
        self.z_speed = 0.06
        self.pitch_deadband = 0.05
        self.pitch_time_constant = 0.25
        self.max_pitch_rate = 0.3
        self.roll_speed = 0.1
        self.yaw_time_constant = 0.3
        self.max_stance_yaw = 1.2
        self.max_stance_yaw_rate = 1.0
        
        # FINAL INPUT FIX: Stable Deadband and Acceleration
        self.x_time_constant = 0.25 
        self.y_time_constant = 0.25
        self.yaw_time_constant = 0.30
        self.velocity_deadband = 0.25 
        self.yaw_deadband = 0.25

        #################### STANCE ####################
        self.delta_x = 0.09      
        self.rear_leg_x_shift = -0.04
        self.front_leg_x_shift = 0.00
        self.delta_y = 0.085     
        self.default_z_ref = -0.15 # Safe Squat Height

        #################### SWING ######################
        self.z_coeffs = None
        self.__z_clearance = 0.06 # High Step Clearance
        self.alpha = 0.5
        self.beta = 0.5

        #################### GAIT ####################
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

        ######################## GEOMETRY ####################
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

class SimulationConfig:
    def __init__(self):
        self.XML_IN = "pupper.xml"
        self.XML_OUT = "pupper_out.xml"
        self.START_HEIGHT = 0.3
        self.MU = 1.5
        self.DT = 0.001
        self.JOINT_SOLREF = "0.001 1"
        self.JOINT_SOLIMP = "0.9 0.95 0.001"
        self.GEOM_SOLREF = "0.01 1"
        self.GEOM_SOLIMP = "0.9 0.95 0.001"
        G = 220
        m_rotor = 0.016
        r_rotor = 0.005
        self.ARMATURE = G ** 2 * m_rotor * r_rotor ** 2
        self.REV_DAMPING = 1.049 
        self.SERVO_REV_KP = 1000 # Stable high stiffness
        self.MAX_JOINT_TORQUE = 3.0
        self.REVOLUTE_RANGE = 1.57

class Leg_linkage:
    def __init__(self, configuration: Configuration):
        # * YOUR MEASUREMENTS (IN MM) *
        self.a = 27.0   # Servo Horn
        self.b = 100.0  # Connecting Rod
        self.c = 26.0   # Lower Leg Lever
        self.d = 100.0  # Upper Leg Distance
        
        # Default Params (Likely unused but kept for compatibility)
        self.e = 67.1
        self.f = 130.0
        self.g = 37
        self.h = 43
        
        self.upper_leg_length = configuration.L2 * 1000
        self.lower_leg_length = configuration.L3 * 1000
        self.lower_leg_bend_angle = m.radians(0)
        self.i = self.upper_leg_length
        self.hip_width = configuration.L1 * 1000
        self.gamma = m.atan(28.80 / 20.20)
        acos_arg = (self.c * 2 + self.h * 2 - self.e ** 2) / (2 * self.c * self.h)
        acos_arg = max(-1.0, min(1.0, acos_arg))
        self.EDC = m.acos(acos_arg)
