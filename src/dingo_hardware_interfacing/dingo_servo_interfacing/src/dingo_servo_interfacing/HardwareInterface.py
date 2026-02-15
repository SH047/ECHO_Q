#!/usr/bin/env python3
from adafruit_servokit import ServoKit
import numpy as np
import math as m
import rospy
import sys

class HardwareInterface:
    def __init__(self, link):
        self.pwm_max = 2400
        self.pwm_min = 370
        self.link = link
        self.servo_angles = np.zeros((3, 4))

        # --- I2C Setup ---
        try:
            self.kit = ServoKit(channels=16, address=0x40)
        except (OSError, ValueError) as e:
            rospy.logfatal("Failed to initialize ServoKit. Check I2C.")
            sys.exit(1)

        # --- FINAL TUNED CALIBRATION MATRIX ---
        # Hips: Lifted to 95 for FL/BR.
        # Calves: Set to 130/120 to ensure maximum tuck.
        self.physical_calibration_offsets = np.array([
            [15, 95, 95, 3],     # Hips (FL & BR confirmed at 95)
            [60, 1, 60, 8],      # Thighs
            [110, -110, 150, -80] # Calves
        ])

        # Map the servo pin IDs
        self.pins = np.array([
            [14, 10, 2, 6],
            [15, 9, 1, 5],
            [12, 8, 0, 4]
        ])

        # --- INVERSION MASK (Final Logic Match) ---
        self.inversion_mask = np.array([
            [1, 0, 0, 1], # Hips: FR and BL are inverted
            [0, 1, 0, 1], # Thighs: FL and BL are inverted
            [0, 1, 0, 1]  # Calves: FL and BL are inverted
        ])

        self.create()

    def create(self):
        for i in range(16):
            try:
                self.kit.servo[i].actuation_range = 180
                self.kit.servo[i].set_pulse_width_range(self.pwm_min, self.pwm_max)
            except Exception:
                pass

    def set_actuator_positions(self, joint_angles):
        for leg_index in range(4):
            for axis_index in range(3):
                theta_degrees = m.degrees(joint_angles[axis_index, leg_index])
                
                if self.inversion_mask[axis_index, leg_index] == 1:
                    command_angle = 180 - theta_degrees
                else:
                    command_angle = theta_degrees

                final_angle = self.physical_calibration_offsets[axis_index, leg_index] + command_angle
                
                # Constrain to servo limits
                final_angle = max(0, min(180, final_angle))
                
                try:
                    self.kit.servo[self.pins[axis_index, leg_index]].angle = final_angle
                except Exception:
                    pass

    def relax_all_motors(self, servo_list=np.ones((3, 4))):
        for leg_index in range(4):
            for axis_index in range(3):
                if servo_list[axis_index, leg_index] == 1:
                    try:
                        self.kit.servo[self.pins[axis_index, leg_index]].angle = None
                    except Exception:
                        pass
