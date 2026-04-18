#!/usr/bin/env python3
"""PCA9685 servo driver for 12x DS3225 servos."""
import sys, math as m
import numpy as np

try:
    from adafruit_servokit import ServoKit
    _HW = True
except ImportError:
    _HW = False

try:
    import rospy
    _log = rospy.logwarn
except ImportError:
    _log = print

# Column order matches JOINT_NAMES in echo_q_driver.py: col0=FR, col1=FL, col2=BR, col3=BL
# Row order: row0=Hip, row1=Thigh, row2=Calf
#
#                                        FR   FL   BR   BL
SERVO_CHANNELS = np.array([[0,   4,   8,  12],   # Hip    channels
                           [1,   5,   9,  13],   # Thigh  channels
                           [2,   6,  10,  14]], dtype=int)  # Calf channels

# 1 = invert (180 - angle), 0 = normal
# Hips:   FR(col0) and BL(col3) face opposite directions → invert
# Thighs: FL(col1) and BL(col3) are mirror-mounted      → invert
# Calves: FL(col1) and BL(col3) are mirror-mounted      → invert
#                                        FR   FL   BR   BL
INVERSION_MASK = np.array([[1,   0,   0,   1],   # Hip
                           [0,   1,   0,   1],   # Thigh
                           [0,   1,   0,   1]], dtype=int)  # Calf

# Tune these degree offsets until the robot stands level at default_z_ref.
#                                          FR    FL    BR    BL
CALIBRATION_OFFSETS = np.array([[ 15,   95,   95,    3],   # Hip
                                [ 60,    1,   60,    8],   # Thigh
                                [110, -110,  150,  -80]], dtype=float)  # Calf

class HardwareInterface:
    def __init__(self, config):
        self.servo_angles = np.zeros((3, 4))
        if _HW:
            try:
                self.kit = ServoKit(channels=16, address=0x40)
                for ch in range(16):
                    self.kit.servo[ch].actuation_range = 180
                    self.kit.servo[ch].set_pulse_width_range(370, 2400)
            except Exception as e:
                _log(f"PCA9685 init failed: {e}"); sys.exit(1)
        else:
            self.kit = None
            _log("adafruit_servokit not found – SIMULATION mode.")

    def set_actuator_positions(self, joint_angles):
        for leg in range(4):
            for axis in range(3):
                deg = m.degrees(joint_angles[axis, leg])
                cmd = (180.0 - deg) if INVERSION_MASK[axis, leg] else deg
                cmd = float(np.clip(cmd + CALIBRATION_OFFSETS[axis, leg], 0, 180))
                self.servo_angles[axis, leg] = cmd
                if self.kit:
                    try: self.kit.servo[SERVO_CHANNELS[axis, leg]].angle = cmd
                    except: pass

    def relax_all_motors(self, mask=None):
        if mask is None: mask = np.ones((3,4), dtype=bool)
        if not self.kit: return
        for leg in range(4):
            for axis in range(3):
                if mask[axis, leg]:
                    try: self.kit.servo[SERVO_CHANNELS[axis, leg]].angle = None
                    except: pass
