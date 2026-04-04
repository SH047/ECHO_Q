#!/usr/bin/env python3
"""PS4/Xbox joystick → ECHO-Q Command translator."""
import numpy as np
import rospy
from sensor_msgs.msg import Joy
from echo_q_control.State   import BehaviorState, State
from echo_q_control.Command import Command
from echo_q_utilities.Utilities import deadband, clipped_first_order_filter

class InputInterface:
    def __init__(self, config):
        self.config = config
        self._prev  = [0, 0, 0, 0]
        self._events= [0, 0, 0, 0]  # [trot, hop, arm, navigate]
        self._latest = Command()
        rospy.Subscriber("/joy", Joy, self._cb, queue_size=1)

    def _cb(self, msg):
        cmd = Command()
        btns = [msg.buttons[5], msg.buttons[0], msg.buttons[4], msg.buttons[7]]
        for i, b in enumerate(btns):
            if not self._events[i]:
                self._events[i] = int(b == 1 and self._prev[i] == 0)
            self._prev[i] = b
        cmd.horizontal_velocity = np.array([
            msg.axes[1] * self.config.max_x_velocity,
            msg.axes[0] * self.config.max_y_velocity])
        cmd.yaw_rate       = msg.axes[3] * self.config.max_yaw_rate
        cmd.pitch          = msg.axes[4] * self.config.max_pitch
        cmd.height_movement= msg.axes[7]
        cmd.roll_movement  = -msg.axes[6]
        self._latest = cmd

    def get_command(self, state, message_rate):
        cmd = self._latest
        cmd.trot_event, cmd.hop_event, cmd.joystick_control_event, cmd.navigate_event = self._events
        self._events = [0, 0, 0, 0]
        dt = 1.0 / message_rate
        vx = deadband(cmd.horizontal_velocity[0], self.config.velocity_deadband)
        vy = deadband(cmd.horizontal_velocity[1], self.config.velocity_deadband)
        yr = deadband(cmd.yaw_rate, self.config.yaw_deadband)
        yr = clipped_first_order_filter(state.yaw_rate, yr, self.config.max_yaw_rate, self.config.yaw_time_constant)
        cmd.horizontal_velocity = np.array([vx, vy])
        cmd.yaw_rate = yr
        cmd.height = float(np.clip(state.height - dt * self.config.z_speed * cmd.height_movement, -0.27, -0.08))
        cmd.roll   = float(np.clip(state.roll   + dt * self.config.roll_speed * cmd.roll_movement, -0.3, 0.3))
        return cmd
