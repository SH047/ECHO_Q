#!/usr/bin/env python3
"""
ECHO-Q Main Driver
==================
The top-level ROS node for the ECHO-Q quadruped.

Responsibilities
----------------
1. Initialise all hardware and software subsystems.
2. Run the 50 Hz control loop.
3. Implement the BehaviorState FSM:
     REST     – hold default stance, motors powered but relaxed.
     TROT     – joystick-driven diagonal trot.
     NAVIGATE – autonomous trot driven by move_base cmd_vel.
     HOP      – single ballistic jump (experimental).
4. Fuse IMU feedback for active body levelling.
5. Publish telemetry topics for the web dashboard and RViz.

Topics Published
----------------
  /echo_q/state          (std_msgs/String)   – JSON state snapshot @ 10 Hz
  /echo_q/joint_states   (sensor_msgs/JointState) – servo angles @ 50 Hz
  /echo_q/odom           (nav_msgs/Odometry) – dead-reckoned odometry @ 50 Hz
  /echo_q/contacts       (std_msgs/String)   – foot contact mask @ 50 Hz

Topics Subscribed
-----------------
  /joy                   (sensor_msgs/Joy)      – PS4 controller
  /echo_q/imu/pitch      (std_msgs/Float32)     – BNO055 pitch from Arduino
  /echo_q/imu/roll       (std_msgs/Float32)     – BNO055 roll  from Arduino
  /echo_q/imu/yaw        (std_msgs/Float32)     – BNO055 yaw   from Arduino
  /cmd_vel               (geometry_msgs/Twist)  – move_base velocity command
"""

import rospy
import numpy as np
import json
import time

from std_msgs.msg       import Float32, String
from sensor_msgs.msg    import JointState
from geometry_msgs.msg  import Twist, TransformStamped
from nav_msgs.msg       import Odometry
import tf2_ros

# ── ECHO-Q Library ────────────────────────────────────────────────────────────
from echo_q_control.Config      import Configuration
from echo_q_control.Kinematics  import inverse_kinematics
from echo_q_control.State       import State, BehaviorState
from echo_q_control.Gait        import GaitController
from echo_q_control.Command     import Command
from echo_q_utilities.Utilities import apply_body_rotation, euler_to_rotation_matrix

# ── Hardware & Input ──────────────────────────────────────────────────────────
from echo_q_hardware_interfacing.HardwareInterface import HardwareInterface
from echo_q_input_interfacing.InputInterface       import InputInterface

# ── Joint name order: [FR_hip, FR_thigh, FR_calf,  FL_hip, FL_thigh, FL_calf,
#                       BR_hip, BR_thigh, BR_calf,  BL_hip, BL_thigh, BL_calf]
JOINT_NAMES = [
    "FR_hip", "FR_thigh", "FR_calf",
    "FL_hip", "FL_thigh", "FL_calf",
    "BR_hip", "BR_thigh", "BR_calf",
    "BL_hip", "BL_thigh", "BL_calf",
]

CONTROL_RATE_HZ  = 50
TELEMETRY_PERIOD = 5    # publish full telemetry every N ticks


class EchoQ_Driver:
    """Main ECHO-Q control node."""

    def __init__(self):
        rospy.init_node("echo_q_driver")
        self._print_banner()

        # ── Config & State ────────────────────────────────────────────────
        self.config = Configuration()
        self.state  = State()
        self.rate   = rospy.Rate(CONTROL_RATE_HZ)

        # ── Hardware ──────────────────────────────────────────────────────
        rospy.loginfo("[ECHO-Q] Initialising PCA9685 servo driver …")
        self.hardware = HardwareInterface(self.config)

        # ── Input ─────────────────────────────────────────────────────────
        rospy.loginfo("[ECHO-Q] Starting joystick input interface …")
        self.input_interface = InputInterface(self.config)

        # ── Gait ──────────────────────────────────────────────────────────
        self.gait_controller = GaitController(self.config)

        # ── Odometry ──────────────────────────────────────────────────────
        self._odom_x   = 0.0
        self._odom_y   = 0.0
        self._odom_yaw = 0.0

        # ── Publishers ────────────────────────────────────────────────────
        self.pub_joint_states = rospy.Publisher(
            "/echo_q/joint_states", JointState, queue_size=10)
        self.pub_odom = rospy.Publisher(
            "/echo_q/odom", Odometry, queue_size=10)
        self.pub_state = rospy.Publisher(
            "/echo_q/state", String, queue_size=10)
        self.pub_contacts = rospy.Publisher(
            "/echo_q/contacts", String, queue_size=10)

        self._tf_broadcaster = tf2_ros.TransformBroadcaster()

        # ── Subscribers ───────────────────────────────────────────────────
        rospy.Subscriber("/echo_q/imu/pitch", Float32, self._cb_pitch)
        rospy.Subscriber("/echo_q/imu/roll",  Float32, self._cb_roll)
        rospy.Subscriber("/echo_q/imu/yaw",   Float32, self._cb_yaw)
        rospy.Subscriber("/cmd_vel",           Twist,   self._cb_cmd_vel)

        rospy.loginfo("[ECHO-Q] All systems nominal. Press L1 to arm.")

    # ── IMU Callbacks ─────────────────────────────────────────────────────────

    def _cb_pitch(self, msg): self.state.pitch = msg.data
    def _cb_roll(self,  msg): self.state.roll  = msg.data
    def _cb_yaw(self,   msg): self.state.yaw   = msg.data

    # ── Navigation Callback (move_base) ──────────────────────────────────────

    def _cb_cmd_vel(self, msg: Twist):
        """Bridge move_base velocity commands into ECHO-Q state."""
        self.state.cmd_vel_linear  = np.array([msg.linear.x, msg.linear.y])
        self.state.cmd_vel_angular = msg.angular.z

    # ── Main Loop ─────────────────────────────────────────────────────────────

    def run(self):
        tick        = 0
        last_time   = rospy.Time.now()

        while not rospy.is_shutdown():
            now  = rospy.Time.now()
            dt   = (now - last_time).to_sec()
            last_time = now
            tick += 1

            # ── A. Get operator command ───────────────────────────────────
            command = self.input_interface.get_command(self.state, CONTROL_RATE_HZ)

            # ── B. FSM transitions ────────────────────────────────────────
            if command.joystick_control_event:
                if self.state.behavior_state == BehaviorState.REST:
                    self.state.behavior_state = BehaviorState.TROT
                    self.gait_controller.reset()
                    rospy.loginfo("[ECHO-Q] ▶  ARMED  → TROT")
                else:
                    self.state.behavior_state = BehaviorState.REST
                    rospy.loginfo("[ECHO-Q] ■  DISARMED → REST")

            if command.navigate_event:
                if self.state.behavior_state != BehaviorState.REST:
                    self.state.behavior_state = BehaviorState.NAVIGATE
                    rospy.loginfo("[ECHO-Q] 🗺  NAVIGATE mode (move_base)")

            # ── C. Update state from command ──────────────────────────────
            self.state.velocity  = command.horizontal_velocity
            self.state.yaw_rate  = command.yaw_rate
            self.state.height    = command.height

            # ── D. Execute behavior ───────────────────────────────────────
            foot_positions = self.config.default_stance.copy()
            contacts       = np.ones(4, dtype=bool)

            if self.state.behavior_state == BehaviorState.REST:
                self.hardware.relax_all_motors()

            elif self.state.behavior_state in (BehaviorState.TROT, BehaviorState.NAVIGATE):
                # Select velocity source
                if self.state.behavior_state == BehaviorState.NAVIGATE:
                    vx = np.clip(self.state.cmd_vel_linear[0] * self.config.nav_x_scale,
                                 -self.config.max_x_velocity, self.config.max_x_velocity)
                    vy = np.clip(self.state.cmd_vel_linear[1] * self.config.nav_y_scale,
                                 -self.config.max_y_velocity, self.config.max_y_velocity)
                    vel = np.array([vx, vy])
                    self.state.yaw_rate = np.clip(
                        self.state.cmd_vel_angular * self.config.nav_yaw_scale,
                        -self.config.max_yaw_rate, self.config.max_yaw_rate)
                else:
                    vel = self.state.velocity

                # 1. Gait phase update → foot positions + contacts
                foot_positions, contacts = self.gait_controller.update(
                    current_time     = now.to_sec(),
                    command_velocity = vel,
                    default_stance   = self.config.default_stance,
                )

                # 2. Apply body height from command
                foot_positions[2, :] += self.state.height - self.config.default_z_ref

                # 3. Active body levelling via IMU feedback
                #    We counter the measured roll/pitch to keep body horizontal
                levelling_roll  = -self.state.roll  * 0.4   # gain < 1 to avoid overshoot
                levelling_pitch = -self.state.pitch * 0.4
                foot_positions = apply_body_rotation(
                    foot_positions, levelling_roll, levelling_pitch, 0.0)

                # 4. Inverse kinematics
                joint_angles = inverse_kinematics(foot_positions, self.config)
                self.state.joint_angles = joint_angles

                # 5. Send to servos
                self.hardware.set_actuator_positions(joint_angles)

                # 6. Dead-reckoned odometry integration
                self._integrate_odom(vel, self.state.yaw_rate, dt)

            self.state.foot_contacts  = contacts
            self.state.foot_locations = foot_positions
            self.state.ticks          = tick

            # ── E. Publish telemetry ──────────────────────────────────────
            self._publish_joint_states(now)
            self._publish_odom(now)
            if tick % TELEMETRY_PERIOD == 0:
                self._publish_state_json()

            self.rate.sleep()

    # ── Telemetry Publishers ─────────────────────────────────────────────────

    def _publish_joint_states(self, stamp):
        msg = JointState()
        msg.header.stamp = stamp
        msg.name         = JOINT_NAMES
        # Flatten (3,4) column-major: FR, FL, BR, BL
        angles = self.state.joint_angles
        msg.position = [
            angles[0,0], angles[1,0], angles[2,0],   # FR
            angles[0,1], angles[1,1], angles[2,1],   # FL
            angles[0,2], angles[1,2], angles[2,2],   # BR
            angles[0,3], angles[1,3], angles[2,3],   # BL
        ]
        self.pub_joint_states.publish(msg)

    def _publish_odom(self, stamp):
        odom = Odometry()
        odom.header.stamp    = stamp
        odom.header.frame_id = "odom"
        odom.child_frame_id  = "base_link"

        odom.pose.pose.position.x = self._odom_x
        odom.pose.pose.position.y = self._odom_y
        odom.pose.pose.position.z = abs(self.state.height)

        cy, sy = np.cos(self._odom_yaw * 0.5), np.sin(self._odom_yaw * 0.5)
        odom.pose.pose.orientation.w = cy
        odom.pose.pose.orientation.z = sy

        odom.twist.twist.linear.x  = self.state.velocity[0]
        odom.twist.twist.linear.y  = self.state.velocity[1]
        odom.twist.twist.angular.z = self.state.yaw_rate

        self.pub_odom.publish(odom)

        # TF: odom → base_link
        t = TransformStamped()
        t.header.stamp            = stamp
        t.header.frame_id         = "odom"
        t.child_frame_id          = "base_link"
        t.transform.translation.x = self._odom_x
        t.transform.translation.y = self._odom_y
        t.transform.translation.z = abs(self.state.height)
        t.transform.rotation.w    = cy
        t.transform.rotation.z    = sy
        self._tf_broadcaster.sendTransform(t)

    def _publish_state_json(self):
        payload = {
            "mode"    : self.state.behavior_state.name,
            "height"  : round(self.state.height, 4),
            "roll"    : round(float(np.degrees(self.state.roll)), 2),
            "pitch"   : round(float(np.degrees(self.state.pitch)), 2),
            "yaw"     : round(float(np.degrees(self.state.yaw)), 2),
            "vx"      : round(float(self.state.velocity[0]), 3),
            "vy"      : round(float(self.state.velocity[1]), 3),
            "yaw_rate": round(float(self.state.yaw_rate), 3),
            "odom_x"  : round(self._odom_x, 3),
            "odom_y"  : round(self._odom_y, 3),
            "contacts": self.state.foot_contacts.tolist(),
            "tick"    : self.state.ticks,
        }
        self.pub_state.publish(json.dumps(payload))

    def _integrate_odom(self, velocity: np.ndarray, yaw_rate: float, dt: float):
        """Simple Euler dead-reckoning integration."""
        self._odom_yaw += yaw_rate * dt
        self._odom_x   += velocity[0] * np.cos(self._odom_yaw) * dt
        self._odom_y   += velocity[0] * np.sin(self._odom_yaw) * dt

    @staticmethod
    def _print_banner():
        banner = r"""
╔══════════════════════════════════════════════════════════════════╗
║                                                                  ║
║    ███████╗ ██████╗██╗  ██╗ ██████╗        ██████╗              ║
║    ██╔════╝██╔════╝██║  ██║██╔═══██╗      ██╔═══██╗             ║
║    █████╗  ██║     ███████║██║   ██║█████╗██║   ██║             ║
║    ██╔══╝  ██║     ██╔══██║██║   ██║╚════╝██║▄▄ ██║             ║
║    ███████╗╚██████╗██║  ██║╚██████╔╝      ╚██████╔╝             ║
║    ╚══════╝ ╚═════╝╚═╝  ╚═╝ ╚═════╝        ╚══▀▀═╝              ║
║                                                                  ║
║   Enhanced Cognitive Hybrid Quadruped  ·  ROS Noetic            ║
║   12-DOF  ·  50 Hz  ·  IK + SLAM + move_base                   ║
║                                                                  ║
╚══════════════════════════════════════════════════════════════════╝
"""
        rospy.loginfo(banner)


if __name__ == "__main__":
    try:
        driver = EchoQ_Driver()
        driver.run()
    except rospy.ROSInterruptException:
        rospy.loginfo("[ECHO-Q] Shutdown requested.")
