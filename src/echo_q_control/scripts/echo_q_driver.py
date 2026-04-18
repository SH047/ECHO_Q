#!/usr/bin/env python3
"""ECHO-Q Main Control Node — 50 Hz FSM + IK + Gait + Odometry."""
import rospy, numpy as np, json
from std_msgs.msg      import Float32, String
from sensor_msgs.msg   import JointState
from geometry_msgs.msg import Twist, TransformStamped
from nav_msgs.msg      import Odometry
import tf2_ros

from echo_q_control.Config      import Configuration
from echo_q_control.Kinematics  import inverse_kinematics
from echo_q_control.State       import State, BehaviorState
from echo_q_control.Gait        import GaitController
from echo_q_control.Command     import Command
from echo_q_utilities.Utilities import apply_body_rotation
from echo_q_hardware_interfacing.HardwareInterface import HardwareInterface
from echo_q_input_interfacing.InputInterface       import InputInterface

JOINT_NAMES = [f"{leg}_{joint}"
               for leg in ("FR","FL","BR","BL")
               for joint in ("hip","thigh","calf")]

class EchoQ_Driver:
    def __init__(self):
        rospy.init_node("echo_q_driver")
        rospy.loginfo("\n[ECHO-Q] ╔══════════════════════════════════╗\n"
                      "[ECHO-Q] ║  ECHO-Q  v1.0  |  ROS Noetic    ║\n"
                      "[ECHO-Q] ║  12-DOF · 50Hz · IK+SLAM+Nav    ║\n"
                      "[ECHO-Q] ╚══════════════════════════════════╝")
        self.config = Configuration()
        self.state  = State()
        self.rate   = rospy.Rate(50)
        self.hardware = HardwareInterface(self.config)
        self.input_interface = InputInterface(self.config)
        self.gait_controller = GaitController(self.config)
        self._ox = self._oy = self._oyaw = 0.0
        self.pub_joints   = rospy.Publisher("/echo_q/joint_states", JointState, queue_size=10)
        self.pub_odom     = rospy.Publisher("/echo_q/odom",         Odometry,   queue_size=10)
        self.pub_state    = rospy.Publisher("/echo_q/state",        String,     queue_size=10)
        self.pub_contacts = rospy.Publisher("/echo_q/contacts",    String,     queue_size=10)
        self._tfb         = tf2_ros.TransformBroadcaster()
        rospy.Subscriber("/echo_q/imu/pitch", Float32, lambda m: setattr(self.state, 'pitch', m.data))
        rospy.Subscriber("/echo_q/imu/roll",  Float32, lambda m: setattr(self.state, 'roll',  m.data))
        rospy.Subscriber("/echo_q/imu/yaw",   Float32, lambda m: setattr(self.state, 'yaw',   m.data))
        rospy.Subscriber("/cmd_vel", Twist, self._cb_cmdvel)
        rospy.loginfo("[ECHO-Q] Ready. Press L1 to arm.")

    def _cb_cmdvel(self, msg):
        self.state.cmd_vel_linear  = np.array([msg.linear.x, msg.linear.y])
        self.state.cmd_vel_angular = msg.angular.z

    def run(self):
        tick = 0
        last = rospy.Time.now()
        while not rospy.is_shutdown():
            now = rospy.Time.now()
            dt  = (now - last).to_sec()
            last = now; tick += 1
            cmd = self.input_interface.get_command(self.state, 50)
            if cmd.joystick_control_event:
                if self.state.behavior_state == BehaviorState.REST:
                    self.state.behavior_state = BehaviorState.TROT
                    self.gait_controller.reset()
                    rospy.loginfo("[ECHO-Q] ▶  TROT")
                else:
                    self.state.behavior_state = BehaviorState.REST
                    rospy.loginfo("[ECHO-Q] ■  REST")
            if cmd.navigate_event and self.state.behavior_state != BehaviorState.REST:
                self.state.behavior_state = BehaviorState.NAVIGATE
                rospy.loginfo("[ECHO-Q] 🗺  NAVIGATE")
            self.state.velocity = cmd.horizontal_velocity
            self.state.yaw_rate = cmd.yaw_rate
            self.state.height   = cmd.height
            contacts = np.ones(4, dtype=bool)
            if self.state.behavior_state == BehaviorState.REST:
                self.hardware.relax_all_motors()
            elif self.state.behavior_state in (BehaviorState.TROT, BehaviorState.NAVIGATE):
                vel = (np.array([self.state.cmd_vel_linear[0], self.state.cmd_vel_linear[1]])
                       if self.state.behavior_state == BehaviorState.NAVIGATE
                       else self.state.velocity)
                fp, contacts = self.gait_controller.update(now.to_sec(), vel, self.config.default_stance)
                fp[2, :] += self.state.height - self.config.default_z_ref
                fp = apply_body_rotation(fp, -self.state.roll*0.4, -self.state.pitch*0.4, 0.0)
                ja = inverse_kinematics(fp, self.config)
                self.state.joint_angles = ja
                self.hardware.set_actuator_positions(ja)
                self._ox  += vel[0] * np.cos(self._oyaw) * dt
                self._oy  += vel[0] * np.sin(self._oyaw) * dt
                self._oyaw+= self.state.yaw_rate * dt
            self.state.foot_contacts = contacts; self.state.ticks = tick
            self.pub_contacts.publish(json.dumps(self.state.foot_contacts.tolist()))
            self._pub_joints(now); self._pub_odom(now)
            if tick % 5 == 0: self._pub_state()
            self.rate.sleep()

    def _pub_joints(self, stamp):
        msg = JointState(); msg.header.stamp = stamp; msg.name = JOINT_NAMES
        ja  = self.state.joint_angles
        msg.position = [ja[r,c] for c in range(4) for r in range(3)]
        self.pub_joints.publish(msg)

    def _pub_odom(self, stamp):
        odom = Odometry(); odom.header.stamp = stamp
        odom.header.frame_id = "odom"; odom.child_frame_id = "base_link"
        odom.pose.pose.position.x = self._ox; odom.pose.pose.position.y = self._oy
        cy, sy = np.cos(self._oyaw/2), np.sin(self._oyaw/2)
        odom.pose.pose.orientation.w = cy; odom.pose.pose.orientation.z = sy
        odom.twist.twist.linear.x = self.state.velocity[0]
        odom.twist.twist.angular.z = self.state.yaw_rate
        self.pub_odom.publish(odom)
        t = TransformStamped(); t.header = odom.header; t.child_frame_id = "base_link"
        t.transform.translation.x = self._ox; t.transform.translation.y = self._oy
        t.transform.rotation.w = cy; t.transform.rotation.z = sy
        self._tfb.sendTransform(t)

    def _pub_state(self):
        self.pub_state.publish(json.dumps({
            "mode": self.state.behavior_state.name,
            "height": round(self.state.height, 4),
            "roll": round(float(np.degrees(self.state.roll)), 2),
            "pitch": round(float(np.degrees(self.state.pitch)), 2),
            "yaw": round(float(np.degrees(self.state.yaw)), 2),
            "vx": round(float(self.state.velocity[0]), 3),
            "vy": round(float(self.state.velocity[1]), 3),
            "yaw_rate": round(float(self.state.yaw_rate), 3),
            "odom_x": round(self._ox, 3), "odom_y": round(self._oy, 3),
            "contacts": self.state.foot_contacts.tolist(), "tick": self.state.ticks}))

if __name__ == "__main__":
    try: EchoQ_Driver().run()
    except rospy.ROSInterruptException: pass
