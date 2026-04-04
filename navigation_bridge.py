#!/usr/bin/env python3
"""
ECHO-Q Navigation Bridge
========================
Sits between the ROS Navigation Stack (move_base) and the ECHO-Q
locomotion controller. Handles:

  1. Republishing /cmd_vel → /echo_q/cmd_vel with safety clamping.
  2. Monitoring the robot footprint and sending a STOP on emergency.
  3. Publishing a nav_status topic for the web dashboard.
  4. Computing and publishing a simple cost-map-friendly footprint.

Architecture
------------
  move_base ──/cmd_vel──► [NavigationBridge] ──/echo_q/cmd_vel──► echo_q_driver
                                   │
                         /echo_q/state (feedback)
                                   │
                         /echo_q/nav_status

The bridge also monitors /echo_q/contacts; if all 4 feet slip
simultaneously (contacts = [0,0,0,0]) for > SLIP_TIMEOUT seconds,
it sends a zero cmd_vel to halt navigation until the robot recovers.
"""

import rospy
import json
import numpy as np

from geometry_msgs.msg import Twist, PolygonStamped, Point32
from std_msgs.msg      import String, Bool
from nav_msgs.msg      import Odometry


# ── Tunable safety parameters ─────────────────────────────────────────────────
MAX_LINEAR_VEL  = 0.60    # m/s  – hard cap forwarded to driver
MAX_ANGULAR_VEL = 1.00    # rad/s
SLIP_TIMEOUT    = 0.5     # s    – zero-contact duration before emergency halt
FOOTPRINT_X     = 0.34    # m    – robot body half-length
FOOTPRINT_Y     = 0.17    # m    – robot body half-width


class NavigationBridge:
    """ROS node bridging move_base navigation to ECHO-Q locomotion."""

    def __init__(self):
        rospy.init_node("echo_q_navigation_bridge")

        # ── State ─────────────────────────────────────────────────────────
        self._last_contacts      = [True] * 4
        self._slip_start         = None
        self._navigation_active  = False
        self._last_cmd_vel       = Twist()
        self._robot_state        = {}

        # ── Publishers ────────────────────────────────────────────────────
        self._pub_cmd    = rospy.Publisher("/cmd_vel",              Twist,          queue_size=1)
        self._pub_status = rospy.Publisher("/echo_q/nav_status",   String,         queue_size=5)
        self._pub_fp     = rospy.Publisher("/echo_q/footprint",    PolygonStamped, queue_size=1)
        self._pub_estop  = rospy.Publisher("/echo_q/emergency_stop", Bool,         queue_size=1)

        # ── Subscribers ───────────────────────────────────────────────────
        rospy.Subscriber("/move_base/cmd_vel", Twist,  self._cb_move_base_cmd)
        rospy.Subscriber("/echo_q/state",      String, self._cb_robot_state)
        rospy.Subscriber("/echo_q/contacts",   String, self._cb_contacts)

        # ── Timers ────────────────────────────────────────────────────────
        rospy.Timer(rospy.Duration(0.1),  self._publish_footprint)
        rospy.Timer(rospy.Duration(0.2),  self._publish_nav_status)
        rospy.Timer(rospy.Duration(0.05), self._slip_watchdog)

        rospy.loginfo("[NAV_BRIDGE] Ready. Forwarding move_base → ECHO-Q driver.")

    # ── Callbacks ─────────────────────────────────────────────────────────────

    def _cb_move_base_cmd(self, msg: Twist):
        """Clamp and forward velocity commands from move_base."""
        safe = Twist()
        safe.linear.x  = float(np.clip(msg.linear.x,  -MAX_LINEAR_VEL,  MAX_LINEAR_VEL))
        safe.linear.y  = float(np.clip(msg.linear.y,  -MAX_LINEAR_VEL,  MAX_LINEAR_VEL))
        safe.angular.z = float(np.clip(msg.angular.z, -MAX_ANGULAR_VEL, MAX_ANGULAR_VEL))
        self._last_cmd_vel    = safe
        self._navigation_active = True
        self._pub_cmd.publish(safe)

    def _cb_robot_state(self, msg: String):
        try:
            self._robot_state = json.loads(msg.data)
        except json.JSONDecodeError:
            pass

    def _cb_contacts(self, msg: String):
        try:
            self._last_contacts = json.loads(msg.data)
        except (json.JSONDecodeError, TypeError):
            pass

    # ── Timers ────────────────────────────────────────────────────────────────

    def _slip_watchdog(self, _event):
        """Halt navigation if all feet are airborne (full slip / fall)."""
        if not self._navigation_active:
            return

        if not any(self._last_contacts):
            if self._slip_start is None:
                self._slip_start = rospy.Time.now()
            elif (rospy.Time.now() - self._slip_start).to_sec() > SLIP_TIMEOUT:
                rospy.logwarn("[NAV_BRIDGE] ⚠  Slip detected – sending EMERGENCY STOP!")
                self._pub_cmd.publish(Twist())      # zero velocity
                self._pub_estop.publish(Bool(True))
                self._slip_start = None
        else:
            self._slip_start = None

    def _publish_footprint(self, _event):
        """Publish rectangular robot footprint for move_base costmap."""
        fp = PolygonStamped()
        fp.header.stamp    = rospy.Time.now()
        fp.header.frame_id = "base_link"
        corners = [
            ( FOOTPRINT_X,  FOOTPRINT_Y),
            ( FOOTPRINT_X, -FOOTPRINT_Y),
            (-FOOTPRINT_X, -FOOTPRINT_Y),
            (-FOOTPRINT_X,  FOOTPRINT_Y),
        ]
        for x, y in corners:
            p = Point32()
            p.x, p.y = x, y
            fp.polygon.points.append(p)
        self._pub_fp.publish(fp)

    def _publish_nav_status(self, _event):
        """Publish a JSON summary of navigation health."""
        status = {
            "nav_active"    : self._navigation_active,
            "cmd_vx"        : round(self._last_cmd_vel.linear.x, 3),
            "cmd_vy"        : round(self._last_cmd_vel.linear.y, 3),
            "cmd_omega"     : round(self._last_cmd_vel.angular.z, 3),
            "contacts"      : self._last_contacts,
            "slip_detected" : not any(self._last_contacts),
            "robot_mode"    : self._robot_state.get("mode", "UNKNOWN"),
            "odom_x"        : self._robot_state.get("odom_x", 0.0),
            "odom_y"        : self._robot_state.get("odom_y", 0.0),
        }
        self._pub_status.publish(json.dumps(status))

    def spin(self):
        rospy.spin()


if __name__ == "__main__":
    NavigationBridge().spin()
