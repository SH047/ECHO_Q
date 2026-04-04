#!/usr/bin/env python3
"""Bridges move_base /cmd_vel → ECHO-Q driver with slip watchdog."""
import rospy, json, numpy as np
from geometry_msgs.msg import Twist, PolygonStamped, Point32
from std_msgs.msg      import String, Bool

MAX_LIN, MAX_ANG = 0.60, 1.00
SLIP_TIMEOUT     = 0.5

class NavigationBridge:
    def __init__(self):
        rospy.init_node("echo_q_navigation_bridge")
        self._contacts = [True]*4; self._slip_start = None; self._last = Twist()
        self._pub_cmd   = rospy.Publisher("/cmd_vel",               Twist,          queue_size=1)
        self._pub_fp    = rospy.Publisher("/echo_q/footprint",      PolygonStamped, queue_size=1)
        self._pub_estop = rospy.Publisher("/echo_q/emergency_stop", Bool,           queue_size=1)
        rospy.Subscriber("/move_base/cmd_vel", Twist,  self._cb_cmd)
        rospy.Subscriber("/echo_q/contacts",   String, self._cb_contacts)
        rospy.Timer(rospy.Duration(0.1),  self._pub_footprint)
        rospy.Timer(rospy.Duration(0.05), self._watchdog)
        rospy.loginfo("[NAV_BRIDGE] Ready.")

    def _cb_cmd(self, msg):
        safe = Twist()
        safe.linear.x  = float(np.clip(msg.linear.x,  -MAX_LIN, MAX_LIN))
        safe.linear.y  = float(np.clip(msg.linear.y,  -MAX_LIN, MAX_LIN))
        safe.angular.z = float(np.clip(msg.angular.z, -MAX_ANG, MAX_ANG))
        self._last = safe; self._pub_cmd.publish(safe)

    def _cb_contacts(self, msg):
        try: self._contacts = json.loads(msg.data)
        except: pass

    def _watchdog(self, _):
        if not any(self._contacts):
            if self._slip_start is None: self._slip_start = rospy.Time.now()
            elif (rospy.Time.now()-self._slip_start).to_sec() > SLIP_TIMEOUT:
                rospy.logwarn("[NAV_BRIDGE] Slip – EMERGENCY STOP")
                self._pub_cmd.publish(Twist()); self._pub_estop.publish(Bool(True))
                self._slip_start = None
        else: self._slip_start = None

    def _pub_footprint(self, _):
        fp = PolygonStamped(); fp.header.stamp = rospy.Time.now(); fp.header.frame_id = "base_link"
        for x, y in [(0.34,0.17),(0.34,-0.17),(-0.34,-0.17),(-0.34,0.17)]:
            p = Point32(); p.x = x; p.y = y; fp.polygon.points.append(p)
        self._pub_fp.publish(fp)

    def spin(self): rospy.spin()

if __name__ == "__main__": NavigationBridge().spin()
