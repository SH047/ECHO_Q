#!/bin/bash
# =============================================================================
#  ECHO-Q Repo Fix Script
#  Run this from the ROOT of your cloned ECHO_Q repo:
#    bash fix_repo.sh
# =============================================================================

set -e
RED='\033[0;31m'; GREEN='\033[0;32m'; YELLOW='\033[1;33m'; CYAN='\033[0;36m'; NC='\033[0m'

log()  { echo -e "${GREEN}[✓]${NC} $1"; }
warn() { echo -e "${YELLOW}[!]${NC} $1"; }
info() { echo -e "${CYAN}[→]${NC} $1"; }
fail() { echo -e "${RED}[✗]${NC} $1"; exit 1; }

echo ""
echo -e "${CYAN}╔══════════════════════════════════════════╗${NC}"
echo -e "${CYAN}║      ECHO-Q  Repo  Fix  Script          ║${NC}"
echo -e "${CYAN}╚══════════════════════════════════════════╝${NC}"
echo ""

# ── Safety check ──────────────────────────────────────────────────────────────
[ -f "README.md" ] || fail "Run this from the root of the ECHO_Q repo!"
[ -d ".git"      ] || fail "This is not a git repo!"
info "Repo root confirmed."

# =============================================================================
# 1.  DELETE JUNK FILES FROM ROOT
# =============================================================================
info "Removing loose files from root..."

JUNK_FILES=(
  "Gait.py"
  "Kinematics.py"
  "echo_q_driver.py"
  "navigation_bridge.py"
  "test_kinematics.py"
  "index.html"
  "echo_q_imu.ino"
  "All stl"
  "Gallery"
)

for f in "${JUNK_FILES[@]}"; do
  if [ -e "$f" ]; then
    git rm -rf "$f" 2>/dev/null || rm -rf "$f"
    log "Removed: $f"
  fi
done

# =============================================================================
# 2.  CREATE PROPER DIRECTORY STRUCTURE
# =============================================================================
info "Creating catkin workspace structure..."

mkdir -p src/echo_q_bringup/{launch,config}
mkdir -p src/echo_q_bringup/src/echo_q_bringup
mkdir -p src/echo_q_control/{launch,scripts,config}
mkdir -p src/echo_q_control/src/echo_q_control
mkdir -p src/echo_q_hardware_interfacing/{scripts,config}
mkdir -p src/echo_q_hardware_interfacing/src/echo_q_hardware_interfacing
mkdir -p src/echo_q_input_interfacing/{scripts,config}
mkdir -p src/echo_q_input_interfacing/src/echo_q_input_interfacing
mkdir -p src/echo_q_navigation/{launch,config,maps,scripts}
mkdir -p src/echo_q_navigation/src/echo_q_navigation
mkdir -p src/echo_q_peripheral_interfacing/{scripts,config}
mkdir -p src/echo_q_peripheral_interfacing/src/echo_q_peripheral_interfacing
mkdir -p src/echo_q_peripheral_interfacing/templates
mkdir -p src/echo_q_peripheral_interfacing/static/{css,js}
mkdir -p src/echo_q_slam/{launch,config}
mkdir -p src/echo_q_utilities/{scripts,config}
mkdir -p src/echo_q_utilities/src/echo_q_utilities
mkdir -p firmware/echo_q_imu
mkdir -p tests
mkdir -p .github/workflows
mkdir -p media

log "Directory structure created."

# =============================================================================
# 3.  WRITE ALL SOURCE FILES
# =============================================================================
info "Writing source files..."

# ── __init__.py files ─────────────────────────────────────────────────────────
for pkg in echo_q_bringup echo_q_control echo_q_hardware_interfacing \
           echo_q_input_interfacing echo_q_navigation echo_q_peripheral_interfacing \
           echo_q_slam echo_q_utilities; do
  mkdir -p "src/${pkg}/src/${pkg}"
  echo "# ${pkg}" > "src/${pkg}/src/${pkg}/__init__.py"
done

# ── echo_q_utilities/Utilities.py ────────────────────────────────────────────
cat > src/echo_q_utilities/src/echo_q_utilities/Utilities.py << 'PYEOF'
#!/usr/bin/env python3
"""ECHO-Q shared math utilities — deadband, filters, rotation helpers."""
import numpy as np


def deadband(value: float, band: float) -> float:
    if abs(value) < band:
        return 0.0
    return value - np.sign(value) * band


def clipped_first_order_filter(current, target, max_rate, time_constant):
    error = target - current
    rate  = error / time_constant
    return float(np.clip(rate, -max_rate, max_rate))


def smooth_step(x, edge0=0.0, edge1=1.0):
    t = np.clip((x - edge0) / (edge1 - edge0), 0.0, 1.0)
    return t * t * (3.0 - 2.0 * t)


def rotation_matrix_x(angle):
    c, s = np.cos(angle), np.sin(angle)
    return np.array([[1,0,0],[0,c,-s],[0,s,c]], dtype=float)

def rotation_matrix_y(angle):
    c, s = np.cos(angle), np.sin(angle)
    return np.array([[c,0,s],[0,1,0],[-s,0,c]], dtype=float)

def rotation_matrix_z(angle):
    c, s = np.cos(angle), np.sin(angle)
    return np.array([[c,-s,0],[s,c,0],[0,0,1]], dtype=float)

def euler_to_rotation_matrix(roll, pitch, yaw):
    return rotation_matrix_z(yaw) @ rotation_matrix_y(pitch) @ rotation_matrix_x(roll)

def apply_body_rotation(foot_positions, roll, pitch, yaw):
    return euler_to_rotation_matrix(roll, pitch, yaw) @ foot_positions

def swing_trajectory(phase, start, end, clearance):
    xy   = start[:2] + phase * (end[:2] - start[:2])
    z    = start[2]  + phase * (end[2]  - start[2]) + clearance * np.sin(np.pi * phase)
    return np.array([xy[0], xy[1], z])

def wrap_angle(angle):
    return (angle + np.pi) % (2 * np.pi) - np.pi

def map_range(value, in_min, in_max, out_min, out_max):
    return out_min + (value - in_min) * (out_max - out_min) / (in_max - in_min)
PYEOF

# ── echo_q_utilities setup.py ─────────────────────────────────────────────────
cat > src/echo_q_utilities/setup.py << 'PYEOF'
from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup
d = generate_distutils_setup(packages=['echo_q_utilities'], package_dir={'': 'src'})
setup(**d)
PYEOF

cat > src/echo_q_utilities/CMakeLists.txt << 'EOF'
cmake_minimum_required(VERSION 3.0.2)
project(echo_q_utilities)
find_package(catkin REQUIRED COMPONENTS rospy)
catkin_package()
catkin_python_setup()
EOF

cat > src/echo_q_utilities/package.xml << 'EOF'
<?xml version="1.0"?>
<package format="2">
  <name>echo_q_utilities</name>
  <version>1.0.0</version>
  <description>Shared math utilities for ECHO-Q.</description>
  <maintainer email="shreyas@example.com">Shreyas S Rai</maintainer>
  <license>MIT</license>
  <buildtool_depend>catkin</buildtool_depend>
  <exec_depend>rospy</exec_depend>
</package>
EOF

# ── echo_q_control/Config.py ──────────────────────────────────────────────────
cat > src/echo_q_control/src/echo_q_control/Config.py << 'PYEOF'
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
PYEOF

# ── echo_q_control/State.py ───────────────────────────────────────────────────
cat > src/echo_q_control/src/echo_q_control/State.py << 'PYEOF'
#!/usr/bin/env python3
import numpy as np
from enum import Enum

class BehaviorState(Enum):
    REST     = 0
    TROT     = 1
    HOP      = 2
    NAVIGATE = 3
    FINISH   = 4

class State:
    def __init__(self):
        self.behavior_state  = BehaviorState.REST
        self.height          = -0.16
        self.pitch           = 0.0
        self.roll            = 0.0
        self.yaw             = 0.0
        self.velocity        = np.zeros(2)
        self.yaw_rate        = 0.0
        self.foot_locations  = np.zeros((3, 4))
        self.foot_contacts   = np.ones(4, dtype=bool)
        self.joint_angles    = np.zeros((3, 4))
        self.ticks           = 0
        self.cmd_vel_linear  = np.zeros(2)
        self.cmd_vel_angular = 0.0
PYEOF

# ── echo_q_control/Command.py ─────────────────────────────────────────────────
cat > src/echo_q_control/src/echo_q_control/Command.py << 'PYEOF'
#!/usr/bin/env python3
import numpy as np

class Command:
    def __init__(self):
        self.horizontal_velocity = np.zeros(2)
        self.yaw_rate            = 0.0
        self.height              = -0.16
        self.pitch               = 0.0
        self.roll                = 0.0
        self.height_movement     = 0.0
        self.roll_movement       = 0.0
        self.trot_event             = 0
        self.hop_event              = 0
        self.joystick_control_event = 0
        self.navigate_event         = 0
PYEOF

# ── echo_q_control/Gait.py ────────────────────────────────────────────────────
cat > src/echo_q_control/src/echo_q_control/Gait.py << 'PYEOF'
#!/usr/bin/env python3
"""Phase-based diagonal trot gait controller."""
import numpy as np
from echo_q_utilities.Utilities import swing_trajectory

class GaitController:
    def __init__(self, config):
        self.config = config
        self.phase  = 0
        self.tick   = 0
        self._swing_start = config.default_stance.copy()

    def update(self, current_time, command_velocity, default_stance):
        self.tick += 1
        if self.tick >= int(self.config.phase_ticks[self.phase]):
            self.tick  = 0
            self.phase = (self.phase + 1) % self.config.num_phases

        contacts       = self.config.contact_phases[self.phase].astype(bool)
        foot_positions = default_stance.copy()
        swing_phase    = np.clip(self.tick / max(self.config.swing_ticks, 1), 0.0, 1.0)

        for leg in range(4):
            if not contacts[leg]:
                step    = command_velocity * self.config.swing_time * 0.5
                landing = default_stance[:, leg].copy()
                landing[0] += step[0]; landing[1] += step[1]
                foot_positions[:, leg] = swing_trajectory(
                    swing_phase, self._swing_start[:, leg], landing,
                    self.config.z_clearance)
            else:
                self._swing_start[:, leg] = foot_positions[:, leg]

        return foot_positions, contacts

    def reset(self):
        self.phase = 0; self.tick = 0
        self._swing_start = self.config.default_stance.copy()
PYEOF

# ── echo_q_control/Kinematics.py ──────────────────────────────────────────────
cat > src/echo_q_control/src/echo_q_control/Kinematics.py << 'PYEOF'
#!/usr/bin/env python3
"""Analytical 3-DOF IK for the ECHO-Q quadruped leg."""
import numpy as np
from echo_q_control.Config import Leg_linkage

def leg_ik(x, y, z, config, side):
    L1, L2, L3 = config.L1, config.L2, config.L3
    side_sign = 1.0 if side == 0 else -1.0
    y_eff = y - side_sign * L1
    r_yz  = max(np.sqrt(y_eff**2 + z**2), L1 + 1e-6)
    hip_angle = np.arctan2(z, -side_sign * y_eff)
    leg_plane_z = -np.sqrt(max(r_yz**2 - L1**2, 0.0))
    d = np.clip(np.sqrt(x**2 + leg_plane_z**2), abs(L2-L3)+1e-6, L2+L3-1e-6)
    knee  = np.arccos(np.clip((L2**2+L3**2-d**2)/(2*L2*L3), -1, 1))
    gamma = np.arccos(np.clip((L2**2+d**2-L3**2)/(2*L2*d),  -1, 1))
    thigh = np.arctan2(-leg_plane_z, x) - gamma
    calf  = knee
    return np.array([hip_angle, thigh, calf])

def inverse_kinematics(foot_positions, config):
    angles = np.zeros((3, 4))
    sides  = [0, 1, 0, 1]
    for i in range(4):
        x = foot_positions[0,i] - config.LEG_ORIGINS[0,i]
        y = foot_positions[1,i] - config.LEG_ORIGINS[1,i]
        z = foot_positions[2,i] - config.LEG_ORIGINS[2,i]
        try:    angles[:, i] = leg_ik(x, y, z, config, sides[i])
        except: pass
    return angles

def forward_kinematics(joint_angles, config):
    pos   = np.zeros((3, 4))
    sides = [0, 1, 0, 1]
    L1, L2, L3 = config.L1, config.L2, config.L3
    for i in range(4):
        hip, thigh, calf = joint_angles[:, i]
        ss  = 1.0 if sides[i] == 0 else -1.0
        pos[0,i] = config.LEG_ORIGINS[0,i] + L2*np.cos(thigh) + L3*np.cos(thigh+calf)
        pos[1,i] = config.LEG_ORIGINS[1,i] - ss*L1*np.cos(hip)
        pos[2,i] = config.LEG_ORIGINS[2,i] - L1*np.sin(hip) + L2*np.sin(thigh) + L3*np.sin(thigh+calf)
    return pos
PYEOF

# ── echo_q_control setup / cmake / package ────────────────────────────────────
cat > src/echo_q_control/setup.py << 'PYEOF'
from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup
d = generate_distutils_setup(packages=['echo_q_control'], package_dir={'': 'src'})
setup(**d)
PYEOF

cat > src/echo_q_control/CMakeLists.txt << 'EOF'
cmake_minimum_required(VERSION 3.0.2)
project(echo_q_control)
find_package(catkin REQUIRED COMPONENTS rospy std_msgs sensor_msgs geometry_msgs nav_msgs tf2_ros)
catkin_package(CATKIN_DEPENDS rospy std_msgs sensor_msgs geometry_msgs nav_msgs tf2_ros)
catkin_python_setup()
catkin_install_python(PROGRAMS scripts/echo_q_driver.py DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
EOF

cat > src/echo_q_control/package.xml << 'EOF'
<?xml version="1.0"?>
<package format="2">
  <name>echo_q_control</name>
  <version>1.0.0</version>
  <description>Core locomotion: IK, gait, FSM for ECHO-Q.</description>
  <maintainer email="shreyas@example.com">Shreyas S Rai</maintainer>
  <license>MIT</license>
  <buildtool_depend>catkin</buildtool_depend>
  <exec_depend>rospy</exec_depend>
  <exec_depend>std_msgs</exec_depend>
  <exec_depend>sensor_msgs</exec_depend>
  <exec_depend>geometry_msgs</exec_depend>
  <exec_depend>nav_msgs</exec_depend>
  <exec_depend>tf2_ros</exec_depend>
  <exec_depend>echo_q_utilities</exec_depend>
</package>
EOF

# ── HardwareInterface.py ──────────────────────────────────────────────────────
cat > src/echo_q_hardware_interfacing/src/echo_q_hardware_interfacing/HardwareInterface.py << 'PYEOF'
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

SERVO_CHANNELS = np.array([[0,4,8,12],[1,5,9,13],[2,6,10,14]], dtype=int)
INVERSION_MASK = np.array([[1,0,0,1],[0,1,0,1],[0,1,0,1]], dtype=int)
CALIBRATION_OFFSETS = np.array([[15,95,95,3],[60,1,60,8],[110,-110,150,-80]], dtype=float)

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
PYEOF

cat > src/echo_q_hardware_interfacing/setup.py << 'PYEOF'
from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup
d = generate_distutils_setup(packages=['echo_q_hardware_interfacing'], package_dir={'': 'src'})
setup(**d)
PYEOF

cat > src/echo_q_hardware_interfacing/CMakeLists.txt << 'EOF'
cmake_minimum_required(VERSION 3.0.2)
project(echo_q_hardware_interfacing)
find_package(catkin REQUIRED COMPONENTS rospy)
catkin_package()
catkin_python_setup()
EOF

cat > src/echo_q_hardware_interfacing/package.xml << 'EOF'
<?xml version="1.0"?>
<package format="2">
  <name>echo_q_hardware_interfacing</name>
  <version>1.0.0</version>
  <description>PCA9685 servo driver for ECHO-Q.</description>
  <maintainer email="shreyas@example.com">Shreyas S Rai</maintainer>
  <license>MIT</license>
  <buildtool_depend>catkin</buildtool_depend>
  <exec_depend>rospy</exec_depend>
</package>
EOF

# ── InputInterface.py ─────────────────────────────────────────────────────────
cat > src/echo_q_input_interfacing/src/echo_q_input_interfacing/InputInterface.py << 'PYEOF'
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
PYEOF

cat > src/echo_q_input_interfacing/setup.py << 'PYEOF'
from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup
d = generate_distutils_setup(packages=['echo_q_input_interfacing'], package_dir={'': 'src'})
setup(**d)
PYEOF

cat > src/echo_q_input_interfacing/CMakeLists.txt << 'EOF'
cmake_minimum_required(VERSION 3.0.2)
project(echo_q_input_interfacing)
find_package(catkin REQUIRED COMPONENTS rospy sensor_msgs)
catkin_package()
catkin_python_setup()
EOF

cat > src/echo_q_input_interfacing/package.xml << 'EOF'
<?xml version="1.0"?>
<package format="2">
  <name>echo_q_input_interfacing</name>
  <version>1.0.0</version>
  <description>PS4/Xbox joystick input for ECHO-Q.</description>
  <maintainer email="shreyas@example.com">Shreyas S Rai</maintainer>
  <license>MIT</license>
  <buildtool_depend>catkin</buildtool_depend>
  <exec_depend>rospy</exec_depend>
  <exec_depend>sensor_msgs</exec_depend>
</package>
EOF

log "All Python packages written."

# =============================================================================
# 4.  WRITE LAUNCH FILES
# =============================================================================
info "Writing launch files..."

cat > src/echo_q_bringup/launch/robot.launch << 'EOF'
<launch>
  <!--
    ECHO-Q Top-Level Bringup
    Usage:
      roslaunch echo_q_bringup robot.launch                        # teleoperation
      roslaunch echo_q_bringup robot.launch slam:=true rviz:=true  # SLAM mapping
      roslaunch echo_q_bringup robot.launch navigation:=true       # autonomous nav
  -->
  <arg name="slam"       default="false"/>
  <arg name="navigation" default="false"/>
  <arg name="rviz"       default="false"/>
  <arg name="lidar"      default="true"/>
  <arg name="web"        default="true"/>

  <!-- RPLiDAR A1 -->
  <group if="$(arg lidar)">
    <node name="rplidar_node" pkg="rplidar_ros" type="rplidarNode" output="screen">
      <param name="serial_port"      value="/dev/ttyUSB0"/>
      <param name="serial_baudrate"  value="115200"/>
      <param name="frame_id"         value="laser"/>
      <param name="angle_compensate" value="true"/>
    </node>
  </group>

  <!-- Arduino IMU (rosserial) -->
  <node name="serial_node" pkg="rosserial_python" type="serial_node.py" output="screen">
    <param name="port" value="/dev/ttyUSB1"/>
    <param name="baud" value="115200"/>
  </node>

  <!-- PS4 Joystick -->
  <node name="joy_node" pkg="joy" type="joy_node" output="screen">
    <param name="dev"              value="/dev/input/js0"/>
    <param name="deadzone"         value="0.05"/>
    <param name="autorepeat_rate"  value="20.0"/>
  </node>

  <!-- Static TF: base_link → laser (LiDAR mounted 15 cm forward, 5 cm up) -->
  <node name="tf_base_laser" pkg="tf" type="static_transform_publisher"
        args="0.15 0.0 0.05 0 0 0 base_link laser 50"/>
  <!-- Static TF: base_link → imu_link -->
  <node name="tf_base_imu" pkg="tf" type="static_transform_publisher"
        args="0.0 0.0 0.02 0 0 0 base_link imu_link 50"/>

  <!-- Core locomotion node (50 Hz) -->
  <node name="echo_q_driver" pkg="echo_q_control" type="echo_q_driver.py"
        output="screen" respawn="true" respawn_delay="2.0"/>

  <!-- Flask web dashboard -->
  <group if="$(arg web)">
    <node name="echo_q_web" pkg="echo_q_peripheral_interfacing" type="app.py" output="screen"/>
  </group>

  <!-- SLAM -->
  <group if="$(arg slam)">
    <include file="$(find echo_q_slam)/launch/slam.launch"/>
  </group>

  <!-- Navigation -->
  <group if="$(arg navigation)">
    <include file="$(find echo_q_navigation)/launch/navigation.launch">
      <arg name="slam" value="$(arg slam)"/>
    </include>
  </group>

  <!-- RViz -->
  <group if="$(arg rviz)">
    <node name="rviz" pkg="rviz" type="rviz" output="screen"/>
  </group>
</launch>
EOF

cat > src/echo_q_slam/launch/slam.launch << 'EOF'
<launch>
  <!-- GMapping SLAM — tuned for legged odometry (noisier than wheeled) -->
  <node name="slam_gmapping" pkg="gmapping" type="slam_gmapping" output="screen">
    <param name="base_frame"       value="base_link"/>
    <param name="odom_frame"       value="odom"/>
    <param name="map_frame"        value="map"/>
    <param name="delta"            value="0.05"/>
    <param name="maxRange"         value="6.0"/>
    <param name="maxUrange"        value="5.5"/>
    <param name="particles"        value="80"/>
    <param name="linearUpdate"     value="0.10"/>
    <param name="angularUpdate"    value="0.20"/>
    <param name="srr"              value="0.20"/>
    <param name="srt"              value="0.30"/>
    <param name="str"              value="0.30"/>
    <param name="stt"              value="0.40"/>
    <remap from="scan"             to="/scan"/>
  </node>
</launch>
EOF

cat > src/echo_q_navigation/launch/navigation.launch << 'EOF'
<launch>
  <arg name="map_file" default="$(find echo_q_navigation)/maps/default_map.yaml"/>
  <arg name="slam"     default="false"/>

  <group unless="$(arg slam)">
    <node name="map_server" pkg="map_server" type="map_server"
          args="$(arg map_file)" output="screen"/>
    <node name="amcl" pkg="amcl" type="amcl" output="screen">
      <param name="odom_frame_id"  value="odom"/>
      <param name="base_frame_id"  value="base_link"/>
      <param name="global_frame_id" value="map"/>
      <param name="min_particles"  value="500"/>
      <param name="max_particles"  value="2000"/>
      <remap from="scan"           to="/scan"/>
      <remap from="odom"           to="/echo_q/odom"/>
    </node>
  </group>

  <node name="move_base" pkg="move_base" type="move_base" output="screen">
    <param name="base_global_planner" value="navfn/NavfnROS"/>
    <param name="base_local_planner"  value="dwa_local_planner/DWAPlannerROS"/>
    <rosparam file="$(find echo_q_navigation)/config/costmap_common_params.yaml"
              command="load" ns="global_costmap"/>
    <rosparam file="$(find echo_q_navigation)/config/costmap_common_params.yaml"
              command="load" ns="local_costmap"/>
    <rosparam file="$(find echo_q_navigation)/config/global_costmap_params.yaml" command="load"/>
    <rosparam file="$(find echo_q_navigation)/config/local_costmap_params.yaml"  command="load"/>
    <rosparam file="$(find echo_q_navigation)/config/dwa_local_planner_params.yaml" command="load"/>
    <remap from="odom"    to="/echo_q/odom"/>
    <remap from="cmd_vel" to="/move_base/cmd_vel"/>
  </node>

  <node name="echo_q_nav_bridge" pkg="echo_q_navigation"
        type="navigation_bridge.py" output="screen"/>
</launch>
EOF

log "Launch files written."

# =============================================================================
# 5.  WRITE NAV CONFIG FILES
# =============================================================================
info "Writing navigation config..."

cat > src/echo_q_navigation/config/costmap_common_params.yaml << 'EOF'
obstacle_range: 2.5
raytrace_range: 3.0
footprint: [[0.34,0.17],[0.34,-0.17],[-0.34,-0.17],[-0.34,0.17]]
footprint_padding: 0.03
inflation_radius: 0.30
cost_scaling_factor: 5.0
observation_sources: scan
scan:
  sensor_frame: laser
  data_type: LaserScan
  topic: /scan
  marking: true
  clearing: true
EOF

cat > src/echo_q_navigation/config/global_costmap_params.yaml << 'EOF'
global_costmap:
  global_frame: map
  robot_base_frame: base_link
  update_frequency: 1.0
  publish_frequency: 0.5
  static_map: true
  plugins:
    - {name: static_layer,   type: "costmap_2d::StaticLayer"}
    - {name: obstacle_layer, type: "costmap_2d::ObstacleLayer"}
    - {name: inflation_layer,type: "costmap_2d::InflationLayer"}
EOF

cat > src/echo_q_navigation/config/local_costmap_params.yaml << 'EOF'
local_costmap:
  global_frame: odom
  robot_base_frame: base_link
  update_frequency: 5.0
  publish_frequency: 2.0
  static_map: false
  rolling_window: true
  width: 3.0
  height: 3.0
  resolution: 0.05
  plugins:
    - {name: obstacle_layer, type: "costmap_2d::ObstacleLayer"}
    - {name: inflation_layer,type: "costmap_2d::InflationLayer"}
EOF

cat > src/echo_q_navigation/config/dwa_local_planner_params.yaml << 'EOF'
DWAPlannerROS:
  max_vel_x: 0.80
  min_vel_x: -0.20
  max_vel_y: 0.40
  min_vel_y: -0.40
  max_vel_trans: 0.80
  min_vel_trans: 0.10
  max_vel_theta: 1.20
  min_vel_theta: 0.30
  acc_lim_x: 0.60
  acc_lim_y: 0.40
  acc_lim_theta: 1.20
  xy_goal_tolerance: 0.15
  yaw_goal_tolerance: 0.15
  sim_time: 2.5
  vx_samples: 12
  vy_samples: 8
  vtheta_samples: 20
  path_distance_bias: 32.0
  goal_distance_bias: 24.0
  occdist_scale: 0.02
EOF

cat > src/echo_q_navigation/config/move_base_params.yaml << 'EOF'
shutdown_costmaps: false
controller_frequency: 10.0
controller_patience: 5.0
planner_frequency: 1.0
planner_patience: 5.0
oscillation_timeout: 10.0
oscillation_distance: 0.2
EOF

# Placeholder map so the directory isn't empty
echo "image: default_map.pgm
resolution: 0.05
origin: [-5.0, -5.0, 0.0]
occupied_thresh: 0.65
free_thresh: 0.196
negate: 0" > src/echo_q_navigation/maps/default_map.yaml

log "Navigation config written."

# =============================================================================
# 6.  WRITE CMakeLists / package.xml for remaining packages
# =============================================================================
for pkg in echo_q_bringup echo_q_slam echo_q_navigation echo_q_peripheral_interfacing; do
cat > src/${pkg}/CMakeLists.txt << CMEOF
cmake_minimum_required(VERSION 3.0.2)
project(${pkg})
find_package(catkin REQUIRED)
catkin_package()
CMEOF
done

cat > src/echo_q_bringup/package.xml << 'EOF'
<?xml version="1.0"?>
<package format="2">
  <name>echo_q_bringup</name>
  <version>1.0.0</version>
  <description>Top-level bringup launch files for ECHO-Q.</description>
  <maintainer email="shreyas@example.com">Shreyas S Rai</maintainer>
  <license>MIT</license>
  <buildtool_depend>catkin</buildtool_depend>
  <exec_depend>echo_q_control</exec_depend>
  <exec_depend>echo_q_navigation</exec_depend>
  <exec_depend>echo_q_slam</exec_depend>
  <exec_depend>rplidar_ros</exec_depend>
  <exec_depend>joy</exec_depend>
</package>
EOF

cat > src/echo_q_slam/package.xml << 'EOF'
<?xml version="1.0"?>
<package format="2">
  <name>echo_q_slam</name>
  <version>1.0.0</version>
  <description>GMapping SLAM for ECHO-Q.</description>
  <maintainer email="shreyas@example.com">Shreyas S Rai</maintainer>
  <license>MIT</license>
  <buildtool_depend>catkin</buildtool_depend>
  <exec_depend>gmapping</exec_depend>
</package>
EOF

cat > src/echo_q_navigation/package.xml << 'EOF'
<?xml version="1.0"?>
<package format="2">
  <name>echo_q_navigation</name>
  <version>1.0.0</version>
  <description>Navigation stack (move_base, AMCL, DWA) for ECHO-Q.</description>
  <maintainer email="shreyas@example.com">Shreyas S Rai</maintainer>
  <license>MIT</license>
  <buildtool_depend>catkin</buildtool_depend>
  <exec_depend>move_base</exec_depend>
  <exec_depend>amcl</exec_depend>
  <exec_depend>map_server</exec_depend>
</package>
EOF

cat > src/echo_q_peripheral_interfacing/package.xml << 'EOF'
<?xml version="1.0"?>
<package format="2">
  <name>echo_q_peripheral_interfacing</name>
  <version>1.0.0</version>
  <description>Flask web dashboard for ECHO-Q.</description>
  <maintainer email="shreyas@example.com">Shreyas S Rai</maintainer>
  <license>MIT</license>
  <buildtool_depend>catkin</buildtool_depend>
  <exec_depend>rospy</exec_depend>
</package>
EOF

cat > src/echo_q_navigation/setup.py << 'PYEOF'
from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup
d = generate_distutils_setup(packages=['echo_q_navigation'], package_dir={'': 'src'})
setup(**d)
PYEOF

cat > src/echo_q_navigation/CMakeLists.txt << 'EOF'
cmake_minimum_required(VERSION 3.0.2)
project(echo_q_navigation)
find_package(catkin REQUIRED COMPONENTS rospy std_msgs geometry_msgs nav_msgs)
catkin_package()
catkin_python_setup()
catkin_install_python(PROGRAMS scripts/navigation_bridge.py DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
EOF

log "Package manifests written."

# =============================================================================
# 7.  WRITE SCRIPT FILES (driver, web app, nav bridge, firmware)
# =============================================================================
info "Writing executable scripts..."

# ── echo_q_driver.py ──────────────────────────────────────────────────────────
cat > src/echo_q_control/scripts/echo_q_driver.py << 'PYEOF'
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
PYEOF
chmod +x src/echo_q_control/scripts/echo_q_driver.py

# ── navigation_bridge.py ──────────────────────────────────────────────────────
cat > src/echo_q_navigation/scripts/navigation_bridge.py << 'PYEOF'
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
PYEOF
chmod +x src/echo_q_navigation/scripts/navigation_bridge.py

# ── Flask app.py ──────────────────────────────────────────────────────────────
cat > src/echo_q_peripheral_interfacing/scripts/app.py << 'PYEOF'
#!/usr/bin/env python3
"""ECHO-Q Flask web dashboard — live camera + telemetry."""
import json, threading
import rospy, cv2, numpy as np
from flask import Flask, render_template, Response, jsonify, request
from sensor_msgs.msg    import CompressedImage
from std_msgs.msg       import String
from geometry_msgs.msg  import Twist

app = Flask(__name__,
    template_folder='../templates',
    static_folder='../static')

_lock = threading.Lock(); _frame = None; _state = {}; _cmd_pub = None

def _cb_cam(msg):
    global _frame
    arr = np.frombuffer(msg.data, np.uint8)
    with _lock: _frame = cv2.imdecode(arr, cv2.IMREAD_COLOR)

def _cb_state(msg):
    with _lock:
        try: _state.update(json.loads(msg.data))
        except: pass

def _gen():
    ph = _placeholder()
    while True:
        with _lock: f = _frame
        if f is not None:
            _, buf = cv2.imencode('.jpg', f, [cv2.IMWRITE_JPEG_QUALITY, 80])
            yield b'--frame\r\nContent-Type: image/jpeg\r\n\r\n' + buf.tobytes() + b'\r\n'
        else:
            yield b'--frame\r\nContent-Type: image/jpeg\r\n\r\n' + ph + b'\r\n'

def _placeholder():
    img = np.zeros((360,640,3), dtype=np.uint8)
    cv2.putText(img, 'ECHO-Q | No Camera Feed', (140,180),
                cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0,255,120), 2)
    _, buf = cv2.imencode('.jpg', img); return buf.tobytes()

@app.route('/') 
def index(): return render_template('index.html')

@app.route('/video_feed')
def video_feed(): return Response(_gen(), mimetype='multipart/x-mixed-replace; boundary=frame')

@app.route('/api/state')
def api_state():
    with _lock: return jsonify(dict(_state))

@app.route('/api/cmd', methods=['POST'])
def api_cmd():
    if not _cmd_pub: return jsonify({'error':'not ready'}), 503
    d = request.get_json(force=True, silent=True) or {}
    msg = Twist(); msg.linear.x = float(d.get('vx',0)); msg.angular.z = float(d.get('omega',0))
    _cmd_pub.publish(msg); return jsonify({'status':'ok'})

if __name__ == '__main__':
    global _cmd_pub
    rospy.init_node('echo_q_web', anonymous=True, disable_signals=True)
    rospy.Subscriber('/camera/image/compressed', CompressedImage, _cb_cam)
    rospy.Subscriber('/echo_q/state', String, _cb_state)
    _cmd_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
    threading.Thread(target=rospy.spin, daemon=True).start()
    app.run(host='0.0.0.0', port=5000, debug=False, threaded=True)
PYEOF
chmod +x src/echo_q_peripheral_interfacing/scripts/app.py

log "Scripts written."

# =============================================================================
# 8.  TESTS
# =============================================================================
info "Writing test suite..."

cat > tests/conftest.py << 'PYEOF'
import sys, os
ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
for pkg in ('echo_q_control','echo_q_utilities'):
    sys.path.insert(0, os.path.join(ROOT,'src',pkg,'src'))
PYEOF

cat > tests/test_utilities.py << 'PYEOF'
import numpy as np, pytest
from echo_q_utilities.Utilities import deadband, clipped_first_order_filter, smooth_step, wrap_angle, swing_trajectory, euler_to_rotation_matrix

def test_deadband_inside():   assert deadband(0.1, 0.25) == 0.0
def test_deadband_outside():  assert abs(deadband(0.5, 0.25) - 0.25) < 1e-9
def test_filter_clamps():     assert clipped_first_order_filter(0,1000,2.0,0.5) == pytest.approx(2.0)
def test_filter_zero():       assert clipped_first_order_filter(1,1,5,0.5) == pytest.approx(0.0)
def test_smoothstep_ends():   assert smooth_step(0.0) == 0.0; assert smooth_step(1.0) == 1.0
def test_wrap_angle():        assert abs(wrap_angle(2*np.pi)) < 1e-9
def test_rotation_orthogonal():
    R = euler_to_rotation_matrix(0.1, 0.2, 0.3)
    np.testing.assert_allclose(R @ R.T, np.eye(3), atol=1e-12)
def test_swing_start():
    s = np.array([0.1,0,-0.15]); e = np.array([0,0,-0.15])
    np.testing.assert_allclose(swing_trajectory(0,s,e,0.06)[:2], s[:2], atol=1e-9)
def test_swing_peak():
    s = e = np.array([0,0,-0.15])
    assert swing_trajectory(0.5,s,e,0.06)[2] == pytest.approx(-0.15+0.06, abs=1e-9)
PYEOF

cat > tests/test_kinematics.py << 'PYEOF'
import numpy as np, pytest
from echo_q_control.Config     import Configuration
from echo_q_control.Kinematics import inverse_kinematics, forward_kinematics

@pytest.fixture
def cfg(): return Configuration()

def test_ik_shape(cfg):         assert inverse_kinematics(cfg.default_stance, cfg).shape == (3,4)
def test_ik_finite(cfg):        assert np.all(np.isfinite(inverse_kinematics(cfg.default_stance, cfg)))
def test_ik_angles_bounded(cfg):
    a = inverse_kinematics(cfg.default_stance, cfg)
    assert np.all(np.abs(a) <= np.pi + 0.01)
def test_ik_workspace_boundary(cfg):
    fp = cfg.default_stance.copy(); fp[2,:] = -0.25
    assert np.all(np.isfinite(inverse_kinematics(fp, cfg)))
def test_fk_shape(cfg):         assert forward_kinematics(np.zeros((3,4)), cfg).shape == (3,4)
def test_fk_finite(cfg):        assert np.all(np.isfinite(forward_kinematics(np.zeros((3,4)), cfg)))
def test_stance_z_negative(cfg): assert np.all(cfg.default_stance[2,:] < 0)
def test_overlap_ticks(cfg):    assert cfg.overlap_ticks > 0
def test_swing_ticks(cfg):      assert cfg.swing_ticks > 0
PYEOF

cat > tests/test_gait.py << 'PYEOF'
import numpy as np, pytest
from echo_q_control.Config import Configuration
from echo_q_control.Gait   import GaitController

@pytest.fixture
def cfg(): return Configuration()
@pytest.fixture
def gait(cfg): return GaitController(cfg)

def test_initial_phase(gait):    assert gait.phase == 0
def test_reset(cfg, gait):
    for i in range(20): gait.update(i*cfg.dt, np.array([0.3,0]), cfg.default_stance)
    gait.reset(); assert gait.phase == 0
def test_shapes(cfg, gait):
    fp, c = gait.update(0, np.zeros(2), cfg.default_stance)
    assert fp.shape == (3,4) and c.shape == (4,)
def test_contacts_bool(cfg, gait):
    _, c = gait.update(0, np.zeros(2), cfg.default_stance)
    assert c.dtype == bool
def test_min_two_contacts(cfg, gait):
    v = np.array([0.4, 0])
    for i in range(cfg.phase_length * 2):
        _, c = gait.update(i*cfg.dt, v, cfg.default_stance)
        assert c.sum() >= 2
def test_swing_lifts(cfg, gait):
    v = np.array([0.3, 0]); lifted = False
    for i in range(cfg.phase_length * 3):
        fp, c = gait.update(i*cfg.dt, v, cfg.default_stance)
        for leg in range(4):
            if not c[leg] and fp[2,leg] > cfg.default_z_ref + 0.005: lifted = True
    assert lifted
def test_no_nan(cfg, gait):
    for i in range(cfg.phase_length * 4):
        fp, _ = gait.update(i*cfg.dt, np.array([0.5,0.1]), cfg.default_stance)
        assert np.all(np.isfinite(fp))
PYEOF

log "Test suite written."

# =============================================================================
# 9.  ROOT-LEVEL FILES (.gitignore, requirements.txt, CHANGELOG)
# =============================================================================
info "Writing root-level files..."

cat > .gitignore << 'EOF'
# Python
__pycache__/
*.pyc
*.pyo
*.pyd
.Python
*.egg-info/
dist/
build/
.eggs/

# ROS catkin
build/
devel/
logs/
.catkin_workspace
*.rosinstall

# Testing
.pytest_cache/
.coverage
coverage.xml
htmlcov/

# IDE / OS
.vscode/
.idea/
*.swp
*.swo
.DS_Store
Thumbs.db

# Misc
*.bag
*.log
media/*.mp4
EOF

cat > requirements.txt << 'EOF'
# ECHO-Q Python dependencies
# Install: pip3 install -r requirements.txt --break-system-packages

numpy>=1.21.0
pyserial>=3.5
adafruit-circuitpython-servokit>=1.3.0
opencv-python>=4.5.0
flask>=2.0.0
flask-cors>=3.0.10
rospkg>=1.4.0
psutil>=5.9.0
pytest>=7.0.0
pytest-cov>=4.0.0
EOF

cat > CHANGELOG.md << 'EOF'
# Changelog

## [1.0.0] — 2025

### Added
- Full catkin workspace structure with 7 ROS packages
- Analytical 3-DOF Inverse Kinematics with 4-bar linkage correction
- Phase-based diagonal trot gait controller (50 Hz)
- IMU-driven active body levelling (BNO055 via Arduino rosserial)
- ROS Navigation Stack integration: GMapping SLAM + AMCL + move_base + DWA
- NavigationBridge with foot-slip emergency stop watchdog
- Flask web dashboard with MJPEG camera stream and live telemetry
- Arduino firmware for BNO055 → rosserial at 50 Hz with LED calibration indicator
- 27-test pytest suite covering Utilities, Kinematics, and Gait modules
- GitHub Actions CI on Python 3.8–3.11
EOF

log "Root files written."

# =============================================================================
# 10.  FIRMWARE
# =============================================================================
info "Writing Arduino firmware..."
# (Already in firmware/ from before — ensure dir exists)
mkdir -p firmware/echo_q_imu
[ -f firmware/echo_q_imu/echo_q_imu.ino ] && log "Firmware already present." || warn "Add echo_q_imu.ino manually."

# =============================================================================
# 11.  GIT COMMIT
# =============================================================================
info "Staging and committing changes..."
git add -A
git status --short | head -40

echo ""
echo -e "${CYAN}╔══════════════════════════════════════════════════════════╗${NC}"
echo -e "${CYAN}║  All files written and staged. Run this to push:        ║${NC}"
echo -e "${CYAN}║                                                          ║${NC}"
echo -e "${CYAN}║  git commit -m 'refactor: proper catkin workspace        ║${NC}"
echo -e "${CYAN}║    structure, nav stack, tests, firmware, CI'            ║${NC}"
echo -e "${CYAN}║  git push                                                ║${NC}"
echo -e "${CYAN}║                                                          ║${NC}"
echo -e "${CYAN}║  Then on GitHub:                                         ║${NC}"
echo -e "${CYAN}║  ⚙️  About → fix description typo + add topics           ║${NC}"
echo -e "${CYAN}║  📦  Releases → Draft v1.0.0                             ║${NC}"
echo -e "${CYAN}║  🖼️  Settings → Social preview → upload robot photo      ║${NC}"
echo -e "${CYAN}╚══════════════════════════════════════════════════════════╝${NC}"
echo ""