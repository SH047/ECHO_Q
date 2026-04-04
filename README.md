# ECHO-Q: Enhanced Cognitive Hybrid Quadruped

> **From Print Bed to Patrol in Hours.**
> *"A Quadruped Robot Dog built to break the complexity barrier."*

[![CI](https://github.com/SH047/ECHO_Q/actions/workflows/python-package.yml/badge.svg)](https://github.com/SH047/ECHO_Q/actions)
[![Tests](https://img.shields.io/badge/tests-58%20passing-brightgreen)](#)
[![ROS](https://img.shields.io/badge/ROS-Noetic-blue)](#)
[![Python](https://img.shields.io/badge/python-3.8%20|%203.9%20|%203.10%20|%203.11-blue)](#)
[![License](https://img.shields.io/badge/license-MIT-green)](#)

ECHO-Q is a high-performance, open-source quadrupedal robot built as a **Final Year Project**. Unlike commercial quadrupeds costing thousands of dollars, ECHO-Q uses accessible **COTS (Commercial Off-The-Shelf)** components and optimised **3D-printed mechanics** to produce a robot capable of autonomous navigation, real-time SLAM mapping, and stable dynamic gait — at a fraction of the commercial cost.

---

## Table of Contents

- [Why ECHO-Q?](#why-echo-q)
- [Architecture](#architecture)
- [Hardware Specifications](#hardware-specifications)
- [Software Stack](#software-stack)
- [Repository Structure](#repository-structure)
- [Getting Started](#getting-started)
  - [1. The Build](#1-the-build)
  - [2. The Circuit](#2-the-circuit)
  - [3. Software Installation](#3-software-installation)
  - [4. First Boot & Calibration](#4-first-boot--calibration)
  - [5. Operation Modes](#5-operation-modes)
- [Navigation Stack](#navigation-stack)
  - [SLAM Mapping](#slam-mapping)
  - [Autonomous Navigation](#autonomous-navigation)
- [Web Dashboard](#web-dashboard)
- [Running Tests](#running-tests)
- [Firmware](#firmware)
- [Troubleshooting](#troubleshooting)

---

## Why ECHO-Q?

Most robot dogs are either too expensive or too fragile. ECHO-Q hits the sweet spot:

| Feature | ECHO-Q | Commercial Alternatives |
|:--------|:-------|:------------------------|
| **Cost** | ~$300 USD | $10,000–$80,000 |
| **Assembly time** | ~8 hours | N/A (closed-source) |
| **Open source** | ✅ Full stack | ❌ |
| **ROS support** | ✅ Noetic + Nav Stack | Partial |
| **SLAM / Nav** | ✅ GMapping + move_base | Limited |
| **Autonomous nav** | ✅ move_base + DWA | Proprietary SDK |

---

## Architecture

```
┌─────────────────────────────────────────────────────────────────────┐
│                         ECHO-Q Software Stack                       │
├────────────────┬────────────────┬───────────────┬───────────────────┤
│  PERCEPTION    │  LOCALISATION  │   PLANNING    │    EXECUTION      │
│                │                │               │                   │
│  RPLiDAR A1    │  GMapping SLAM │  move_base    │  echo_q_driver    │
│  /scan ──────► │  map ────────► │  global plan  │  50 Hz loop       │
│                │                │  local  plan  │                   │
│  Pi Camera V2  │  AMCL (saved   │  DWA planner  │  IK solver        │
│  /camera/      │  map mode)     │  /cmd_vel ──► │  Gait controller  │
│  image/        │                │               │  FSM              │
│  compressed    │  BNO055 IMU    │  NavigationBr │                   │
│                │  /echo_q/imu/* │  idge         │  PCA9685 PWM      │
├────────────────┴────────────────┴───────────────┤  12× DS3225 servo │
│               Web Dashboard (Flask :5000)       │                   │
│   Live camera · Telemetry · Foot contacts       │  Arduino Nano     │
│   MJPEG stream · REST API · Manual cmd_vel      │  (IMU bridge)     │
└─────────────────────────────────────────────────┴───────────────────┘

ROS Topics
──────────
/scan                     ← RPLiDAR (LaserScan)
/echo_q/imu/{roll,pitch,yaw,cal}  ← Arduino/BNO055 (Float32)
/echo_q/odom              ← Dead-reckoned odometry (Odometry)
/echo_q/joint_states      ← Servo angles (JointState)
/echo_q/state             ← JSON telemetry snapshot (String, 10 Hz)
/echo_q/contacts          ← Foot contact mask (String)
/cmd_vel                  ← Velocity command (Twist)
/move_base/cmd_vel        ← move_base output → NavigationBridge
/map                      ← GMapping / map_server (OccupancyGrid)
```

---

## Hardware Specifications

| Component | Specification | Function |
|:----------|:--------------|:---------|
| **Brain** | Raspberry Pi 4 (4GB) + Pi Cam V2 | High-level logic, ROS Master, AI Visual Tracking |
| **Spine** | Arduino Nano | BNO055 IMU bridge via rosserial |
| **Muscles** | 12× DS3225 MG (25 kg·cm) | Waterproof High-Torque Metal Gear Digital Servos |
| **Senses** | BNO055 + RPLiDAR A1 | Orientation fusion · 2D SLAM |
| **Driver** | PCA9685 (16-ch PWM, I2C) | Servo PWM generation |
| **Display** | I2C OLED 128×64 | Real-time telemetry |
| **Skeleton** | SLA Resin + CF PLA hybrid | Resin for links, CF PLA for chassis |
| **Power** | Custom PDB + LiPo 3S | High-current delivery, voltage-sag protection |

> **Servo Recommendation:** While 25 kg·cm servos work, **≥ 35 kg·cm** is strongly recommended for payload capacity and outdoor terrain.
>
> **Power Warning:** Standard buck converters **will** brown-out under servo load. The custom Power Distribution Board (PDB) with heavy copper traces is **essential**.

---

## Software Stack

| Layer | Technology |
|:------|:-----------|
| OS | Ubuntu 20.04 LTS (Focal Fossa) |
| Middleware | ROS 1 Noetic Ninjemys |
| SLAM | GMapping (`gmapping`) |
| Localisation | AMCL |
| Navigation | `move_base` + DWA Local Planner + NavFn Global Planner |
| IMU bridge | `rosserial_python` ↔ Arduino |
| Servo driver | Adafruit ServoKit (PCA9685) |
| Kinematics | Custom Python analytical IK + 4-bar linkage correction |
| Gait | Phase-based diagonal trot, 50 Hz |
| Web dashboard | Flask + MJPEG streaming |
| Languages | Python 3 (control / AI) · C++ (kinematics option) · C (Arduino) |

---

## Repository Structure

```
ECHO_Q/
├── firmware/
│   └── echo_q_imu/
│       └── echo_q_imu.ino          # Arduino BNO055 → rosserial firmware
│
├── src/
│   ├── echo_q_bringup/             # Top-level launch files
│   │   └── launch/
│   │       └── robot.launch        # ← START HERE
│   │
│   ├── echo_q_control/             # Core locomotion loop
│   │   ├── scripts/
│   │   │   └── echo_q_driver.py   # Main 50 Hz ROS node
│   │   └── src/echo_q_control/
│   │       ├── Config.py           # All robot parameters
│   │       ├── State.py            # FSM + runtime state
│   │       ├── Command.py          # Command data class
│   │       ├── Gait.py             # Diagonal trot gait
│   │       └── Kinematics.py       # Analytical IK + FK
│   │
│   ├── echo_q_hardware_interfacing/
│   │   └── src/.../HardwareInterface.py  # PCA9685 servo driver
│   │
│   ├── echo_q_input_interfacing/
│   │   └── src/.../InputInterface.py     # PS4 joystick handler
│   │
│   ├── echo_q_navigation/          # move_base + AMCL integration
│   │   ├── config/
│   │   │   ├── costmap_common_params.yaml
│   │   │   ├── global_costmap_params.yaml
│   │   │   ├── local_costmap_params.yaml
│   │   │   ├── dwa_local_planner_params.yaml
│   │   │   └── move_base_params.yaml
│   │   ├── launch/
│   │   │   └── navigation.launch
│   │   ├── maps/                   # Save your maps here
│   │   └── scripts/
│   │       └── navigation_bridge.py  # move_base ↔ ECHO-Q bridge
│   │
│   ├── echo_q_slam/
│   │   └── launch/slam.launch      # GMapping SLAM launch
│   │
│   ├── echo_q_peripheral_interfacing/
│   │   ├── src/.../app.py          # Flask web server
│   │   └── templates/index.html    # Terminal-aesthetic dashboard
│   │
│   └── echo_q_utilities/
│       └── src/.../Utilities.py    # Deadband · filters · rotation math
│
├── tests/                          # pytest test suite (58 tests)
│   ├── conftest.py
│   ├── test_utilities.py
│   ├── test_kinematics.py
│   └── test_gait.py
│
└── requirements.txt
```

---

## Getting Started

### 1. The Build

STL files are in the `/stl` directory. [Download all STLs →](https://drive.google.com/file/d/1YwuytM52wO8_xmOiyafuNOSdY19ycysT/view)

**Hybrid Material Strategy:**

| Part | Material | Reason |
|:-----|:---------|:-------|
| Hip brackets, joint links | **SLA Resin** | Dimensional accuracy, low flex |
| Body chassis, motor mounts | **CF PLA** | Rigidity, light weight |
| All structural | 0.2 mm layer · 25% infill · Gyroid | Balance of strength and print speed |

### 2. The Circuit

```
Raspberry Pi 4
  Pin 3 (SDA) ──────► PCA9685 SDA ──► 12× DS3225 Servos
  Pin 5 (SCL) ──────► PCA9685 SCL
  USB 0       ──────► Arduino Nano (rosserial / IMU)
  USB 1       ──────► RPLiDAR A1
  CSI         ──────► Pi Camera V2

Arduino Nano
  A4 (SDA)    ──────► BNO055 SDA
  A5 (SCL)    ──────► BNO055 SCL

PCA9685 Channel Map:
  Ch 0,1,2    = FR Hip, Thigh, Calf
  Ch 4,5,6    = FL Hip, Thigh, Calf
  Ch 8,9,10   = BR Hip, Thigh, Calf
  Ch 12,13,14 = BL Hip, Thigh, Calf
```

### 3. Software Installation

```bash
# On the Raspberry Pi (Ubuntu 20.04 + ROS Noetic pre-installed)

# Clone the workspace
mkdir -p ~/echo_q_ws/src && cd ~/echo_q_ws/src
git clone https://github.com/SH047/ECHO_Q.git .

# Install Python dependencies
pip3 install -r ../requirements.txt --break-system-packages

# Install ROS dependencies
sudo apt-get install -y \
  ros-noetic-joy \
  ros-noetic-rplidar-ros \
  ros-noetic-rosserial-python \
  ros-noetic-gmapping \
  ros-noetic-navigation \
  ros-noetic-amcl \
  ros-noetic-map-server \
  ros-noetic-move-base \
  ros-noetic-dwa-local-planner \
  ros-noetic-tf2-ros

# Build the workspace
cd ~/echo_q_ws
catkin_make
source devel/setup.bash
echo "source ~/echo_q_ws/devel/setup.bash" >> ~/.bashrc

# Grant hardware permissions
sudo chmod 666 /dev/ttyUSB0   # RPLiDAR
sudo chmod 666 /dev/ttyUSB1   # Arduino
sudo chmod 666 /dev/i2c-1     # I2C (PCA9685)
```

### 4. First Boot & Calibration

**Arduino firmware** — Flash `firmware/echo_q_imu/echo_q_imu.ino` via Arduino IDE.
- Fast LED blink = IMU calibrating (rotate the robot through all axes)
- Slow LED blink = IMU fully calibrated and publishing

**Servo calibration** — Edit `CALIBRATION_OFFSETS` in `HardwareInterface.py`:
```python
# Tune these offsets (degrees) until the robot stands level
CALIBRATION_OFFSETS = np.array([
    [15,  95,  95,   3],   # Hip
    [60,   1,  60,   8],   # Thigh
    [110,-110, 150, -80],  # Calf
])
```

### 5. Operation Modes

#### Teleoperation only
```bash
roslaunch echo_q_bringup robot.launch
```

#### SLAM mapping
```bash
roslaunch echo_q_bringup robot.launch slam:=true rviz:=true
# Drive around to build the map, then save it:
rosrun map_server map_saver -f ~/echo_q_map
cp ~/echo_q_map.* src/echo_q_navigation/maps/
```

#### Autonomous navigation (requires saved map)
```bash
roslaunch echo_q_bringup robot.launch navigation:=true rviz:=true
# Send a 2D Nav Goal in RViz, or via CLI:
rostopic pub /move_base_simple/goal geometry_msgs/PoseStamped \
  "{ header: {frame_id: 'map'}, pose: {position: {x: 1.0, y: 0.0, z: 0.0}, \
   orientation: {w: 1.0}} }"
```

**PS4 Controller Map:**

| Button | Action |
|:-------|:-------|
| **L1** | ARM / DISARM (toggle) |
| **R1** | Switch to TROT mode |
| **R2** | Switch to NAVIGATE mode (autonomous) |
| **✕**  | HOP (experimental) |
| **Left Stick** | Move forward / strafe |
| **Right Stick** | Rotate (yaw) / pitch lean |
| **D-Pad ↑↓** | Adjust body height |
| **D-Pad ←→** | Adjust body roll |

---

## Navigation Stack

ECHO-Q ships with a fully configured ROS Navigation Stack.

### SLAM Mapping

Uses **GMapping** (Rao-Blackwellised particle filter SLAM):

```
RPLiDAR /scan ──► gmapping ──► /map (OccupancyGrid)
                      ▲
               /echo_q/odom  (dead-reckoned odometry)
```

Tuned for legged odometry (noisier than wheels) with higher motion noise parameters and a smaller particle count for real-time performance on the Pi 4.

### Autonomous Navigation

Uses the standard ROS Navigation Stack:

```
/map ──► NavFn (global planner) ──► DWA (local planner) ──► /cmd_vel
              ▲                             ▲
         global costmap               local costmap
              ▲                             ▲
         AMCL localisation          /scan (obstacle avoidance)
              ▲
         /echo_q/odom
```

**NavigationBridge** translates `move_base`'s `/cmd_vel` into ECHO-Q locomotion commands, with:
- Velocity clamping to safe trot limits
- Foot-slip detection (emergency halt if all 4 feet lose contact)
- Robot footprint publishing for accurate costmap inflation

---

## Web Dashboard

Access the live dashboard at `http://<ROBOT_IP>:5000`

Features:
- **Live MJPEG camera feed** from Pi Camera V2
- **Real-time telemetry**: mode, velocity, odometry, body height, RPY angles
- **Foot contact visualiser**: shows which feet are in stance vs swing
- **Roll/Pitch/Yaw bar graphs** updated at 5 Hz
- **System log**: streaming event log in the browser
- **REST API**: `POST /api/cmd` to send velocity commands from any HTTP client

```bash
# Manual velocity command via curl
curl -X POST http://<ROBOT_IP>:5000/api/cmd \
     -H 'Content-Type: application/json' \
     -d '{"vx": 0.3, "vy": 0.0, "omega": 0.0}'
```

---

## Running Tests

```bash
cd ~/echo_q_ws
pip3 install pytest --break-system-packages
python -m pytest tests/ -v

# With coverage report
pip3 install pytest-cov --break-system-packages
python -m pytest tests/ --cov=src --cov-report=term-missing
```

Current: **58 tests · 3 modules** (Utilities, Kinematics, Gait)

---

## Firmware

The Arduino Nano runs `firmware/echo_q_imu/echo_q_imu.ino`:

- Reads BNO055 in **NDOF fusion mode** (accelerometer + gyro + magnetometer)
- Publishes `/echo_q/imu/{roll,pitch,yaw}` (Float32, radians) at **50 Hz**
- Publishes `/echo_q/imu/cal` (UInt8, calibration bitmask)
- LED heartbeat: fast blink = calibrating, slow = nominal, SOS = IMU fault

Flash via Arduino IDE (Board: Nano, Processor: ATmega328P Old Bootloader).

---

## Troubleshooting

| Symptom | Cause | Fix |
|:--------|:------|:----|
| Servos jitter / reset | Battery voltage sag | Recharge; check PDB current rating |
| `"Device not found"` for controller | Joy not paired | `bluetoothctl` → pair, trust, connect |
| Lidar not detected | Wrong USB port | `ls /dev/ttyUSB*`, update launch param |
| IMU publishes zeros | Arduino not flashed / wrong baud | Reflash; check `serial_node` baud=115200 |
| Robot leans / can't stand level | Calibration offsets wrong | Edit `CALIBRATION_OFFSETS` in `HardwareInterface.py` |
| move_base costmap empty | Lidar TF missing | Check `tf_base_to_laser` static publisher |
| AMCL delocalises | Odometry too noisy | Increase `srr/srt/str/stt` in `slam.launch` |

---

**Developed by Shreyas S Rai · Rajarajeshwari College of Engineering**

[LinkedIn](https://www.linkedin.com/in/shreyas-s-rai/) · [Gallery & Assets](https://drive.google.com/drive/folders/1M44qh0k5k6DG8-52khi2ZnXKBumhORSk)
