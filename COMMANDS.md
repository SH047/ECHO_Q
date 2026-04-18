# ECHO-Q Useful Commands

Quick reference for common tasks during development, calibration, and operation.

---

## Build & Source

```bash
cd ~/echo_q_ws
catkin_make
source devel/setup.bash

# Add to ~/.bashrc so every terminal is sourced automatically
echo "source ~/echo_q_ws/devel/setup.bash" >> ~/.bashrc
```

---

## Launch Modes

```bash
# Teleoperation only (PS4 controller)
roslaunch echo_q_bringup robot.launch

# SLAM mapping (build a map while driving)
roslaunch echo_q_bringup robot.launch slam:=true rviz:=true

# Save the map after driving around
rosrun map_server map_saver -f ~/echo_q_map
cp ~/echo_q_map.* src/echo_q_navigation/maps/

# Autonomous navigation (requires saved map)
roslaunch echo_q_bringup robot.launch navigation:=true rviz:=true

# All features at once
roslaunch echo_q_bringup robot.launch slam:=true navigation:=true rviz:=true web:=true
```

---

## ROS Topic Inspection

```bash
# List all active topics
rostopic list

# Live telemetry snapshot (mode, velocity, IMU, contacts)
rostopic echo /echo_q/state

# Raw IMU values from Arduino
rostopic echo /echo_q/imu/roll
rostopic echo /echo_q/imu/pitch
rostopic echo /echo_q/imu/yaw

# Foot contact mask (true = on ground)
rostopic echo /echo_q/contacts

# Joint angles (radians)
rostopic echo /echo_q/joint_states

# Dead-reckoned odometry
rostopic echo /echo_q/odom

# LiDAR scan
rostopic echo /scan

# Camera (compressed JPEG stream)
rostopic echo /camera/image/compressed
```

---

## Manual Velocity Commands

```bash
# Move forward at 0.3 m/s
rostopic pub /cmd_vel geometry_msgs/Twist \
  "{ linear: {x: 0.3, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0} }" -r 10

# Rotate in place (1.0 rad/s)
rostopic pub /cmd_vel geometry_msgs/Twist \
  "{ linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 1.0} }" -r 10

# Stop
rostopic pub /cmd_vel geometry_msgs/Twist "{}" -1

# Send autonomous nav goal from CLI
rostopic pub /move_base_simple/goal geometry_msgs/PoseStamped \
  "{ header: {frame_id: 'map'}, pose: {position: {x: 1.0, y: 0.0, z: 0.0}, orientation: {w: 1.0}} }"
```

---

## Hardware Permissions

```bash
# Required after every reboot (or add to /etc/rc.local)
sudo chmod 666 /dev/ttyUSB0   # RPLiDAR
sudo chmod 666 /dev/ttyUSB1   # Arduino (IMU)
sudo chmod 666 /dev/i2c-1     # PCA9685 (servos)

# Check which device is on which USB port
ls -la /dev/ttyUSB*
dmesg | grep ttyUSB | tail -10
```

---

## Diagnostics

```bash
# Check TF tree (should show map → odom → base_link → laser)
rosrun tf view_frames && evince frames.pdf

# Check node connectivity
rosrun rqt_graph rqt_graph

# Check topic publish rates
rostopic hz /scan              # should be ~10 Hz
rostopic hz /echo_q/imu/roll  # should be ~50 Hz
rostopic hz /echo_q/state     # should be ~10 Hz

# System resource usage on the Pi
htop

# Check I2C devices (PCA9685 should show at 0x40)
sudo i2cdetect -y 1
```

---

## Tests

```bash
cd ~/echo_q_ws

# Run all 25 tests
python -m pytest tests/ -v

# Run with coverage report
python -m pytest tests/ --cov=src --cov-report=term-missing

# Run a single test file
python -m pytest tests/test_kinematics.py -v
```

---

## Web Dashboard

```bash
# Access from any device on the same network
http://<ROBOT_IP>:5000

# Find the robot's IP
hostname -I

# Send velocity command via curl
curl -X POST http://<ROBOT_IP>:5000/api/cmd \
     -H 'Content-Type: application/json' \
     -d '{"vx": 0.3, "omega": 0.0}'
```

---

## Git Workflow

```bash
# Check status
git status
git log --oneline -10

# Pull latest changes
cd ~/echo_q_ws/src
git pull origin main

# After pulling, always rebuild
cd ~/echo_q_ws
catkin_make && source devel/setup.bash
```
