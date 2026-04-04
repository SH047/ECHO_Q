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
