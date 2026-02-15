#  ECHO-Q User Manual

This guide provides step-by-step instructions for assembling, installing, and operating the **ECHO-Q Quadruped Robot**.

---

##  1. Hardware Requirements
To build ECHO-Q, you need the following components:

### Core Electronics
* **Computer:** Raspberry Pi 4 (4GB or 8GB recommended)
* **Microcontroller:** Arduino Nano (for IMU offloading)
* **Sensors:** * RPLiDAR A1 (2D Mapping)
    * BNO055 IMU (Orientation)
    * Pi Camera Module V2 (Visual Feed)
* **Actuators:** 12x DS3225 MG High-Torque Servos (25kg or 35kg)
* **Drivers:** PCA9685 16-Channel PWM Driver (I2C)
* **Power:** 2S or 3S LiPo Battery with High-Amp Buck Converter (5V/10A+)

### Mechanical Parts
* **Frame:** 3D Printed (Carbon Fiber PLA for chassis, SLA Resin for links)
* **Fasteners:** M3 Bolts & Nuts (Assorted lengths)
* **Bearings:** 8x22x7mm (608ZZ) for leg joints

---

##  2. Wiring Diagram

### Raspberry Pi 4 GPIO
| Pin | Function | Connection |
| :--- | :--- | :--- |
| **Pin 3 (GPIO 2)** | I2C SDA | PCA9685 SDA |
| **Pin 5 (GPIO 3)** | I2C SCL | PCA9685 SCL |
| **USB Port 1** | Serial | Arduino Nano (IMU) |
| **USB Port 2** | Serial | RPLiDAR A1 |
| **CSI Port** | Camera | Pi Camera Ribbon Cable |

### Servo Mapping (PCA9685)
| Leg | Hip Servo | Upper Leg Servo | Lower Leg Servo |
| :--- | :--- | :--- | :--- |
| **Front Right (FR)** | Channel 0 | Channel 1 | Channel 2 |
| **Front Left (FL)** | Channel 4 | Channel 5 | Channel 6 |
| **Back Right (BR)** | Channel 8 | Channel 9 | Channel 10 |
| **Back Left (BL)** | Channel 12 | Channel 13 | Channel 14 |

---

##  3. Software Installation

### Prerequisite
* **OS:** Ubuntu 20.04 LTS (Focal Fossa) server installed on Raspberry Pi.
* **ROS:** ROS Noetic Ninjemys installed.

### Setup Steps
1.  **Clone the Repository:**
    ```bash
    mkdir -p ~/echo_q_ws/src
    cd ~/echo_q_ws/src
    git clone [https://github.com/SH047/ECHO_Q.git](https://github.com/SH047/ECHO_Q.git) .
    ```

2.  **Install Dependencies:**
    ```bash
    cd ~/echo_q_ws
    pip3 install -r requirements.txt
    sudo apt-get install ros-noetic-joy ros-noetic-rplidar-ros
    ```

3.  **Build the Workspace:**
    ```bash
    cd ~/echo_q_ws
    catkin_make
    source devel/setup.bash
    ```

4.  **Enable Hardware Permissions:**
    ```bash
    sudo chmod 666 /dev/ttyUSB0  # Lidar/Arduino permission
    sudo chmod 666 /dev/i2c-1    # I2C permission
    ```

---

## 4. Operation

### Starting the Robot
1.  Power on the ECHO-Q (ensure LiPo is charged).
2.  SSH into the robot or connect a monitor.
3.  Run the launch command:
    ```bash
    roslaunch echo_q_control echo_q.launch
    ```

### Web Interface
Once the robot is running, open a browser on your laptop/phone and visit:
`http://<ROBOT_IP>:5000`
* View the live camera feed.
* Check system status.

### PS4 Controller Map
| Button | Action | Description |
| :--- | :--- | :--- |
| **L1** | **ARM / DISARM** | Activates the robot from REST mode. |
| **R1** | **Trot Mode** | Switches to Trotting gait. |
| **X** | **Hop** | Triggers a jump (Experimental). |
| **Left Stick** | **Move (X/Y)** | Forward/Backward & Strafe Left/Right. |
| **Right Stick** | **Look (Yaw)** | Rotate the robot left/right. |
| **D-Pad** | **Adjust Height** | Up/Down adjusts body height. |

---

##  Troubleshooting
* **Servos Jitter:** Check battery voltage. If < 7.4V, recharge immediately.
* **"Device not found":** Ensure `ros-noetic-joy` is installed and controller is paired via Bluetooth.
* **Lidar Error:** Check if `/dev/ttyUSB0` exists. You may need to swap USB ports.

---

**Developed by Shreyas S Rai**
*Rajarajeshwari College of Engineering*
