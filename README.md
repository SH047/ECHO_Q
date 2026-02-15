# ECHO-Q: Enhanced Cognitive Hybrid Quadruped
> **From Print Bed to Patrol in Hours.**
> *"A Quadruped Robot Dog built to break the complexity barrier."*

ECHO-Q is a high-performance, open-source quadrupedal robot designed as a **Final Year Project**. Unlike commercial quadrupeds costing thousands of dollars, ECHO-Q utilizes accessible **COTS (Commercial Off-The-Shelf)** components and optimized **3D-printed mechanics** to create a robot that is affordable, reproducible, and capable of stable locomotion.

---

## Why ECHO-Q?
Most robot dogs are either too expensive or too fragile. ECHO-Q hits the sweet spot:

*  **Rapid Deployment:** Designed to be assembled, wired, and code-deployed in a matter of hours.
*  **Budget-Friendly:** Built using standard Hobby Servos, Arduino, and Raspberry Pi.
*  **Custom Kinematics:** Features a custom-written **Inverse Kinematics (IK)** solver tailored to its specific geometry for fluid movement.
*  **Self-Stabilizing:** Implements sensor fusion (IMU) to detect tilts and correct posture in real-time.
*  **DfAM Optimized:** The chassis is engineered for **Additive Manufacturing**, requiring minimal support material and reducing print time/waste.

---

## The Anatomy of ECHO-Q

### Hardware Specifications

| Component | Specification | Function |
| :--- | :--- | :--- |
| **Brain** | Raspberry Pi 4 (4GB) + Pi Cam | High-level logic, ROS Master, AI Visual Tracking |
| **Spine** | Arduino Nano | Low-level gait execution, BNO055 Interface |
| **Muscles** | 12x DS3225 MG (25kg) | Waterproof High-Torque Metal Gear Digital Servos |
| **Senses** | BNO055 + RPLiDAR A1 | Orientation, Mapping (SLAM), and Balance |
| **Display** | I2C OLED Screen | Real-time telemetry and status visualization |
| **Skeleton** | Hybrid: SLA Resin + CF PLA | Resin for main links (stiffness), CF PLA for body |
| **Power** | Custom Power Distribution PCB | High-current delivery to prevent voltage sag |

> [!IMPORTANT]
> **Construction Note:** While 25kg servos were used, it is **highly recommended** to use higher torque servos (>30kg) for better payload capacity.
> **Power Note:** Standard buck converters will fail under load. A custom **Power Distribution Board (PDB)** with heavy copper traces is essential to prevent MCU resets.

### Software Architecture
* **OS:** Ubuntu 20.04.06 LTS (Focal Fossa)
* **Middleware:** ROS 1 Noetic
* **Communication:** `rosserial` (Pi ↔ Arduino)
* **Servo Driver:** PCA9685 (16-channel PWM driver) via I2C
* **Languages:** Python (High-level/AI), C++ (Kinematics Engine)
* **Input:** Wireless Gamepad (PS4/Xbox) or Keyboard Teleop

---


## 🚀 Getting Started

### 1. The Build
All STL files are located in the `/stl` directory. 

**Hybrid Material Strategy:**
* **Cosmetic & Joint Parts:** Printed in **SLA Resin** for high dimensional accuracy.
* **Structural Chassis:** Printed in **Carbon Fiber Infused PLA** for maximum rigidity.
* **Settings:** 0.2mm layer height, 20-30% Infill, Gyroid pattern.

### 2. The Circuit
1. Connect Servos to **PCA9685** driver.
2. Connect **BNO055 IMU** to Arduino I2C.
3. Connect **RPLiDAR** to Raspberry Pi USB.
4. Connect **OLED Screen** to Pi I2C.

### 3. Deployment
On the Raspberry Pi:
```bash
# Clone the repo
git clone [https://github.com/yourusername/ECHO-Q.git](https://github.com/yourusername/ECHO-Q.git)

# Launch the ROS Master, Lidar, and Drivers
roslaunch echo_q_bringup robot.launch
