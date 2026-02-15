#!/usr/bin/env python3
import rospy
import numpy as np
import time

# Import ECHO-Q Library
from echo_q_control.Config import Configuration
from echo_q_control.Kinematics import inverse_kinematics
from echo_q_control.State import State, BehaviorState
from echo_q_control.Gait import GaitController
from echo_q_control.Command import Command
from std_msgs.msg import Float32 # Needed for IMU

# Import Hardware & Input
from echo_q_hardware_interfacing.HardwareInterface import HardwareInterface
from echo_q_input_interfacing.InputInterface import InputInterface

class EchoQ_Driver:
    def __init__(self):
        rospy.init_node('echo_q_driver')
        
        # 1. Load Configuration
        self.config = Configuration()
        self.state = State()
        
        # 2. Initialize Hardware (Servos)
        rospy.loginfo("Initializing PCA9685 Servo Driver...")
        self.hardware = HardwareInterface(self.config)
        
        # 3. Initialize Input (PS4 Joystick)
        self.input_interface = InputInterface(self.config)
        
        # --- NEW: IMU SUBSCRIBERS ---
        # Listens to the pitch/roll topics from Arduino
        rospy.Subscriber("/dingo_pitch", Float32, self.imu_pitch_callback)
        rospy.Subscriber("/dingo_roll", Float32, self.imu_roll_callback)
        # ---------------------------

        # 4. Initialize Gait Controller
        self.gait_controller = GaitController(self.config)

        # 5. Timing
        self.rate = rospy.Rate(50) # Run at 50Hz
        self.last_time = rospy.Time.now()

    def run(self):
        rospy.loginfo("ECHO-Q is ready! Press L1 to activate.")
        
        while not rospy.is_shutdown():
            current_time = rospy.Time.now()
            dt = (current_time - self.last_time).to_sec()
            self.last_time = current_time
            
            # --- A. Get User Command ---
            command = self.input_interface.get_command(self.state, 50)
            
            # --- B. Update State Machine ---
            if command.joystick_control_event == 1: 
                # Toggle between REST and TROT
                if self.state.behavior_state == BehaviorState.REST:
                    self.state.behavior_state = BehaviorState.TROT
                    rospy.loginfo("Mode: TROT (Active)")
                else:
                    self.state.behavior_state = BehaviorState.REST
                    rospy.loginfo("Mode: REST (Relaxed)") 
            
            # --- C. Execute Behavior ---
            if self.state.behavior_state == BehaviorState.REST:
                # Relax motors or hold pose
                pass
            
            elif self.state.behavior_state == BehaviorState.TROT:
                # 1. Update Gait Phase
                self.gait_controller.update(current_time.to_sec())
       
                # 2. Trajectory Calculation (Placeholder for Swing Control)
                # In full version, check self.gait_controller.contacts
            
                # 3. Inverse Kinematics
                default_stance = self.config.default_stance
                feet_positions = default_stance.copy()
                
                # Add Height/Roll/Pitch from Command + IMU Correction
                feet_positions[2, :] += command.height
                
                # Solve Angles
                joint_angles = inverse_kinematics(feet_positions, self.config)
                
                # 4. Send to Servos
                self.hardware.set_actuator_positions(joint_angles)
            
            self.rate.sleep()

    # --- NEW: IMU CALLBACK FUNCTIONS ---
    def imu_pitch_callback(self, msg):
        self.state.pitch = msg.data

    def imu_roll_callback(self, msg):
        self.state.roll = msg.data

if __name__ == '__main__':
    try:
        driver = EchoQ_Driver()
        driver.run()
    except rospy.ROSInterruptException:
        pass
