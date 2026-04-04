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
