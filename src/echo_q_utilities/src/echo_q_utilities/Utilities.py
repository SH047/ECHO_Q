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
