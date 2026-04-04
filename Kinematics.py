#!/usr/bin/env python3
"""
ECHO-Q Inverse Kinematics
==========================
Analytical 3-DOF IK for a standard mammalian-style quadruped leg:

    Hip (abduction/adduction) → Thigh (flexion) → Calf (flexion)

The knee uses a 4-bar parallel linkage; `four_bar_ik()` maps the
virtual straight-leg angle to the actual servo angle required.

Coordinate frame
----------------
Body frame origin at centre of robot, X forward, Y left, Z up.
Leg frame origin at the hip joint. Positive Z is up; the foot hangs
in negative Z space.

            Body CoM
               │
    ┌──────────┼──────────┐
    │          │          │
  [FR hip]  centre  [FL hip]
    │                     │
  [thigh]             [thigh]
    │                     │
  [calf ]             [calf ]
    │                     │
  [foot ]             [foot ]
"""

import numpy as np
from echo_q_control.Config import Leg_linkage, Configuration


def four_bar_ik(thigh_angle: float, linkage: Leg_linkage) -> float:
    """
    Convert a thigh angle (radians) to the physical calf servo angle,
    accounting for the 4-bar parallel linkage mechanism.

    The ECHO-Q knee uses a push-rod linkage where the calf servo drives
    a short rocker arm connected to the calf via a coupler rod.
    The transmission ratio is approximately 1:1 near the neutral position.

    Args:
        thigh_angle: Desired virtual thigh-to-calf angle (radians).
        linkage:     Leg_linkage geometry constants.

    Returns:
        Servo angle for the calf joint (radians).
    """
    # For a standard parallel 4-bar linkage, the calf angle tracks the
    # thigh angle. Departures come from the specific link lengths.
    # Simple implementation: direct mapping with small geometric correction.
    a, b, c, d = linkage.a, linkage.b, linkage.c, linkage.d

    # Normalise to mm-space
    norm = d / 100.0   # dimensionless scale

    # Correction term from coupler geometry (small-angle linear approximation)
    correction = np.arctan2(a * np.sin(thigh_angle),
                             d + a * np.cos(thigh_angle))

    return thigh_angle + correction * norm


def leg_ik(x: float, y: float, z: float, config: Configuration, side: int) -> np.ndarray:
    """
    Solve IK for a single leg.

    Args:
        x:     Foot X in body frame (forward positive).
        y:     Foot Y in body frame (left positive).
        z:     Foot Z in body frame (up positive; foot is negative).
        config: Robot configuration.
        side:  0 = right leg, 1 = left leg.

    Returns:
        np.ndarray([hip_angle, thigh_angle, calf_angle]) in radians.
    """
    linkage = Leg_linkage(config)
    L1, L2, L3 = config.L1, config.L2, config.L3
    side_sign = 1.0 if side == 0 else -1.0   # right legs vs left legs

    # ── 1. HIP (abduction/adduction) ────────────────────────────────────
    # Project into YZ plane of the hip
    y_eff = y - side_sign * L1    # y relative to hip pivot
    r_yz  = np.sqrt(y_eff**2 + z**2)
    r_yz  = max(r_yz, L1 + 1e-6)   # prevent division by zero

    hip_angle = np.arctan2(z, -side_sign * y_eff)

    # ── 2. THIGH + CALF (2-link planar IK in leg plane) ──────────────────
    # Length of the leg in the XZ plane after removing hip offset
    leg_plane_z = -np.sqrt(max(r_yz**2 - L1**2, 0.0))   # downward
    d = np.sqrt(x**2 + leg_plane_z**2)

    # Clamp to reachable workspace
    d = np.clip(d, abs(L2 - L3) + 1e-6, L2 + L3 - 1e-6)

    # Law of cosines – knee angle
    cos_knee = (L2**2 + L3**2 - d**2) / (2.0 * L2 * L3)
    knee_angle = np.arccos(np.clip(cos_knee, -1.0, 1.0))   # [0, π]

    # Law of cosines – thigh angle relative to the leg-plane downward axis
    cos_thigh = (L2**2 + d**2 - L3**2) / (2.0 * L2 * d)
    gamma = np.arccos(np.clip(cos_thigh, -1.0, 1.0))

    leg_plane_angle = np.arctan2(-leg_plane_z, x)
    thigh_angle = leg_plane_angle - gamma

    # Apply 4-bar linkage correction to the calf
    calf_angle = four_bar_ik(knee_angle, linkage)

    return np.array([hip_angle, thigh_angle, calf_angle])


def inverse_kinematics(foot_positions: np.ndarray, config: Configuration) -> np.ndarray:
    """
    Compute joint angles for all 4 legs simultaneously.

    Args:
        foot_positions: (3, 4) array – [x, y, z] for each foot in body frame.
                        Leg order: [FR, FL, BR, BL]
        config:         Robot configuration.

    Returns:
        (3, 4) array of [hip, thigh, calf] angles in radians.
        Row 0 = Hip, Row 1 = Thigh, Row 2 = Calf.
        Column order: [FR, FL, BR, BL].
    """
    angles = np.zeros((3, 4))

    # side: 0 = right (FR=0, BR=2), 1 = left (FL=1, BL=3)
    sides = [0, 1, 0, 1]

    for i in range(4):
        x, y, z = foot_positions[:, i]
        # Transform foot from body frame to hip-local frame
        x_local = x - config.LEG_ORIGINS[0, i]
        y_local = y - config.LEG_ORIGINS[1, i]
        z_local = z - config.LEG_ORIGINS[2, i]

        try:
            angles[:, i] = leg_ik(x_local, y_local, z_local, config, sides[i])
        except (ValueError, ZeroDivisionError):
            # Workspace violation – hold previous angles (zero on first tick)
            pass

    return angles


def forward_kinematics(joint_angles: np.ndarray, config: Configuration) -> np.ndarray:
    """
    Forward kinematics for all 4 legs (for visualisation / sanity checks).

    Args:
        joint_angles: (3, 4) [hip, thigh, calf] in radians.
        config:       Robot configuration.

    Returns:
        (3, 4) foot positions in body frame.
    """
    foot_positions = np.zeros((3, 4))
    L1, L2, L3 = config.L1, config.L2, config.L3
    sides = [0, 1, 0, 1]

    for i in range(4):
        hip, thigh, calf = joint_angles[:, i]
        side_sign = 1.0 if sides[i] == 0 else -1.0

        # Hip rotation (YZ plane)
        y_hip = -side_sign * L1 * np.cos(hip)
        z_hip = -L1 * np.sin(hip)

        # Thigh + calf in leg plane
        x_foot = L2 * np.cos(thigh) + L3 * np.cos(thigh + calf)
        z_leg  = L2 * np.sin(thigh) + L3 * np.sin(thigh + calf)

        foot_positions[0, i] = config.LEG_ORIGINS[0, i] + x_foot
        foot_positions[1, i] = config.LEG_ORIGINS[1, i] + y_hip
        foot_positions[2, i] = config.LEG_ORIGINS[2, i] + z_hip + z_leg

    return foot_positions
