#!/usr/bin/env python3
"""
ECHO-Q Gait Controller
=======================
Implements a diagonal trot gait using a phase-based contact schedule.

Theory
------
The trot is divided into 4 phases cycling at `phase_length` ticks.
Each phase specifies which feet are in STANCE (contact) and which are in
SWING (airborne).  The diagonal pairs (FR+BL) and (FL+BR) alternate.

Swing trajectory is a sinusoidal lift computed in Utilities.swing_trajectory().
"""

import numpy as np
from echo_q_utilities.Utilities import swing_trajectory


class GaitController:
    """
    Phase-based trot gait generator.

    Call `update()` every control tick; it returns the desired foot
    positions and the contact mask for the current phase.
    """

    def __init__(self, config):
        self.config  = config
        self.phase   = 0          # Current phase index (0-3)
        self.tick    = 0          # Ticks elapsed in current phase
        self.time    = 0.0        # Absolute simulation time (s)

        # Previous foot positions – used as swing start points
        self._swing_start = config.default_stance.copy()

    # ── Public API ────────────────────────────────────────────────────────────

    def update(self, current_time: float, command_velocity: np.ndarray,
               default_stance: np.ndarray) -> tuple:
        """
        Advance the gait by one control tick.

        Args:
            current_time:     Current ROS time in seconds.
            command_velocity: (2,) [vx, vy] body-frame velocity command (m/s).
            default_stance:   (3, 4) nominal foot positions.

        Returns:
            foot_positions: (3, 4) desired foot XYZ in body frame.
            contacts:       (4,)  bool array – True if foot should be in stance.
        """
        dt = self.config.dt

        # ── Advance phase counter ─────────────────────────────────────────
        self.tick += 1
        self.time  = current_time

        phase_duration = int(self.config.phase_ticks[self.phase])
        if self.tick >= phase_duration:
            self.tick  = 0
            self.phase = (self.phase + 1) % self.config.num_phases

        # ── Contact mask for this phase ───────────────────────────────────
        contacts = self.config.contact_phases[self.phase].astype(bool)   # (4,)

        # ── Compute foot positions ────────────────────────────────────────
        foot_positions = default_stance.copy()
        swing_phase    = self.tick / max(self.config.swing_ticks, 1)
        swing_phase    = np.clip(swing_phase, 0.0, 1.0)

        for leg_idx in range(4):
            if not contacts[leg_idx]:
                # ── SWING foot: lift toward next stance point ─────────────
                # Predict where the foot should land based on velocity
                step_distance = command_velocity * self.config.swing_time * 0.5
                landing = default_stance[:, leg_idx].copy()
                landing[0] += step_distance[0]
                landing[1] += step_distance[1]

                pos = swing_trajectory(
                    phase     = swing_phase,
                    start     = self._swing_start[:, leg_idx],
                    end       = landing,
                    clearance = self.config.z_clearance,
                )
                foot_positions[:, leg_idx] = pos
            else:
                # ── STANCE foot: record position as potential swing start ──
                self._swing_start[:, leg_idx] = foot_positions[:, leg_idx]

        return foot_positions, contacts

    def reset(self):
        """Reset the gait cycle to phase 0."""
        self.phase = 0
        self.tick  = 0
        self._swing_start = self.config.default_stance.copy()
