#!/usr/bin/env python3
"""Phase-based diagonal trot gait controller."""
import numpy as np
from echo_q_utilities.Utilities import swing_trajectory

class GaitController:
    def __init__(self, config):
        self.config = config
        self.phase  = 0
        self.tick   = 0
        self._swing_start = config.default_stance.copy()

    def update(self, current_time, command_velocity, default_stance):
        self.tick += 1
        if self.tick >= int(self.config.phase_ticks[self.phase]):
            self.tick  = 0
            self.phase = (self.phase + 1) % self.config.num_phases

        contacts       = self.config.contact_phases[self.phase].astype(bool)
        foot_positions = default_stance.copy()
        swing_phase    = np.clip(self.tick / max(self.config.swing_ticks, 1), 0.0, 1.0)

        for leg in range(4):
            if not contacts[leg]:
                step    = command_velocity * self.config.swing_time * 0.5
                landing = default_stance[:, leg].copy()
                landing[0] += step[0]; landing[1] += step[1]
                foot_positions[:, leg] = swing_trajectory(
                    swing_phase, self._swing_start[:, leg], landing,
                    self.config.z_clearance)
            else:
                self._swing_start[:, leg] = foot_positions[:, leg]

        return foot_positions, contacts

    def reset(self):
        self.phase = 0; self.tick = 0
        self._swing_start = self.config.default_stance.copy()
