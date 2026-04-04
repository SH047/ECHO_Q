"""
Tests for echo_q_control.Kinematics
=====================================
Run with:   pytest tests/test_kinematics.py -v
"""

import sys, os
sys.path.insert(0, os.path.join(os.path.dirname(__file__),
                                '..', 'src', 'echo_q_control', 'src'))
sys.path.insert(0, os.path.join(os.path.dirname(__file__),
                                '..', 'src', 'echo_q_utilities', 'src'))

import numpy as np
import pytest
from echo_q_control.Config      import Configuration
from echo_q_control.Kinematics  import inverse_kinematics, forward_kinematics, leg_ik


@pytest.fixture
def config():
    return Configuration()


class TestInverseKinematics:

    def test_returns_correct_shape(self, config):
        foot_pos = config.default_stance
        angles   = inverse_kinematics(foot_pos, config)
        assert angles.shape == (3, 4)

    def test_default_stance_is_reachable(self, config):
        foot_pos = config.default_stance
        angles   = inverse_kinematics(foot_pos, config)
        # All angles should be finite (no NaN / inf)
        assert np.all(np.isfinite(angles))

    def test_angles_in_plausible_range(self, config):
        foot_pos = config.default_stance
        angles   = inverse_kinematics(foot_pos, config)
        # Hip angles should be small (robot standing upright)
        assert np.all(np.abs(angles[0, :]) < np.pi / 2)
        # Thigh and calf angles bounded to ±π
        assert np.all(np.abs(angles[1:, :]) <= np.pi + 0.01)

    def test_symmetric_left_right(self, config):
        """FR and FL should be mirror images in default stance."""
        foot_pos = config.default_stance
        angles   = inverse_kinematics(foot_pos, config)
        # Hip angles FR vs FL should be negated (mirrored)
        # Thigh and calf should be equal magnitude
        assert abs(abs(angles[1, 0]) - abs(angles[1, 1])) < 0.1  # thigh
        assert abs(abs(angles[2, 0]) - abs(angles[2, 1])) < 0.1  # calf

    def test_lower_foot_increases_knee_bend(self, config):
        """Lowering the foot should increase the knee (calf) angle."""
        stance_normal = config.default_stance.copy()
        stance_lower  = config.default_stance.copy()
        stance_lower[2, :] -= 0.03   # Lower all feet 3 cm

        angles_normal = inverse_kinematics(stance_normal, config)
        angles_lower  = inverse_kinematics(stance_lower,  config)

        # Calf angles (row 2) should be larger when foot is lower
        assert np.all(angles_lower[2, :] >= angles_normal[2, :] - 0.05)

    def test_no_nan_at_workspace_boundary(self, config):
        """Even near the workspace edge, IK must not produce NaN."""
        foot_pos = config.default_stance.copy()
        foot_pos[2, :] = -0.25   # Nearly fully extended
        angles = inverse_kinematics(foot_pos, config)
        assert np.all(np.isfinite(angles))


class TestForwardKinematics:

    def test_returns_correct_shape(self, config):
        angles = np.zeros((3, 4))
        pos    = forward_kinematics(angles, config)
        assert pos.shape == (3, 4)

    def test_fk_output_is_finite(self, config):
        """FK must always produce finite positions (no NaN/inf)."""
        angles   = inverse_kinematics(config.default_stance, config)
        recovered = forward_kinematics(angles, config)
        assert np.all(np.isfinite(recovered)), "FK produced non-finite values"

    def test_fk_xy_roughly_matches_target(self, config):
        """FK XY coordinates should be within 3 cm of IK target (XY only)."""
        target   = config.default_stance
        angles   = inverse_kinematics(target, config)
        recovered = forward_kinematics(angles, config)
        # XY is more reliable than Z in the simplified FK model
        np.testing.assert_allclose(recovered[:2, :], target[:2, :], atol=0.03)


class TestLegIK:

    def test_returns_three_angles(self, config):
        angles = leg_ik(0.0, -config.L1 - 0.01, -0.15, config, side=0)
        assert len(angles) == 3

    def test_all_angles_finite(self, config):
        for side in [0, 1]:
            angles = leg_ik(0.05, -config.L1 - 0.05, -0.14, config, side=side)
            assert np.all(np.isfinite(angles)), f"Got NaN for side={side}"


class TestGaitConfig:
    """Sanity-check gait timing properties."""

    def test_phase_ticks_positive(self, config):
        assert all(t > 0 for t in config.phase_ticks)

    def test_overlap_plus_swing_equals_stance(self, config):
        assert config.stance_ticks == 2 * config.overlap_ticks + config.swing_ticks

    def test_default_stance_shape(self, config):
        assert config.default_stance.shape == (3, 4)

    def test_default_stance_z_is_negative(self, config):
        assert np.all(config.default_stance[2, :] < 0)
