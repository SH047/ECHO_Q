import numpy as np, pytest
from echo_q_utilities.Utilities import deadband, clipped_first_order_filter, smooth_step, wrap_angle, swing_trajectory, euler_to_rotation_matrix

def test_deadband_inside():   assert deadband(0.1, 0.25) == 0.0
def test_deadband_outside():  assert abs(deadband(0.5, 0.25) - 0.25) < 1e-9
def test_filter_clamps():     assert clipped_first_order_filter(0,1000,2.0,0.5) == pytest.approx(2.0)
def test_filter_zero():       assert clipped_first_order_filter(1,1,5,0.5) == pytest.approx(0.0)
def test_smoothstep_ends():   assert smooth_step(0.0) == 0.0; assert smooth_step(1.0) == 1.0
def test_wrap_angle():        assert abs(wrap_angle(2*np.pi)) < 1e-9
def test_rotation_orthogonal():
    R = euler_to_rotation_matrix(0.1, 0.2, 0.3)
    np.testing.assert_allclose(R @ R.T, np.eye(3), atol=1e-12)
def test_swing_start():
    s = np.array([0.1,0,-0.15]); e = np.array([0,0,-0.15])
    np.testing.assert_allclose(swing_trajectory(0,s,e,0.06)[:2], s[:2], atol=1e-9)
def test_swing_peak():
    s = e = np.array([0,0,-0.15])
    assert swing_trajectory(0.5,s,e,0.06)[2] == pytest.approx(-0.15+0.06, abs=1e-9)
