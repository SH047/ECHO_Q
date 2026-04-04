import numpy as np, pytest
from echo_q_control.Config     import Configuration
from echo_q_control.Kinematics import inverse_kinematics, forward_kinematics

@pytest.fixture
def cfg(): return Configuration()

def test_ik_shape(cfg):         assert inverse_kinematics(cfg.default_stance, cfg).shape == (3,4)
def test_ik_finite(cfg):        assert np.all(np.isfinite(inverse_kinematics(cfg.default_stance, cfg)))
def test_ik_angles_bounded(cfg):
    a = inverse_kinematics(cfg.default_stance, cfg)
    assert np.all(np.abs(a) <= np.pi + 0.01)
def test_ik_workspace_boundary(cfg):
    fp = cfg.default_stance.copy(); fp[2,:] = -0.25
    assert np.all(np.isfinite(inverse_kinematics(fp, cfg)))
def test_fk_shape(cfg):         assert forward_kinematics(np.zeros((3,4)), cfg).shape == (3,4)
def test_fk_finite(cfg):        assert np.all(np.isfinite(forward_kinematics(np.zeros((3,4)), cfg)))
def test_stance_z_negative(cfg): assert np.all(cfg.default_stance[2,:] < 0)
def test_overlap_ticks(cfg):    assert cfg.overlap_ticks > 0
def test_swing_ticks(cfg):      assert cfg.swing_ticks > 0
