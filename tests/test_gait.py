import numpy as np, pytest
from echo_q_control.Config import Configuration
from echo_q_control.Gait   import GaitController

@pytest.fixture
def cfg(): return Configuration()
@pytest.fixture
def gait(cfg): return GaitController(cfg)

def test_initial_phase(gait):    assert gait.phase == 0
def test_reset(cfg, gait):
    for i in range(20): gait.update(i*cfg.dt, np.array([0.3,0]), cfg.default_stance)
    gait.reset(); assert gait.phase == 0
def test_shapes(cfg, gait):
    fp, c = gait.update(0, np.zeros(2), cfg.default_stance)
    assert fp.shape == (3,4) and c.shape == (4,)
def test_contacts_bool(cfg, gait):
    _, c = gait.update(0, np.zeros(2), cfg.default_stance)
    assert c.dtype == bool
def test_min_two_contacts(cfg, gait):
    v = np.array([0.4, 0])
    for i in range(cfg.phase_length * 2):
        _, c = gait.update(i*cfg.dt, v, cfg.default_stance)
        assert c.sum() >= 2
def test_swing_lifts(cfg, gait):
    v = np.array([0.3, 0]); lifted = False
    for i in range(cfg.phase_length * 3):
        fp, c = gait.update(i*cfg.dt, v, cfg.default_stance)
        for leg in range(4):
            if not c[leg] and fp[2,leg] > cfg.default_z_ref + 0.005: lifted = True
    assert lifted
def test_no_nan(cfg, gait):
    for i in range(cfg.phase_length * 4):
        fp, _ = gait.update(i*cfg.dt, np.array([0.5,0.1]), cfg.default_stance)
        assert np.all(np.isfinite(fp))
