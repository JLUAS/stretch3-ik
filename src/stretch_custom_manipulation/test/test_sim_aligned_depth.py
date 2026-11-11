import importlib.util
from pathlib import Path

import numpy as np

path = Path(__file__).resolve().parents[1] / 'scripts' / 'sim_aligned_depth.py'
spec = importlib.util.spec_from_file_location('sim_aligned_depth', path)
module = importlib.util.module_from_spec(spec)
spec.loader.exec_module(module)

K = np.array([[200., 0, 50], [0, 200., 40], [0, 0, 1]])


def test_identity_keeps_depth_in_mm():
    d = np.full((80, 100), 0.75, np.float32)
    out = module.register(d, K, K, np.eye(4), (80, 100))
    assert out.dtype == np.uint16 and (out == 750).all()


def test_baseline_shifts_pixels_and_leaves_holes_empty():
    d = np.full((80, 100), 1.0, np.float32)
    t = np.eye(4)
    t[0, 3] = 0.05                      # color camera 5 cm to the -x of depth
    out = module.register(d, K, K, t, (80, 100))
    # u' = u + fx * 0.05 / 1.0 = u + 10 px: the first 10 columns get no data.
    assert (out[:, :10] == 0).all() and (out[:, 10:] == 1000).all()


def test_invalid_depth_is_dropped():
    d = np.full((80, 100), np.nan, np.float32)
    d[40, 50] = 0.5
    out = module.register(d, K, K, np.eye(4), (80, 100))
    assert out[40, 50] == 500 and (out > 0).sum() == 1
