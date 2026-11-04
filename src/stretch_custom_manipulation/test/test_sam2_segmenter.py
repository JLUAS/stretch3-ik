import importlib.util
from pathlib import Path

import numpy as np

path = Path(__file__).parents[1] / 'scripts' / 'sam2_segmenter.py'
spec = importlib.util.spec_from_file_location('sam2_segmenter', path)
seg = importlib.util.module_from_spec(spec)
spec.loader.exec_module(seg)


def masks():
    logo = np.zeros((240, 424), bool)
    logo[70:80, 290:300] = True                     # part: the logo
    can = np.zeros((240, 424), bool)
    can[60:90, 260:330] = True                      # whole can
    wall = np.zeros((240, 424), bool)
    wall[:, 200:] = True                            # half the image
    return np.stack([logo, can, wall])


def test_whole_object_beats_higher_scoring_part():
    m = seg.pick_mask(masks(), np.array([0.99, 0.95, 0.99]), (295, 75))
    assert m.sum() == 30 * 70


def test_low_scoring_large_mask_is_not_chosen():
    m = seg.pick_mask(masks(), np.array([0.99, 0.6, 0.5]), (295, 75))
    assert m.sum() == 100


def test_click_outside_every_mask():
    assert seg.pick_mask(masks(), np.array([0.9, 0.9, 0.9]), (10, 10)) is None
