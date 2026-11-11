"""Aperture calibration of the gripper bridge (no ROS graph needed)."""
import importlib.util
import sys
from pathlib import Path
from types import SimpleNamespace

import pytest

path = Path(__file__).parents[1] / 'scripts' / 'gripper_bridge.py'
sys.path.insert(0, str(path.parent))
spec = importlib.util.spec_from_file_location('gripper_bridge', path)
gb = importlib.util.module_from_spec(spec)
spec.loader.exec_module(gb)

# Shape of the 2026-11-06 robot finding: 6 cm commanded opened ~2 cm.
CAL = gb.calibration([0.0, 0.06, 0.12], [0.0, 0.06, 0.12], [0.0, 0.02, 0.09])


def test_interp_is_exact_on_points_and_extends_past_the_ends():
    xs, ys = [0.0, 0.06, 0.12], [0.0, 0.02, 0.09]
    assert gb.interp(0.06, xs, ys) == pytest.approx(0.02)
    assert gb.interp(0.09, xs, ys) == pytest.approx(0.055)
    assert gb.interp(-0.03, xs, ys) == pytest.approx(-0.01)   # squeeze stays negative
    assert gb.interp(0.15, xs, ys) == pytest.approx(0.125)


def test_calibrated_bridge_converts_both_ways():
    b = SimpleNamespace(cal=CAL)
    to_driver = lambda a: gb.GripperBridge.to_driver(b, a)
    to_physical = lambda r: gb.GripperBridge.to_physical(b, r)
    assert to_driver(0.02) == pytest.approx(0.06)
    assert to_physical(0.06) == pytest.approx(0.02)
    for a in (0.0, 0.01, 0.04, 0.08):
        assert to_physical(to_driver(a)) == pytest.approx(a)
    assert gb.GripperBridge.to_driver(SimpleNamespace(cal=None), 0.05) == 0.05


@pytest.mark.parametrize('rows', [([0.0], [0.0], [0.0]),
                                  ([0.0, 0.06], [0.0, 0.06], [0.02]),
                                  ([0.0, 0.06], [0.0, 0.06], [0.02, 0.01])])
def test_bad_tables_are_refused(rows):
    with pytest.raises(ValueError):
        gb.calibration(*rows)


def test_no_table_means_no_calibration():
    assert gb.calibration(None, None, None) is None


def test_robot_calibration_in_real_yaml_is_valid_and_matches_the_mission():
    import yaml
    cfg = yaml.safe_load(open(path.parents[1] / 'config' / 'real.yaml'))
    p = cfg['/gripper_bridge']['ros__parameters']
    cal = gb.calibration(p['calibration_command'], p['calibration_reported'],
                         p['calibration_measured'])
    assert cfg['/grasp_mission']['ros__parameters']['max_aperture'] == cal[2][-1]
    b = SimpleNamespace(cal=cal)
    # The mission's squeeze (-2 cm) must still command below the contact point.
    assert gb.GripperBridge.to_driver(b, -0.02) < cal[0][0]
    # The fully closed reading (driver 0.0017 m) reads as "nothing held".
    assert gb.GripperBridge.to_physical(b, 0.0017) < 0.01
