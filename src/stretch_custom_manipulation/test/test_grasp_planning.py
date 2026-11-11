import importlib.util
import math
from pathlib import Path

import numpy as np

path = Path(__file__).parents[1] / 'scripts' / 'grasp_planning.py'
spec = importlib.util.spec_from_file_location('grasp_planning', path)
gp = importlib.util.module_from_spec(spec)
spec.loader.exec_module(gp)


def can(cx, cy, z0=1.0, r=0.033, h=0.12):
    """Front half of a vertical cylinder as the head camera sees it."""
    a = np.linspace(0, np.pi, 30)  # side facing the robot (+y)
    z = np.linspace(z0, z0 + h, 15)
    aa, zz = np.meshgrid(a, z)
    return np.column_stack([cx + r * np.cos(aa).ravel(),
                            cy + r * np.sin(aa).ravel(), zz.ravel()])


def board(z, y0=-0.85, y1=-0.55, x0=-0.3, x1=0.5):
    xs, ys = np.meshgrid(np.linspace(x0, x1, 60), np.linspace(y0, y1, 20))
    return np.column_stack([xs.ravel(), ys.ravel(), np.full(xs.size, z)])


def test_free_can_side_grasp_adaptive_aperture():
    obj = can(0.1, -0.65)
    scene = np.vstack([obj, board(0.995)])
    best = gp.plan_grasps(obj, scene)[0]
    assert best['mode'] == 'side'
    g = best['grasp']
    np.testing.assert_allclose(g[:3, 0], [0, -1, 0], atol=1e-9)   # approach -y
    assert abs(g[0, 3] - 0.1) < 0.005 and abs(g[2, 3] - 1.06) < 0.005
    # Width 66 mm: opening is width + margins, not the full 13 cm.
    assert 0.066 < best['aperture'] < 0.1
    assert best['pregrasp'][1, 3] > g[1, 3]  # pre-grasp is closer to the robot


def test_neighbor_limits_aperture():
    obj = can(0.1, -0.65)
    neighbor = can(0.1 + 0.09, -0.65)  # 24 mm gap between cans
    scene = np.vstack([obj, neighbor, board(0.995)])
    best = gp.plan_grasps(obj, scene)[0]
    gap_limit = 2 * (0.09 - 0.033 - gp.FINGER_THICKNESS)
    assert best['aperture'] <= gap_limit + 1e-9
    assert best['aperture'] >= best['width'] + 2 * gp.MIN_MARGIN


def test_too_wide_object_is_rejected():
    xs, zs = np.meshgrid(np.linspace(-0.1, 0.1, 30), np.linspace(1.0, 1.1, 10))
    box = np.column_stack([xs.ravel(), np.full(xs.size, -0.6), zs.ravel()])
    assert not [c for c in gp.plan_grasps(box, box) if c['mode'] == 'side']


def test_shelf_above_blocks_top_grasp_and_body_corridor_blocks_side():
    obj = can(0.1, -0.65)
    above = board(1.14)  # 2 cm above the can top
    cands = gp.plan_grasps(obj, np.vstack([obj, above]))
    assert cands and all(c['mode'] == 'side' for c in cands)
    wall = np.column_stack([np.full(50, 0.1), np.full(50, -0.5),
                            np.linspace(1.0, 1.12, 50)])
    blocked = gp.plan_grasps(obj, np.vstack([obj, wall]))
    assert not any(np.allclose(c['grasp'][:3, 0], [0, -1, 0], atol=1e-6)
                   for c in blocked)


def test_quaternion_round_trip():
    r = gp.frame([0, -1, 0], [1, 0, 0])
    x, y, z, w = gp.quaternion(r)
    q = np.array([x, y, z, w])
    assert abs(np.linalg.norm(q) - 1) < 1e-9
    m = np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])
    np.testing.assert_allclose(m, r[:3, :3], atol=1e-9)


def test_transform_and_mask_points():
    from types import SimpleNamespace as N
    r = gp.frame([0, -1, 0], [1, 0, 0])
    x, y, z, w = gp.quaternion(r)
    t = N(rotation=N(x=x, y=y, z=z, w=w), translation=N(x=1.0, y=2.0, z=3.0))
    m = gp.matrix_from_transform(t)
    np.testing.assert_allclose(m[:3, :3], r[:3, :3], atol=1e-9)
    np.testing.assert_allclose(gp.apply(m, np.zeros((1, 3)))[0], [1, 2, 3])
    mask = np.zeros((20, 20), np.uint8)
    mask[5:15, 5:15] = 255
    depth = np.full((20, 20), 800, np.uint16)
    pts = gp.mask_points(mask, depth, 300.0, 300.0, 10.0, 10.0)
    assert len(pts) == 36  # 10x10 eroded by 2 px per side
    np.testing.assert_allclose(pts[:, 2], 0.8)


def test_extra_opening_adds_to_free_can_but_not_past_neighbors():
    obj = can(0.1, -0.65)
    scene = np.vstack([obj, board(0.995)])
    base = gp.plan_grasps(obj, scene)[0]
    wide = gp.plan_grasps(obj, scene, extra_opening=0.04)[0]
    assert abs(wide['aperture'] - min(base['aperture'] + 0.04, gp.MAX_APERTURE)) < 1e-9
    neighbor = can(0.1 + 0.09, -0.65)
    tight = gp.plan_grasps(obj, np.vstack([obj, neighbor, board(0.995)]), extra_opening=0.04)[0]
    assert tight['aperture'] <= 2 * (0.09 - 0.033 - gp.FINGER_THICKNESS) + 1e-9


def test_short_object_side_grasp_clears_the_shelf():
    obj = can(0.1, -0.65, z0=1.0, h=0.074)
    # Shelf under the whole approach: the level grasp's gripper body would hit
    # it; the downward-tilted side grasp fits.
    side = [c for c in gp.plan_grasps(obj, np.vstack([obj, board(0.995)])) if c['mode'] == 'side']
    assert side and side[0]['key'] == ('side', 0.0, -0.25)
    # Level grasp (object on an edge): fingers raised to clear the surface.
    level = [c for c in gp.plan_grasps(obj, obj) if c['key'] == ('side', 0.0, 0.0)][0]
    z = level['grasp'][2, 3]
    assert z >= 1.0 + gp.FINGER_HALF_HEIGHT + 0.015 - 1e-9 and z <= 1.074 - 0.01


def test_tall_object_side_grasp_stays_at_mid_height():
    obj = can(0.1, -0.65, z0=1.0, h=0.17)
    side = [c for c in gp.plan_grasps(obj, np.vstack([obj, board(0.995)])) if c['mode'] == 'side']
    assert abs(side[0]['grasp'][2, 3] - 1.085) < 0.005


def test_tilted_side_grasp_reaches_as_deep_as_level_on_tall_object():
    # Front face + top of a 2.5 x 3.3 x 13 cm box (the 2026-11-03 15:39 run).
    xs, zs = np.meshgrid(np.linspace(-0.057, -0.032, 8), np.linspace(0.983, 1.112, 30))
    front = np.column_stack([xs.ravel(), np.full(xs.size, -0.648), zs.ravel()])
    xt, yt = np.meshgrid(np.linspace(-0.057, -0.032, 8), np.linspace(-0.681, -0.648, 10))
    obj = np.vstack([front, np.column_stack([xt.ravel(), yt.ravel(), np.full(xt.size, 1.112)])])
    by_key = {c['key']: c['grasp'][:3, 3] for c in gp.plan_grasps(obj, np.zeros((0, 3)))}
    level, tilted = by_key[('side', 0.0, 0.0)], by_key[('side', 0.0, -0.25)]
    np.testing.assert_allclose(tilted, level, atol=1e-9)
    # Fingertips 4 cm past the near face (-0.648), grasp center FINGERTIP_BEHIND further.
    assert abs(level[1] - (-0.648 - 0.04 - gp.FINGERTIP_BEHIND)) < 0.005


def test_real_shelf_scene_from_robot_gets_a_side_grasp():
    # 2026-11-03 16:05 on the robot: shelf-surface noise 1-2 cm above the shelf
    # behind the object blocked every side grasp.
    d = np.load(Path(__file__).parent / 'data' / 'robot_scene_20261103_160513.npz')
    cands = gp.plan_grasps(d['object'], d['scene'], extra_opening=gp.MAX_APERTURE)
    assert cands and cands[0]['key'] == ('side', 0.0, 0.0)
    lo, hi = d['object'].min(0), d['object'].max(0)
    assert lo[2] + gp.FINGER_HALF_HEIGHT + 0.015 - 1e-9 <= cands[0]['grasp'][2, 3] < hi[2]


def box(cx, near_y, w=0.06, d=0.03, z0=1.0, h=0.12, yaw=0.0):
    """Front face and top of a box as the head camera sees it, turned by yaw."""
    xs, zs = np.meshgrid(np.linspace(-w / 2, w / 2, 15), np.linspace(z0, z0 + h, 20))
    front = np.column_stack([xs.ravel(), np.zeros(xs.size), zs.ravel()])
    xt, yt = np.meshgrid(np.linspace(-w / 2, w / 2, 15), np.linspace(-d, 0, 8))
    top = np.column_stack([xt.ravel(), yt.ravel(), np.full(xt.size, z0 + h)])
    pts = np.vstack([front, top])
    c, s = math.cos(yaw), math.sin(yaw)
    pts[:, :2] = pts[:, :2] @ np.array([[c, s], [-s, c]])
    return pts + [cx, near_y, 0.0]


def test_fingertips_reach_insert_past_the_near_face():
    # 2026-11-06 on the robot: the center was 3 cm in, the tips (1.7 cm behind
    # it) only touched the front edge and the fingers closed on nothing.
    for insert in (0.03, 0.04, 0.05):
        g = [c for c in gp.plan_grasps(box(0.0, -0.55), np.zeros((0, 3)), insert=insert)
             if c['key'] == ('side', 0.0, 0.0)][0]['grasp']
        tips_y = g[1, 3] + gp.FINGERTIP_BEHIND
        assert abs(tips_y - (-0.55 - insert)) < 1e-6


def test_wrist_turns_to_a_rotated_box_but_approach_stays_straight():
    for deg in (-15.0, 0.0, 12.0):
        obj = box(0.0, -0.55, yaw=math.radians(deg))
        assert abs(math.degrees(gp.object_yaw(obj)) - deg) <= 1.0
        c = [c for c in gp.plan_grasps(obj, np.zeros((0, 3))) if c['key'] == ('side', 0.0, 0.0)][0]
        fingers = c['grasp'][:3, 1]
        assert abs(math.degrees(math.atan2(fingers[1], fingers[0])) - deg) <= 1.0
        # Pre-grasp to grasp is a pure move along the arm: same x, same wrist yaw.
        assert abs(c['pregrasp'][0, 3] - c['grasp'][0, 3]) < 1e-9
        np.testing.assert_allclose(c['pregrasp'][:3, :3], c['grasp'][:3, :3])


def test_round_objects_are_not_turned():
    assert gp.object_yaw(can(0.0, -0.6)) == 0.0
