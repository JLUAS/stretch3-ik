"""Pixel/mask -> aligned depth -> optical frame -> base_link -> grasp, against
a ray-cast scene with known geometry; gripper self-filter bias; segmenter
mask selection."""
import importlib.util
import math
import sys
from pathlib import Path

import numpy as np

scripts = Path(__file__).parents[1] / 'scripts'
sys.path.insert(0, str(scripts))


def load(name):
    spec = importlib.util.spec_from_file_location(name, scripts / f'{name}.py')
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


gp = load('grasp_planning')

# Head D435i color intrinsics measured on the robot (424x240, no distortion).
FX, FY, CX, CY, W, H = 303.06, 303.07, 212.06, 122.79, 424, 240
CAN = {'c': np.array([-0.03, -0.62, 0.80]), 'r': 0.033, 'h': 0.12}


def camera_pose(cam=(0.0, -0.05, 1.30), target=CAN['c']):
    """base_link <- camera_color_optical_frame: z forward, x right, y down."""
    z = np.asarray(target) - cam
    z /= np.linalg.norm(z)
    x = np.cross(z, [0, 0, 1.0])
    x /= np.linalg.norm(x)
    m = np.eye(4)
    m[:3, :3] = np.column_stack([x, np.cross(z, x), z])
    m[:3, 3] = cam
    return m


def render(pose, can=CAN):
    """Ray-cast 16UC1 depth (mm) of a can standing on a table; mask of the can."""
    c, r, h = can['c'], can['r'], can['h']
    u, v = np.meshgrid(np.arange(W), np.arange(H))
    rays = np.stack([(u - CX) / FX, (v - CY) / FY, np.ones_like(u, float)], -1)  # z = 1
    d = rays @ pose[:3, :3].T
    o = pose[:3, 3]
    t_hit = np.full((H, W), np.inf)
    # Side of the cylinder.
    a = d[..., 0] ** 2 + d[..., 1] ** 2
    b = 2 * (d[..., 0] * (o[0] - c[0]) + d[..., 1] * (o[1] - c[1]))
    k = (o[0] - c[0]) ** 2 + (o[1] - c[1]) ** 2 - r * r
    disc = b * b - 4 * a * k
    t = (-b - np.sqrt(np.maximum(disc, 0))) / (2 * a)
    z = o[2] + t * d[..., 2]
    side = (disc > 0) & (t > 0) & (np.abs(z - c[2]) <= h / 2)
    t_hit[side] = t[side]
    # Top disc.
    tt = (c[2] + h / 2 - o[2]) / d[..., 2]
    p = o + tt[..., None] * d
    top = (tt > 0) & ((p[..., 0] - c[0]) ** 2 + (p[..., 1] - c[1]) ** 2 <= r * r) & (tt < t_hit)
    t_hit[top] = tt[top]
    mask = np.isfinite(t_hit)
    # Table under the can.
    tb = (c[2] - h / 2 - o[2]) / d[..., 2]
    t_all = np.where(mask, t_hit, np.where(tb > 0, tb, 0))
    depth = np.round(t_all * 1000).astype(np.uint16)  # t is the optical z (rays have z = 1)
    return depth, mask


def radial_error(points, can=CAN):
    c, r, h = can['c'], can['r'], can['h']
    rad = np.hypot(points[:, 0] - c[0], points[:, 1] - c[1])
    on_top = np.abs(points[:, 2] - (c[2] + h / 2)) < 0.002
    return np.where(on_top, np.maximum(rad - r, 0), np.abs(rad - r))


def test_depth_scale_intrinsics_axes_and_tf_direction():
    pose = camera_pose()
    depth, mask = render(pose)
    assert mask.sum() > 300
    pts = gp.mask_points(mask, depth, FX, FY, CX, CY)
    base = gp.apply(pose, pts)
    err = radial_error(base)
    # Millimeter quantization and the eroded edge only: every point on the can.
    assert np.percentile(err, 95) < 0.002 and err.max() < 0.005
    # Units: the same image read as meters would put the can 1000x farther.
    wrong = gp.apply(pose, gp.mask_points(mask, depth.astype(float) * 1000, FX, FY, CX, CY,
                                          z_range=(0.15, 1e9)))
    assert np.linalg.norm(wrong.mean(0) - CAN['c']) > 10
    # Inverse transform direction is caught: points land far from the can.
    assert np.median(radial_error(gp.apply(np.linalg.inv(pose), pts))) > 0.1


def test_matrix_from_transform_maps_source_points_into_target():
    class T:
        pass
    t = T()
    t.translation = T()
    t.rotation = T()
    t.translation.x, t.translation.y, t.translation.z = 1.0, 2.0, 3.0
    yaw = math.pi / 2
    t.rotation.x, t.rotation.y, t.rotation.z, t.rotation.w = 0, 0, math.sin(yaw / 2), math.cos(yaw / 2)
    np.testing.assert_allclose(gp.apply(gp.matrix_from_transform(t), np.array([[1.0, 0, 0]])),
                               [[1.0, 3.0, 3.0]], atol=1e-12)


def test_grasp_from_rendered_view_hits_the_can_axis():
    """Residual budget of perception -> grasp with a perfect camera model."""
    pose = camera_pose()
    depth, mask = render(pose)
    obj = gp.apply(pose, gp.mask_points(mask, depth, FX, FY, CX, CY))
    best = gp.plan_grasps(obj, np.zeros((0, 3)))[0]
    g = best['grasp'][:3, 3]
    lateral = abs(g[0] - CAN['c'][0])          # along the finger axis
    along = abs(g[1] - CAN['c'][1])            # along the approach
    vertical = abs(g[2] - CAN['c'][2])
    print(f'grasp residual: lateral {lateral * 1000:.1f} mm, approach {along * 1000:.1f} mm, '
          f'vertical {vertical * 1000:.1f} mm, width {best["width"] * 1000:.1f} mm')
    assert lateral < 0.003 and along < 0.008 and vertical < 0.01
    # The eroded mask and grazing edges see the can ~11 mm narrower than it
    # is; the opening must still clear the true diameter.
    assert -0.015 < best['width'] - 2 * CAN['r'] < 0
    assert best['aperture'] >= 2 * CAN['r'] + 2 * gp.MIN_MARGIN


def test_gripper_envelope_filter_keeps_the_object_at_pregrasp():
    """The old filter dropped points within 10 cm of link_grasp_center; at
    pre-grasp (10 cm back from the can axis) that removed the near face."""
    pose = camera_pose()
    depth, mask = render(pose)
    obj = gp.apply(pose, gp.mask_points(mask, depth, FX, FY, CX, CY))
    best = gp.plan_grasps(obj, np.zeros((0, 3)))[0]
    tcp = best['pregrasp']
    old = obj[np.linalg.norm(obj - tcp[:3, 3], axis=1) > 0.10]
    new = gp.outside_gripper(obj, tcp)
    old_shift = np.linalg.norm(old.mean(0) - obj.mean(0))
    print(f'points kept: old {len(old)}/{len(obj)}, new {len(new)}/{len(obj)}; '
          f'centroid shift old {old_shift * 1000:.1f} mm')
    assert len(new) == len(obj)
    assert len(old) < 0.8 * len(obj) and old_shift > 0.005
    # Gripper points (fingers, body) are still removed.
    fingers = gp.apply(tcp, np.array([[-0.015, 0.05, 0.0], [-0.015, -0.05, 0.0], [-0.2, 0, 0.0]]))
    assert len(gp.outside_gripper(fingers, tcp)) == 0


def test_rejects_unfit_orientations_and_openings():
    pose = camera_pose()
    depth, mask = render(pose)
    obj = gp.apply(pose, gp.mask_points(mask, depth, FX, FY, CX, CY))
    best = gp.plan_grasps(obj, np.zeros((0, 3)))[0]
    assert gp.fits(obj, np.zeros((0, 3)), best['grasp']) is not None
    # A shelf divider 6.5 cm beside the can axis: the planned straight
    # approach fits; the same grasp with the wrist turned 0.1 rad (inside the
    # IK's 0.12 rad approach tolerance) sweeps a finger into it.
    wall = np.array([[CAN['c'][0] + 0.065, y, z] for y in np.arange(-0.75, -0.45, 0.005)
                     for z in np.arange(0.76, 0.85, 0.01)])
    turned = best['grasp'].copy()
    c, s = math.cos(-0.1), math.sin(-0.1)
    turned[:3, :3] = np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]]) @ turned[:3, :3]
    assert gp.fits(obj, wall, best['grasp']) is not None
    assert gp.fits(obj, wall, turned) is None
    # Wider than the gripper opens in every horizontal direction.
    c0 = obj.mean(0)
    assert not gp.plan_grasps(c0 + (obj - c0) * [3, 3, 1], np.zeros((0, 3)))
    # Tabletop: a top-down grasp is still offered, a side grasp below the
    # table surface is not.
    table = np.array([[x, y, CAN['c'][2] - 0.01] for x in np.arange(-0.25, 0.2, 0.01)
                      for y in np.arange(-0.8, -0.45, 0.01)])
    low = obj - [0, 0, 0.05]
    assert all(c['mode'] != 'side' for c in gp.plan_grasps(low, table))


def test_clicked_geometry_mask_keeps_the_downstream_grasp_contract():
    from rgbd_perception.conversions import select_mask
    pose = camera_pose()
    depth, truth = render(pose)
    v, u = np.argwhere(truth)[len(np.argwhere(truth)) // 2]
    mask, reason = select_mask(np.where(truth, 0, -1), (u, v))
    assert not reason
    assert select_mask(np.where(truth, 0, -1), (0, 0))[0] is None
    obj = gp.apply(pose, gp.mask_points(mask, depth, FX, FY, CX, CY))
    best = gp.plan_grasps(obj, np.zeros((0, 3)))[0]
    assert abs(best['grasp'][0, 3] - CAN['c'][0]) < 0.005
