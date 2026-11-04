#!/usr/bin/env python3
"""Grasp candidates for shelf objects from point clouds (numpy only, no ROS).

All inputs and outputs are in base_link: x forward, z up, the arm reaches
toward -y. Poses use the link_grasp_center convention: X = approach,
Y = finger closing axis, Z = X x Y.
"""
import math

import numpy as np

MAX_APERTURE = 0.13       # SG3 opening used for planning
APERTURE_MARGIN = 0.015   # free space per side between finger and object
MIN_MARGIN = 0.005        # smallest acceptable per-side margin
FINGER_THICKNESS = 0.015  # finger material outside the commanded aperture
FINGER_HALF_HEIGHT = 0.03
BODY_HALF_WIDTH = 0.05    # gripper body and wrist behind the fingers
BODY_LENGTH = 0.20


def frame(approach, finger):
    """4x4 rotation with X = approach, Y = finger, Z = X x Y."""
    x = np.asarray(approach, float)
    x /= np.linalg.norm(x)
    y = np.asarray(finger, float) - np.dot(finger, x) * x
    y /= np.linalg.norm(y)
    t = np.eye(4)
    t[:3, 0], t[:3, 1], t[:3, 2] = x, y, np.cross(x, y)
    return t


def _approach(yaw, pitch):
    """Horizontal arm direction (-y) turned by yaw, tilted down by -pitch."""
    c = math.cos(pitch)
    return np.array([c * math.sin(yaw), -c * math.cos(yaw), math.sin(pitch)])


def _candidate(obj, scene, rot, mode, pregrasp_offset, insert, extra=0.0, why=None):
    """Pose and aperture for one approach frame, or None if it does not fit
    (the reason is appended to `why` when given)."""
    def reject(text):
        if why is not None:
            why.append(text)
        return None
    a, f, z = rot[:3, 0], rot[:3, 1], rot[:3, 2]
    pa, pf, pz = obj @ a, obj @ f, obj @ z
    width = pf.max() - pf.min()
    if width + 2 * MIN_MARGIN > MAX_APERTURE:
        return reject(f'object {width * 100:.1f} cm across the fingers, gripper opens '
                      f'{MAX_APERTURE * 100:.0f} cm')
    depth_in = min(max(0.5 * width, 0.02), insert)
    mid_f = 0.5 * (pf.min() + pf.max())
    if mode == 'side':
        # Depth from the object's horizontal near face, height in world z: on a
        # tilted approach the projection along `a` picks the top front edge and
        # put the grasp ~1 cm short on a 13 cm tall object.
        h = np.array([a[0], a[1], 0.0]) / np.linalg.norm(a[:2])
        ph, wz = obj @ h, obj[:, 2]
        bottom, top = wz.min(), wz.max()
        zc = 0.5 * (bottom + top)
        # Short objects: grasp above mid-height so the fingers clear the
        # surface the object stands on (its lowest points), still below the top.
        zc += min(max(0.0, bottom + FINGER_HALF_HEIGHT + 0.015 - zc),
                  max(0.0, 0.5 * (top - bottom) - 0.01))
        center = h * (ph.min() + depth_in) + f * mid_f
        center[2] = zc
    else:
        center = a * (pa.min() + depth_in) + f * mid_f + z * 0.5 * (pz.min() + pz.max())

    # Scene points that are not the target itself.
    lo = np.array([pa.min(), pf.min(), pz.min()]) - 0.015
    hi = np.array([pa.max(), pf.max(), pz.max()]) + 0.015
    s = np.column_stack([scene @ a, scene @ f, scene @ z]) if len(scene) else \
        np.zeros((0, 3))
    keep = ~np.all((s >= lo) & (s <= hi), axis=1)
    s, world = s[keep], scene[keep] if len(scene) else np.zeros((0, 3))
    c = np.array([center @ a, center @ f, center @ z])
    rel = s - c

    # Gripper body and wrist corridor behind the pre-grasp point.
    body = ((rel[:, 0] > -(pregrasp_offset + BODY_LENGTH)) & (rel[:, 0] < -0.05) &
            (np.abs(rel[:, 1]) < BODY_HALF_WIDTH) &
            (np.abs(rel[:, 2]) < BODY_HALF_WIDTH))
    if body.any():
        hit = world[body]
        return reject(f'gripper body/wrist hits {int(body.sum())} scene points around z '
                      f'{np.median(hit[:, 2]):.2f} m (grasp z {center[2]:.2f})')
    # Finger sweep from pre-grasp to just past the pads (they end 1-2 cm past
    # the grasp center).
    sweep = ((rel[:, 0] > -pregrasp_offset) & (rel[:, 0] < 0.025) &
             (np.abs(rel[:, 2]) < FINGER_HALF_HEIGHT + 0.01))
    if mode == 'side':
        # The support surface and its depth noise (<= 1.2 cm above the object's
        # lowest point) cannot touch the fingers, held >= 1.5 cm above it.
        sweep &= world[:, 2] > obj[:, 2].min() + 0.012
    side = np.abs(rel[sweep, 1])
    clearance = side.min() if len(side) else math.inf
    allowed = min(MAX_APERTURE, 2 * (clearance - FINGER_THICKNESS))
    # `extra`: opening beyond the margins, still limited by neighbors and 13 cm.
    aperture = min(width + 2 * APERTURE_MARGIN + extra, allowed)
    if aperture < width + 2 * MIN_MARGIN:
        near = world[sweep][np.argsort(side)[:20]]
        return reject(f'only {max(clearance - FINGER_THICKNESS, 0) * 100:.1f} cm free beside the '
                      f'object for the fingers (needs {(width / 2 + MIN_MARGIN) * 100:.1f}); '
                      f'closest scene points around z {np.median(near[:, 2]):.2f} m '
                      f'(grasp z {center[2]:.2f})')

    grasp = rot.copy()
    grasp[:3, 3] = center
    pregrasp = grasp.copy()
    pregrasp[:3, 3] = center - a * pregrasp_offset
    lift = grasp.copy()
    lift[2, 3] += 0.03
    retreat = lift.copy()
    retreat[:3, 3] -= a * (pregrasp_offset + 0.05)
    return {'mode': mode, 'grasp': grasp, 'pregrasp': pregrasp, 'lift': lift,
            'retreat': retreat, 'aperture': float(aperture),
            'width': float(width), 'clearance': float(clearance)}


def plan_grasps(obj, scene, pregrasp_offset=0.10, insert=0.04, extra_opening=0.0,
                yaws=(0.0,), pitches=(0.0, -0.25), why=None):
    """Ranked grasp candidates for object points `obj` among `scene` points.

    Side grasps approach along the arm direction (-y), level or tilted down;
    a top-down grasp is added and only wins if every side grasp is blocked.
    Score prefers straight approaches and lateral clearance. No yawed side
    grasps by default: with the base fixed, wrist yaw is the only joint that
    moves the TCP along x, so a yawed approach cannot be a straight line.
    """
    obj = np.asarray(obj, float).reshape(-1, 3)
    scene = np.asarray(scene, float).reshape(-1, 3)
    if len(obj) < 10:
        return []
    out = []
    for yaw in yaws:
        for pitch in pitches:
            a = _approach(yaw, pitch)
            f = np.cross([0.0, 0.0, 1.0], a)
            reasons = []
            c = _candidate(obj, scene, frame(a, f), 'side', pregrasp_offset, insert,
                           extra_opening, reasons)
            if why is not None and reasons:
                why.append(f'side {"level" if pitch == 0 else f"tilted {pitch:+.2f}"}'
                           f'{"" if yaw == 0 else f" yaw {yaw:+.2f}"}: {reasons[0]}')
            if c:
                c['score'] = (min(c['clearance'], 0.1) - 0.3 * abs(yaw) -
                              0.2 * abs(pitch))
                c['key'] = ('side', yaw, pitch)  # same grasp across re-plans
                out.append(c)
    # Top-down: fingers close across the narrow horizontal axis.
    xy = obj[:, :2] - obj[:, :2].mean(0)
    w, v = np.linalg.eigh(np.cov(xy.T))
    narrow = np.append(v[:, int(np.argmin(w))], 0.0)
    reasons = []
    c = _candidate(obj, scene, frame([0.0, 0.0, -1.0], narrow), 'top',
                   pregrasp_offset, 0.03, extra_opening, reasons)
    if why is not None and reasons:
        why.append(f'top-down: {reasons[0]}')
    if c:
        c['score'] = min(c['clearance'], 0.1) - 0.5
        c['key'] = ('top', 0.0, 0.0)
        out.append(c)
    return sorted(out, key=lambda c: -c['score'])


def fits(obj, scene, rot, pregrasp_offset=0.10):
    """Candidate for an explicit gripper orientation (e.g. the FK-achieved
    one), or None when the fingers or the body do not fit there."""
    top = rot[2, 0] < -0.7
    return _candidate(np.asarray(obj, float).reshape(-1, 3),
                      np.asarray(scene, float).reshape(-1, 3), rot,
                      'top' if top else 'side', pregrasp_offset, 0.03 if top else 0.04)


def outside_gripper(points, grasp_center, half_width=0.075, half_height=0.05,
                    front=0.02, back=0.32):
    """Points outside the gripper and wrist envelope. `grasp_center` is the
    4x4 link_grasp_center pose in the points' frame (X approach, Y fingers);
    the box spans the open fingers (tips 1.5 cm behind the grasp center) back
    to the wrist yaw joint (0.27 m)."""
    local = (np.asarray(points) - grasp_center[:3, 3]) @ grasp_center[:3, :3]
    inside = ((local[:, 0] < front) & (local[:, 0] > -back) &
              (np.abs(local[:, 1]) < half_width) & (np.abs(local[:, 2]) < half_height))
    return points[~inside]


def _rot(axis, angle):
    k = np.asarray(axis, float) / np.linalg.norm(axis)
    kx = np.array([[0, -k[2], k[1]], [k[2], 0, -k[0]], [-k[1], k[0], 0]])
    return np.eye(3) + math.sin(angle) * kx + (1 - math.cos(angle)) * kx @ kx


def urdf_chain(xml, tip='link_grasp_center'):
    """Joints from the URDF root to `tip`: (name, type, origin 4x4, axis)."""
    import xml.etree.ElementTree as ET
    joints = {}
    for j in ET.fromstring(xml).findall('joint'):
        o, a = j.find('origin'), j.find('axis')
        xyz = [float(v) for v in (o.get('xyz', '0 0 0') if o is not None else '0 0 0').split()]
        r, p, y = [float(v) for v in (o.get('rpy', '0 0 0') if o is not None else '0 0 0').split()]
        m = np.eye(4)
        m[:3, :3] = _rot([0, 0, 1], y) @ _rot([0, 1, 0], p) @ _rot([1, 0, 0], r)
        m[:3, 3] = xyz
        axis = [float(v) for v in a.get('xyz').split()] if a is not None else [1, 0, 0]
        joints[j.find('child').get('link')] = (j.get('name'), j.get('type'), j.find('parent').get('link'), m, axis)
    chain, link = [], tip
    while link in joints:
        name, kind, parent, m, axis = joints[link]
        chain.append((name, kind, m, axis))
        link = parent
    return chain[::-1]


def fk(chain, q):
    """4x4 pose of the chain tip for joint values `q` (name -> value; missing = 0)."""
    t = np.eye(4)
    for name, kind, m, axis in chain:
        t = t @ m
        v = q.get(name, 0.0)
        motion = np.eye(4)
        if kind == 'prismatic':
            motion[:3, 3] = np.asarray(axis) * v
        elif kind in ('revolute', 'continuous'):
            motion[:3, :3] = _rot(axis, v)
        t = t @ motion
    return t


def quaternion(rot):
    """(x, y, z, w) of a 3x3 or 4x4 rotation."""
    m = np.asarray(rot)[:3, :3]
    w = math.sqrt(max(0.0, 1 + m[0, 0] + m[1, 1] + m[2, 2])) / 2
    x = math.copysign(math.sqrt(max(0.0, 1 + m[0, 0] - m[1, 1] - m[2, 2])) / 2,
                      m[2, 1] - m[1, 2])
    y = math.copysign(math.sqrt(max(0.0, 1 - m[0, 0] + m[1, 1] - m[2, 2])) / 2,
                      m[0, 2] - m[2, 0])
    z = math.copysign(math.sqrt(max(0.0, 1 - m[0, 0] - m[1, 1] + m[2, 2])) / 2,
                      m[1, 0] - m[0, 1])
    return x, y, z, w


def matrix_from_transform(t):
    """4x4 matrix from a geometry_msgs Transform (duck-typed)."""
    q, p = t.rotation, t.translation
    x, y, z, w = q.x, q.y, q.z, q.w
    m = np.eye(4)
    m[:3, :3] = [
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]]
    m[:3, 3] = [p.x, p.y, p.z]
    return m


def apply(m, points):
    return points @ m[:3, :3].T + m[:3, 3]


def mask_points(mask, depth_mm, fx, fy, cx, cy, erode=2, z_range=(0.15, 2.5)):
    """Camera-frame points of a mask over an aligned 16UC1 depth image.

    The mask border is eroded so depth from the background behind the object
    edge does not leak in.
    """
    m = mask.astype(bool)
    for _ in range(erode):
        m = m & np.roll(m, 1, 0) & np.roll(m, -1, 0) & np.roll(m, 1, 1) & \
            np.roll(m, -1, 1)
    v, u = np.nonzero(m)
    z = depth_mm[v, u].astype(float) / 1000.0
    keep = (z > z_range[0]) & (z < z_range[1])
    u, v, z = u[keep], v[keep], z[keep]
    return np.column_stack([(u - cx) * z / fx, (v - cy) * z / fy, z])
