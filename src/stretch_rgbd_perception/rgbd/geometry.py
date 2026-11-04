"""Geometry in the shared optical conventions; no board-specific conversions."""

import cv2
import numpy as np


def padded_box(box, shape, fraction=.1):
    """Exclusive RGB box with modest context, clipped to the source image."""
    x0, y0, x1, y1 = box
    px, py = max(4, round((x1-x0)*fraction)), max(4, round((y1-y0)*fraction))
    return max(0, x0-px), max(0, y0-py), min(shape[1], x1+px), min(shape[0], y1+py)


def reference_positions(detections, calibration, pose):
    transform = pose.camera_to_reference @ calibration.depth_to_color
    points = np.array([d.centroid for d in detections], dtype=float).reshape(-1, 3)
    return points @ transform[:3, :3].T + transform[:3, 3]


def rgb_ownership(frame, detections, calibration, stride=2, scale=1.):
    """Sparse RGB labels, nearest measured depth wins, including non-object depth.

    Output is native RGB resolution with -1 for unknown/background. The stride
    selects exact native depth rays, consistent with tabletop decimation.
    """
    c = calibration
    if np.any(c.depth.D):
        raise ValueError('Depth must be rectified')
    y, x = np.mgrid[stride-1:c.depth.height:stride, stride-1:c.depth.width:stride]
    z = frame.depth[y, x]
    valid = np.isfinite(z) & (z > 0)
    y, x, z = y[valid], x[valid], z[valid]
    k = c.depth.K
    points = np.column_stack(((x-k[0, 2])*z/k[0, 0], (y-k[1, 2])*z/k[1, 1], z))
    # These are tiny 3D operations over many points; avoid BLAS thread startup.
    points = np.einsum('ij,kj->ki', c.depth_to_color[:3, :3], points) + c.depth_to_color[:3, 3]
    keep = points[:, 2] > 0
    points, y, x = points[keep], y[keep], x[keep]
    height, width = round(c.color.height*scale), round(c.color.width*scale)
    result = np.full((height, width), -1, np.int32)
    if not len(points):
        return result
    if np.any(c.color.D):
        pixels = cv2.projectPoints(points, np.zeros(3), np.zeros(3), c.color.K, c.color.D)[0].reshape(-1, 2)
    else:
        pixels = points[:, :2]/points[:, 2, None]*[c.color.K[0, 0], c.color.K[1, 1]]+c.color.K[:2, 2]
    pixels = np.floor((pixels+.5)*[width/c.color.width, height/c.color.height]).astype(int)
    u, v = pixels.T
    keep = (u >= 0) & (u < width) & (v >= 0) & (v < height)
    u, v, points, y, x = u[keep], v[keep], points[keep], y[keep], x[keep]
    keys = v*width+u
    order = np.lexsort((points[:, 2], keys))
    _, first = np.unique(keys[order], return_index=True)
    nearest = order[first]
    labels = np.full(len(x), -1, np.int32)
    for i, detection in enumerate(detections):
        labels[detection.mask[y, x]] = i
    result[v[nearest], u[nearest]] = labels[nearest]
    return result
