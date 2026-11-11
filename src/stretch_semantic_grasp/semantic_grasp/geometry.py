"""Pixel bookkeeping for the sideways-mounted head camera, and back-projection."""
import numpy as np


def upright(img):
    """The D435i is mounted rotated: turn the native image 90 deg clockwise."""
    return np.ascontiguousarray(np.rot90(img, -1))


def upright_to_native(u, v, native_h):
    """Pixel in upright(img) -> pixel in the native image of height native_h.
    rot90(k=-1) maps native (r, c) to upright (c, H-1-r)."""
    return v, native_h - 1 - u          # native (u=col, v=row)


def backproject(u, v, depth_m, k):
    """Pixel + depth (m) -> point in the optical frame. k: 3x3 intrinsics."""
    return np.array([(u - k[0][2]) * depth_m / k[0][0], (v - k[1][2]) * depth_m / k[1][1], depth_m])


def box_depth(depth, box, shrink=0.3):
    """Median valid depth (m) in the central part of box (u0, v0, u1, v1), or None."""
    u0, v0, u1, v1 = box
    du, dv = (u1 - u0) * shrink / 2, (v1 - v0) * shrink / 2
    h, w = depth.shape
    patch = depth[max(0, int(v0 + dv)):min(h, int(v1 - dv) + 1),
                  max(0, int(u0 + du)):min(w, int(u1 - du) + 1)]
    vals = patch[np.isfinite(patch) & (patch > 0.05)]
    return float(np.median(vals)) if vals.size else None
