"""Keyframe memory: head images saved with their camera pose in the map.

A query runs the detector with the user's own words over every keyframe, so
there is no vocabulary to fix in advance.
shortcut: one OWLv2 pass per keyframe per query (~0.4 s each on an RTX 4060);
cache OWLv2 image embeddings if the memory grows past ~100 keyframes.
"""
from dataclasses import dataclass

import numpy as np

from .geometry import backproject, box_depth, upright, upright_to_native


@dataclass
class Keyframe:
    rgb: np.ndarray          # native color image, HxWx3 RGB
    depth: np.ndarray        # native depth in metres, same size as rgb
    k: np.ndarray            # 3x3 intrinsics of the depth image
    map_from_cam: np.ndarray  # 4x4, depth optical frame -> map


@dataclass
class Hit:
    score: float
    xyz: np.ndarray          # object centre in map
    frame: int
    box: tuple               # native pixels (u0, v0, u1, v1)


def in_room(xy, room):
    (x0, x1), (y0, y1) = room
    return x0 <= xy[0] <= x1 and y0 <= xy[1] <= y1


def locate(frame, box_up, k=None):
    """Upright detection box -> (map xyz, native box), or None without depth."""
    h = frame.rgb.shape[0]
    u0, v0 = upright_to_native(box_up[2], box_up[1], h)   # upright top-right
    u1, v1 = upright_to_native(box_up[0], box_up[3], h)   # upright bottom-left
    box = (u0, v0, u1, v1)
    d = box_depth(frame.depth, box)
    if d is None:
        return None
    p = backproject((u0 + u1) / 2, (v0 + v1) / 2, d, frame.k if k is None else k)
    return (frame.map_from_cam @ np.append(p, 1.0))[:3], box


class Memory:
    def __init__(self, detector):
        self.detector = detector
        self.frames = []

    def add(self, frame):
        self.frames.append(frame)

    def save(self, path):
        np.savez_compressed(path, **{f'{k}_{i}': getattr(f, k) for i, f in enumerate(self.frames)
                                     for k in ('rgb', 'depth', 'k', 'map_from_cam')})

    def load(self, path):
        with np.load(path) as z:
            n = len(z.files) // 4
            self.frames = [Keyframe(*(z[f'{k}_{i}'] for k in ('rgb', 'depth', 'k', 'map_from_cam')))
                           for i in range(n)]

    def query(self, phrase, room=None, threshold=0.1):
        """Hits best first; room: ((x0, x1), (y0, y1)) in map or None."""
        hits = []
        for i, f in enumerate(self.frames):
            for score, box_up in self.detector.detect(upright(f.rgb), phrase, threshold):
                loc = locate(f, box_up)
                if loc is not None and (room is None or in_room(loc[0], room)):
                    hits.append(Hit(score, loc[0], i, loc[1]))
        return sorted(hits, key=lambda h: -h.score)
