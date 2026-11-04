"""Source-independent RGB-D data: RGB uint8, depth float meters, capture time ns.

Images and calibration must have the same orientation and native dimensions.
Camera axes are X right, Y down, Z forward; depth_to_color uses meters.
"""

from dataclasses import dataclass
from typing import Iterator, Protocol

import numpy as np


@dataclass
class Camera:
    width: int
    height: int
    K: np.ndarray
    D: np.ndarray
    distortion_model: str


@dataclass
class Calibration:
    color: Camera
    depth: Camera
    depth_to_color: np.ndarray


@dataclass
class Frame:
    timestamp_ns: int
    color: np.ndarray
    depth: np.ndarray


@dataclass
class Pose:
    """Rigid reference -> RGB optical camera transform; translation in meters.

    Uses the same capture timestamp and image orientation as Frame. None, rather
    than an old transform, represents unavailable pose. reference_frame must stay
    fixed while object memory is active. No provider-specific pixel conventions.
    """

    timestamp_ns: int
    reference_to_camera: np.ndarray
    reference_frame: str = "board"

    @property
    def camera_to_reference(self):
        return np.linalg.inv(self.reference_to_camera)


@dataclass
class Detection:
    """One current-frame object; mask is in depth pixels, geometry in depth axes."""

    mask: np.ndarray  # bool HxW
    centroid: np.ndarray  # XYZ meters
    extent: np.ndarray  # axis-aligned XYZ spans, meters
    box: tuple[int, int, int, int] | None = None  # Inclusive depth x0,y0,x1,y1, with bottom padding.
    footprint: float = 0.  # Visible diameter in the support plane, meters.
    height: float = 0.  # Highest measured point above the support, meters.


@dataclass
class Track:
    """Persistent instance, with position in the fixed reference, in meters."""

    id: int
    position: np.ndarray
    detection_index: int | None = None
    confirmed: bool = False
    seen_seconds: float = 0.
    missing_seconds: float = 0.

    @property
    def visible(self):
        return self.detection_index is not None


class FrameSource(Protocol):
    """Bag/live/ROS adapters supply calibration and capture-ordered frames.

    Indexing, frame count and FPS are optional playback-only extensions.
    The caller owns source lifetime and synchronization with external pose.
    """

    calibration: Calibration

    def __iter__(self) -> Iterator[Frame]: ...


class PoseEstimator(Protocol):
    def estimate(self, frame: Frame) -> Pose | None: ...


def rotate_camera_clockwise(camera):
    """Rotate pixels and optical axes clockwise, including Brown distortion."""
    K, D = camera.K, camera.D.copy()
    rotated_K = np.array([[K[1, 1], 0, camera.height-1-K[1, 2]],
                          [0, K[0, 0], K[0, 2]], [0, 0, 1]])
    if ("brown conrady" in camera.distortion_model.lower() or
            camera.distortion_model.lower() in ("plumb_bob", "rational_polynomial")) and len(D) >= 4:
        D[2], D[3] = D[3], -D[2]
    return Camera(camera.height, camera.width, rotated_K, D, camera.distortion_model)


def rotate_calibration(calibration, rotations):
    color, depth, transform = calibration.color, calibration.depth, calibration.depth_to_color.copy()
    axes = np.eye(4)
    axes[:3, :3] = ((0, -1, 0), (1, 0, 0), (0, 0, 1))
    for _ in range(rotations):
        color, depth = rotate_camera_clockwise(color), rotate_camera_clockwise(depth)
        transform = axes @ transform @ axes.T
    return Calibration(color, depth, transform)
