"""Short-window camera-pose smoothing, independent of images and pose provider."""

from collections import deque
from dataclasses import dataclass

import numpy as np

from rgbd.types import Pose


@dataclass
class Parameters:
    window: int = 3
    max_gap_seconds: float = .5


class PoseFilter:
    """Average camera positions and rotations in the fixed reference frame.

    update(Pose | None) -> Pose | None. Output retains the current capture time.
    Missing pose, reference changes, large capture gaps, repeated timestamps and
    playback direction changes reset history. No extrapolation or stale output.
    Parameters may change between calls; shrinking the window trims history.
    """

    def __init__(self, parameters=None):
        self.parameters = Parameters() if parameters is None else parameters
        self.reset()

    def reset(self):
        self.history = deque()
        self._timestamp = self._reference = None
        self._direction = 0

    def update(self, pose: Pose | None) -> Pose | None:
        p = self.parameters
        if not isinstance(p.window, int) or p.window < 1:
            raise ValueError("window must be a positive integer")
        if not np.isfinite(p.max_gap_seconds) or p.max_gap_seconds <= 0:
            raise ValueError("max_gap_seconds must be finite and positive")
        if pose is None:
            self.reset()
            return None
        if self._timestamp is not None:
            delta = (pose.timestamp_ns-self._timestamp)/1e9
            direction = int(np.sign(delta))
            if (pose.reference_frame != self._reference or not direction
                    or abs(delta) > p.max_gap_seconds or self._direction not in (0, direction)):
                self.reset()
            else:
                self._direction = direction
        self.history.append(pose.camera_to_reference.copy())
        while len(self.history) > p.window:
            self.history.popleft()
        self._timestamp, self._reference = pose.timestamp_ns, pose.reference_frame
        matrices = np.array(self.history)
        # Project the mean rotation onto SO(3), avoiding Euler-angle wraparound
        # and ensuring a proper rotation rather than an element-wise matrix mean.
        u, _, vt = np.linalg.svd(matrices[:, :3, :3].mean(axis=0))
        rotation = u @ np.diag([1., 1., np.linalg.det(u @ vt)]) @ vt
        camera_to_reference = np.eye(4)
        camera_to_reference[:3, :3] = rotation
        camera_to_reference[:3, 3] = matrices[:, :3, 3].mean(axis=0)
        return Pose(pose.timestamp_ns, np.linalg.inv(camera_to_reference), pose.reference_frame)


if __name__ == "__main__":
    from rgbd.demos.pose_filter import main
    main()
