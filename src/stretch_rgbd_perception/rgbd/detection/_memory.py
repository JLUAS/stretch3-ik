"""Position-only object memory; all lifecycle timing comes from the caller's dt."""

import numpy as np
from scipy.optimize import linear_sum_assignment
from rgbd.types import Track


class ObjectMemory:
    """One-to-one 3D matching, continuous probation, and bounded lost-track memory.

    Missing a detection discards probationary tracks immediately. Confirmed
    tracks survive until forget_after seconds without a match. IDs are never
    reused, even after clear(). Input positions must share a fixed reference.
    """

    def __init__(self, parameters):
        self.parameters = parameters
        self.tracks = []
        self.elapsed_seconds = 0
        self.next_id = 0

    def clear(self):
        self.tracks.clear()

    def _costs(self, positions, observations, previous_visible):
        old = np.array([t.position for t in self.tracks])
        distance = np.linalg.norm(old[:, None]-positions[None, :], axis=2)
        return np.where(distance <= self.parameters.match_distance, distance, np.inf)

    def _record_match(self, track, index, position, observation, dt):
        track.position = position.copy()
        track.detection_index = int(index)
        track.missing_seconds = 0
        track.seen_seconds += dt
        track.confirmed |= track.seen_seconds >= self.parameters.probation_period

    def _make_track(self, position, index, observation):
        return Track(self.next_id, position.copy(), confirmed=self.parameters.probation_period == 0, detection_index=index)

    def _allow_new(self, index):
        return True

    def update(self, positions, dt, *, observations=None):
        """Observe one processed frame; dt is nonnegative video time in seconds."""
        positions = np.asarray(positions, dtype=float).reshape(-1, 3)
        if not np.isfinite(positions).all() or not np.isfinite(dt) or dt < 0:
            raise ValueError("Require finite positions and finite nonnegative dt")
        self.elapsed_seconds += dt
        previous_visible = {t.id: t.visible for t in self.tracks}
        for track in self.tracks:
            track.missing_seconds += dt
            track.detection_index = None
        # Expire before matching: a jump beyond the retention time starts new IDs.
        self.tracks = [t for t in self.tracks if t.missing_seconds < self.parameters.forget_after]
        matched = set()
        if self.tracks and len(positions):
            costs = self._costs(positions, observations, previous_visible)
            penalty = (len(self.tracks) + 1) * self.parameters.match_distance
            # Dummy columns let tracks remain unmatched. Penalize invalid pairs
            # before assignment, so they cannot steal a valid detection.
            cost = np.column_stack((np.where(np.isfinite(costs), costs, 2*penalty),
                                    np.full((len(self.tracks), len(self.tracks)), penalty)))
            rows, columns = linear_sum_assignment(cost)
            for row, column in zip(rows, columns):
                if column >= len(positions):
                    continue
                track = self.tracks[row]
                observation = None if observations is None else observations[column]
                self._record_match(track, column, positions[column], observation, dt)
                matched.add(int(column))
        self.tracks = [t for t in self.tracks if t.visible or t.confirmed]
        for index, position in enumerate(positions):
            if index not in matched and self._allow_new(index):
                observation = None if observations is None else observations[index]
                self.tracks.append(self._make_track(position, index, observation))
                self.next_id += 1
        return self.tracks
