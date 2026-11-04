import unittest

import numpy as np

from rgbd.pose.filter import Parameters, PoseFilter
from rgbd.types import Pose


def pose(x, timestamp_ns, angle=0., reference="board"):
    angle = np.deg2rad(angle)
    c, s = np.cos(angle), np.sin(angle)
    camera_to_reference = np.array([[c, -s, 0, x], [s, c, 0, 0], [0, 0, 1, 1.], [0, 0, 0, 1.]])
    return Pose(timestamp_ns, np.linalg.inv(camera_to_reference), reference)


class FilterTests(unittest.TestCase):
    def test_averages_camera_positions_and_handles_rotation_wrap(self):
        smoother = PoseFilter()
        smoother.update(pose(0., 0, 179))
        smoother.update(pose(.3, 100_000_000, -179))
        result = smoother.update(pose(.6, 200_000_000, 180))
        matrix = result.camera_to_reference
        np.testing.assert_allclose(matrix[:3, 3], [.3, 0, 1.], atol=1e-12)
        np.testing.assert_allclose(matrix[:3, :3], np.diag([-1, -1, 1]), atol=1e-12)
        np.testing.assert_allclose(matrix[:3, :3].T @ matrix[:3, :3], np.eye(3), atol=1e-12)
        self.assertAlmostEqual(np.linalg.det(matrix[:3, :3]), 1.)
        self.assertEqual(result.timestamp_ns, 200_000_000)

    def test_missing_gap_and_reference_change_reset_without_stale_output(self):
        smoother = PoseFilter()
        smoother.update(pose(0, 0))
        self.assertIsNone(smoother.update(None))
        for value in (pose(1, 100_000_000), pose(2, 1_000_000_000), pose(3, 1_100_000_000, reference="odom")):
            result = smoother.update(value)
            np.testing.assert_allclose(result.reference_to_camera, value.reference_to_camera)
            self.assertEqual(len(smoother.history), 1)

    def test_live_window_change_and_input_copy(self):
        smoother = PoseFilter()
        first = pose(0, 0)
        smoother.update(first)
        first.reference_to_camera[0, 3] = -999
        result = smoother.update(pose(2, 100_000_000))
        self.assertAlmostEqual(result.camera_to_reference[0, 3], 1.)
        smoother.parameters.window = 1
        result = smoother.update(pose(5, 200_000_000))
        self.assertAlmostEqual(result.camera_to_reference[0, 3], 5.)
        self.assertEqual(len(smoother.history), 1)

    def test_reversal_resets_once_and_backward_playback_then_filters(self):
        smoother = PoseFilter()
        for time in (0, 100_000_000, 200_000_000):
            smoother.update(pose(0, time))
        smoother.update(pose(1, 100_000_000))
        self.assertEqual(len(smoother.history), 1)
        result = smoother.update(pose(2, 0))
        self.assertAlmostEqual(result.camera_to_reference[0, 3], 1.5)
        smoother.update(pose(3, 0))
        self.assertEqual(len(smoother.history), 1)

    def test_invalid_parameters(self):
        for parameters in (Parameters(window=0), Parameters(window=1.5), Parameters(max_gap_seconds=0)):
            with self.assertRaises(ValueError):
                PoseFilter(parameters).update(pose(0, 0))


if __name__ == "__main__":
    unittest.main()
