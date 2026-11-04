"""Coordinate-contract regressions; real-image detection is checked with the bag demo."""

import unittest
from unittest.mock import Mock

import cv2
import numpy as np

from rgbd.pose.board import BoardPoseEstimator
from rgbd.demos.common import Playback
from rgbd.types import Camera, Frame, rotate_camera_clockwise


class PoseTests(unittest.TestCase):
    def setUp(self):
        # Off-center intrinsics and tangential distortion expose pixel-rotation errors.
        self.camera = Camera(640, 480, np.array([[600., 0, 301.], [0, 610., 252.], [0, 0, 1.]]),
                             np.array([.03, -.01, .002, -.004, 0.]), "plumb_bob")
        self.estimator = BoardPoseEstimator(self.camera)
        self.estimator._configure()
        objects = np.concatenate(self.estimator._grid.getObjPoints())
        objects = (objects-self.estimator._center)*[1, -1, -1]
        self.rotation = cv2.Rodrigues(np.array([2.9, .15, -.1]))[0]
        self.translation = np.array([.04, -.02, .8])
        self.pixels = cv2.projectPoints(objects, cv2.Rodrigues(self.rotation)[0], self.translation,
                                        self.camera.K, self.camera.D)[0].reshape(-1, 4, 2).astype(np.float32)

    def estimate(self, count=12):
        self.estimator._detector = Mock()
        self.estimator._detector.detectMarkers.return_value = (
            [p.reshape(1, 4, 2) for p in self.pixels[:count]],
            np.arange(count, dtype=np.int32).reshape(-1, 1) if count else None, ())
        frame = Frame(1234, np.zeros((self.camera.height, self.camera.width, 3), np.uint8),
                      np.zeros((1, 1), np.float32))  # Depth is deliberately unused.
        return self.estimator.estimate(frame)

    def test_pose_is_in_original_optical_axes_for_all_image_rotations(self):
        optical_rotation = np.array([[0., -1, 0], [1, 0, 0], [0, 0, 1]])
        rotation, translation = self.rotation, self.translation
        for _ in range(4):
            pose = self.estimate()
            np.testing.assert_allclose(pose.reference_to_camera[:3, :3], rotation, atol=2e-5)
            np.testing.assert_allclose(pose.reference_to_camera[:3, 3], translation, atol=2e-6)
            np.testing.assert_allclose(pose.camera_to_reference @ pose.reference_to_camera, np.eye(4), atol=1e-12)
            self.assertEqual(pose.timestamp_ns, 1234)
            np.testing.assert_allclose(self.estimator.observation.corners[0][0], self.pixels[0])
            self.pixels = np.stack((self.camera.height-1-self.pixels[..., 1], self.pixels[..., 0]), axis=-1)
            self.camera = rotate_camera_clockwise(self.camera)
            self.estimator.camera = self.camera
            rotation, translation = optical_rotation @ rotation, optical_rotation @ translation

    def test_partial_board_and_live_minimum_do_not_return_stale_pose(self):
        self.assertIsNotNone(self.estimate(2))
        self.estimator.parameters.min_markers = 3
        self.assertIsNone(self.estimate(2))
        self.assertIsNone(self.estimate(0))
        self.assertEqual(len(self.estimator.observation.ids), 0)

    def test_runtime_error_threshold(self):
        self.pixels[0, 0] += [7, -4]
        self.estimator.parameters.max_reprojection_error = 10
        self.assertIsNotNone(self.estimate())
        self.estimator.parameters.max_reprojection_error = .1
        self.assertIsNone(self.estimate())
        self.assertEqual(self.estimator.observation.status, "reprojection error too high")

    def test_board_scale_rebuilds_geometry_and_changes_metric_translation(self):
        first = self.estimate()
        self.estimator.board.scale *= .9
        self.estimator._configure()
        second = self.estimate()
        np.testing.assert_allclose(second.reference_to_camera[:3, 3], first.reference_to_camera[:3, 3]*.9, atol=1e-6)


class PlaybackTests(unittest.TestCase):
    def test_shared_keys(self):
        class Source:
            fps = 10
            def __len__(self): return 200
        playback = Playback(Source(), 137)
        playback.update(ord('d'), now=0)
        playback.update(now=.11)
        self.assertEqual(playback.index, 138)
        playback.update(ord('s'), now=.2)
        playback.update(now=1)
        self.assertEqual(playback.index, 138)
        playback.update(ord('q'))
        playback.update(ord('e'))
        self.assertEqual(playback.index, 138)
        playback.update(ord('a'), now=2)
        playback.update(now=2.11)
        self.assertEqual(playback.index, 137)
        playback.update(ord('e'))
        playback.update(ord('w'))
        self.assertEqual((playback.index, playback.direction), (137, 0))
        playback.update(27)
        self.assertTrue(playback.quit)


if __name__ == "__main__":
    unittest.main()
