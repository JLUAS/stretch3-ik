"""Depth-only geometry, native mask contracts, and reference-plane transport."""

import unittest

import cv2
import numpy as np

from rgbd.types import Calibration, Camera, Pose
from rgbd.detection.tabletop import Parameters, TabletopSegmenter


class SegmentationTests(unittest.TestCase):
    def setUp(self):
        camera = Camera(320, 240, np.array([[300., 0, 160], [0, 300, 80], [0, 0, 1]]), np.zeros(5), 'none')
        extrinsics = np.eye(4)
        extrinsics[:3, :3] = cv2.Rodrigues(np.array([.01, -.02, .03]))[0]
        extrinsics[:3, 3] = [.014, -.006, .002]
        self.calibration = Calibration(camera, camera, extrinsics)
        self.parameters = Parameters(max_depth=1.1, plane_min_inliers=80, object_roi_top=.3)
        self.segmenter = TabletopSegmenter(self.calibration, self.parameters)
        self.pose = Pose(0, np.eye(4))
        self.plane = np.array([0., -1, 0, .2])

    def depth_for(self, plane):
        y, x = np.indices((240, 320))
        rays = np.stack(((x-160)/300, (y-80)/300, np.ones(x.shape)), axis=-1)
        with np.errstate(divide='ignore', invalid='ignore'):
            depth = -plane[3]/(rays @ plane[:3])
        depth[~np.isfinite(depth) | (depth < .25) | (depth > 1.1)] = 0
        return depth.astype(np.float32)

    def test_native_masks_boxes_footprint_and_runtime_threshold(self):
        depth = self.depth_for(self.plane)
        depth[120:165, 90:190] = .7  # Wide wall-like patch: >20cm footprint.
        depth[90:165, 235:262] = .7  # Tall, narrow box.
        depth[100, 240] = 0
        result = self.segmenter.segment(depth)
        self.assertEqual(len(result.detections), 1)
        self.assertEqual(result.rejections.get('footprint'), 1)
        self.assertEqual(result.object_labels.shape, depth.shape)
        self.assertFalse(result.detections[0].mask[100, 240])
        detection = result.detections[0]
        self.assertEqual(detection.mask.dtype, bool)
        np.testing.assert_array_equal(detection.mask, result.object_labels == 0)
        ys, xs = np.nonzero(detection.mask)
        x0, y0, x1, y1 = detection.box
        self.assertLessEqual(x0, xs.min())
        self.assertGreaterEqual(x1, xs.max())
        self.assertLessEqual(y0, ys.min())
        self.assertGreaterEqual(y1, ys.max()+self.parameters.bottom_padding_pixels)
        self.assertGreater(detection.height, .15)
        self.assertLess(detection.footprint, .1)
        self.parameters.max_object_footprint = None
        self.assertEqual(len(self.segmenter.segment(depth).detections), 2)

    def test_camera_motion_transports_plane_through_extrinsics(self):
        first = self.segmenter.segment(self.depth_for(self.plane), self.pose)
        self.assertEqual(first.plane_status, 'acquired')
        stored = self.segmenter.reference_plane
        reference_to_camera = np.eye(4)
        reference_to_camera[:3, :3] = cv2.Rodrigues(np.array([.07, .03, -.02]))[0]
        reference_to_camera[:3, 3] = [.04, .025, .02]
        moved = Pose(100_000_000, reference_to_camera)
        depth_to_reference = moved.camera_to_reference @ self.calibration.depth_to_color
        expected = depth_to_reference.T @ stored
        expected /= np.linalg.norm(expected[:3])
        second = self.segmenter.segment(self.depth_for(expected), moved)
        self.assertEqual(second.plane_status, 'tracked')
        self.assertEqual(second.timings_ms['plane_ransac'], 0)
        self.assertGreater(second.support_count, 25)
        np.testing.assert_allclose(second.plane, expected, atol=1e-6)
        np.testing.assert_allclose(self.segmenter.reference_plane, stored, atol=1e-6)

    def test_pose_and_depth_loss_do_not_reuse_unvalidated_support(self):
        depth = self.depth_for(self.plane)
        self.segmenter.segment(depth, self.pose)
        stored = self.segmenter.reference_plane
        without_pose = self.segmenter.segment(depth)
        self.assertEqual(without_pose.plane_status, 'search (no pose)')
        self.assertGreater(without_pose.timings_ms['plane_ransac'], 0)
        np.testing.assert_array_equal(self.segmenter.reference_plane, stored)
        empty = self.segmenter.segment(np.zeros_like(depth), self.pose)
        self.assertEqual(empty.detections, ())
        self.assertIsNone(empty.plane)
        self.assertEqual(empty.plane_status, 'unavailable')
        self.assertTrue(empty.error)
        self.assertFalse(empty.table_mask.any())
        self.assertTrue(np.all(empty.object_labels == -1))
        self.assertEqual(self.segmenter.segment(depth, self.pose).plane_status, 'tracked')

    def test_changed_support_and_reference_reacquire(self):
        self.segmenter.segment(self.depth_for(self.plane), self.pose)
        changed = np.array([0., -1, 0, .3])
        result = self.segmenter.segment(self.depth_for(changed), self.pose)
        self.assertEqual(result.plane_status, 'acquired')
        np.testing.assert_allclose(result.plane, changed, atol=1e-6)
        other_reference = Pose(200_000_000, np.eye(4), 'odom')
        result = self.segmenter.segment(self.depth_for(changed), other_reference)
        self.assertEqual(result.plane_status, 'acquired')
        self.segmenter.reset()
        self.assertIsNone(self.segmenter.reference_plane)

    def test_noisy_local_fit_is_rejected_even_with_enough_support(self):
        rng = np.random.default_rng(0)
        points = np.column_stack((rng.uniform(-.2, .2, 1000), np.full(1000, .2), rng.uniform(.5, 1., 1000)))
        points[:, 1] += rng.choice([-1, 1], 1000)*.001
        self.assertIsNotNone(self.segmenter._memory.validate(self.plane, points, 25, .01))
        points[:, 1] = .2+rng.choice([-1, 1], 1000)*.006
        self.assertIsNone(self.segmenter._memory.validate(self.plane, points, 25, .01))


if __name__ == '__main__':
    unittest.main()
