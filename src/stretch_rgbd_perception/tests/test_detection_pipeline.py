import unittest
from types import SimpleNamespace

import numpy as np

from rgbd.detection.pipeline import DetectionPipeline
from rgbd.types import Calibration, Camera, Detection, Frame, Pose


class PipelineTests(unittest.TestCase):
    def setUp(self):
        camera = Camera(4, 4, np.eye(3), np.zeros(5), 'none')
        self.calibration = Calibration(camera, camera, np.eye(4))
        self.detection = Detection(np.ones((4, 4), bool), np.array([0, 0, 1.]), np.ones(3)*.1)
        segmentation = SimpleNamespace(detections=(self.detection,), timings_ms={'total': 0})
        self.segmenter = SimpleNamespace(segment=lambda depth, pose: segmentation, reset=lambda: None)
        self.extractor = SimpleNamespace(observe=lambda frame, objects: (None,), parameters=None)
        self.pipeline = DetectionPipeline(self.calibration, segmenter=self.segmenter, extractor=self.extractor)
        self.pipeline.persistor.parameters.probation_period = .1

    def frame(self, timestamp):
        return Frame(timestamp, np.zeros((4, 4, 3), np.uint8), np.ones((4, 4), np.float32))

    def test_external_odom_pose_and_capture_dt(self):
        first = self.pipeline.process(self.frame(0), Pose(0, np.eye(4), 'odom'))
        second = self.pipeline.process(self.frame(100_000_000), Pose(100_000_000, np.eye(4), 'odom'))
        self.assertFalse(first.tracks[0].confirmed)
        self.assertTrue(second.tracks[0].confirmed)
        self.assertEqual(second.dt, .1)
        self.assertEqual(first.tracks[0].id, second.tracks[0].id)
        missing = self.pipeline.process(self.frame(200_000_000), None)
        self.assertFalse(missing.tracks[0].visible)

    def test_sync_and_timestamp_contract(self):
        with self.assertRaises(ValueError):
            self.pipeline.process(self.frame(0), Pose(1, np.eye(4)))
        with self.assertRaises(ValueError):
            self.pipeline.process(self.frame(.1))
        self.pipeline.process(self.frame(1), Pose(1, np.eye(4)))
        for timestamp in (1, 0):
            with self.assertRaises(ValueError):
                self.pipeline.process(self.frame(timestamp))

    def test_reverse_playback_and_reset_preserve_nonrecycled_ids(self):
        self.pipeline.allow_reverse = True
        first = self.pipeline.process(self.frame(2), Pose(2, np.eye(4)))
        self.pipeline.process(self.frame(1), Pose(1, np.eye(4)))
        self.pipeline.reset()
        result = self.pipeline.process(self.frame(0), Pose(0, np.eye(4)))
        self.assertGreater(result.tracks[0].id, first.tracks[0].id)
        self.assertEqual(result.dt, 0)


if __name__ == '__main__':
    unittest.main()
