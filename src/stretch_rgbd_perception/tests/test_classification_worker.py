from threading import Event
from types import SimpleNamespace
import unittest
from unittest.mock import Mock
from unittest.mock import patch

import numpy as np

from rgbd.detection.appearance import Observation
from rgbd.classification.worker import ObjectClassifier
from rgbd.classification.ocr import Reading
from rgbd.types import Frame


class WorkerTests(unittest.TestCase):
    def setUp(self):
        self.reader = SimpleNamespace(read=Mock(return_value=(Reading(0, 'Melatonina', .95, (), 0),)))
        self.classifier = ObjectClassifier(reader_factory=lambda: self.reader)
        self.classifier.future.result(timeout=2)
        self.classifier.poll([])
        self.frame = Frame(123, np.zeros((40, 60, 3), np.uint8), np.ones((20, 30), np.float32))
        self.observations = (Observation(None, (.03, .05, .14), (5, 5, 25, 30)),)
        self.track = SimpleNamespace(id=7, confirmed=True, detection_index=0, position=np.zeros(3))

    def tearDown(self):
        self.classifier.close()

    def complete(self, tracks=None):
        self.classifier.future.result(timeout=2)
        return self.classifier.poll([self.track] if tracks is None else tracks)

    def test_one_job_nonblocking_snapshot_and_capture_refresh(self):
        gate = Event()
        def slow(rgb, boxes):
            gate.wait(2)
            self.assertEqual(rgb[0, 0, 0], 0)
            return (Reading(0, 'Melatonina', .95, (), 0),)
        self.reader.read.side_effect = slow
        try:
            self.assertTrue(self.classifier.submit(self.frame, [self.track], self.observations, 0))
            self.frame.color[:] = 255
            self.assertIsNone(self.classifier.poll([self.track]))
            self.assertFalse(self.classifier.submit(self.frame, [self.track], self.observations, 2))
        finally:
            gate.set()
        event = self.complete()
        self.assertEqual(event['accepted'], [7])
        self.assertEqual(self.classifier.memory.labels[7].label, 'Melatonina')
        self.reader.read.side_effect = None
        self.assertFalse(self.classifier.submit(self.frame, [self.track], self.observations, 4.9))
        self.assertTrue(self.classifier.submit(self.frame, [self.track], self.observations, 5))
        self.complete()

    def test_clear_discards_pending_generation(self):
        self.classifier.submit(self.frame, [self.track], self.observations, 0)
        self.classifier.clear()
        event = self.complete()
        self.assertEqual(event['accepted'], [])
        self.assertFalse(self.classifier.memory.labels)

    def test_expired_id_rejects_delayed_text(self):
        self.classifier.submit(self.frame, [self.track], self.observations, 0)
        self.assertEqual(self.complete([])['accepted'], [])

    def test_unverified_move_rejected_but_visual_recovery_allows_it(self):
        self.classifier.submit(self.frame, [self.track], self.observations, 0)
        self.track.position = np.array([.4, 0, 0])
        self.assertEqual(self.complete()['accepted'], [])
        self.track.position[:] = 0
        self.classifier.submit(self.frame, [self.track], self.observations, 5)
        self.track.position = np.array([.4, 0, 0])
        self.track.appearance_verified_at, self.track.association = 6., 'appearance'
        self.assertEqual(self.complete()['accepted'], [7])

    def test_model_failure_is_reported_without_blocking_detection(self):
        def fail():
            raise RuntimeError('model unavailable')
        with ObjectClassifier(reader_factory=fail) as classifier:
            try:
                classifier.future.result(timeout=2)
            except RuntimeError:
                pass
            event = classifier.poll([self.track])
            self.assertEqual(event['kind'], 'ocr_error')
            self.assertIn('model unavailable', classifier.error)
            self.assertFalse(classifier.submit(self.frame, [self.track], self.observations, 0))

    def test_paused_demo_refresh_does_not_reprocess_capture(self):
        from rgbd.demos.common import run_demo
        frame = self.frame
        class Source:
            fps = 30
            def __len__(self): return 1
            def __getitem__(self, index): return frame
        args = SimpleNamespace(max_dimension=40, frames=1, start_frame=0, headless=False, output=None)
        image = np.zeros((40, 60, 3), np.uint8)
        process = Mock(return_value=(image, dict(ocr_state='loading')))
        refresh = Mock(return_value=(image, dict(ocr_state='idle')))
        with patch('rgbd.demos.common.cv2.imshow'), patch('rgbd.demos.common.cv2.waitKey', side_effect=[-1, 27]), \
                patch('rgbd.demos.common.cv2.getWindowProperty', return_value=1), patch('rgbd.demos.common.cv2.destroyAllWindows'):
            rows = run_demo(Source(), process, args, title='test', refresh=refresh)
        self.assertEqual(process.call_count, 1)
        self.assertEqual(refresh.call_count, 1)
        self.assertEqual(rows[-1]['ocr_state'], 'idle')


if __name__ == '__main__':
    unittest.main()
