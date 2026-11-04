import unittest
from unittest.mock import patch

import numpy as np

from rgbd.detection.appearance import Observation, Similarity
from rgbd.detection.persistence import ObjectPersistor, Parameters, _AppearanceMemory
from rgbd.types import Calibration, Camera, Detection, Pose
from tests.test_appearance import signature


class MemoryTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.a, cls.b = signature(2), signature(5)
        cls.blue, cls.red = signature(3, (30, 90, 230)), signature(4, (230, 40, 30))

    def memory(self, **kwargs):
        return _AppearanceMemory(Parameters(probation_period=0, **kwargs))

    def observation(self, value, extent=(.03, .055, .14)):
        return Observation(value, extent)

    def test_recovers_after_occlusion_and_45cm_move(self):
        memory = self.memory()
        obs = self.observation(self.a)
        memory.update([[0, 0, 1]], 0, observations=[obs])
        memory.update([], 2, observations=[])
        tracks = memory.update([[.45, 0, 1]], 1, observations=[obs])
        self.assertEqual(len(tracks), 1)
        self.assertEqual((tracks[0].id, tracks[0].recoveries), (0, 1))
        self.assertEqual(memory.events[0]['kind'], 'appearance_recovery')

    def test_ambiguous_identical_instances_do_not_get_arbitrary_recovery(self):
        memory = self.memory()
        obs = self.observation(self.a)
        memory.update([[0, 0, 1], [.25, 0, 1]], 0, observations=[obs, obs])
        memory.update([], 1, observations=[])
        tracks = memory.update([[.6, 0, 1]], .1, observations=[obs])
        self.assertEqual([(t.id, t.visible) for t in tracks], [(0, False), (1, False), (2, True)])

    def test_white_box_with_different_print_is_not_recovered_by_color(self):
        memory = self.memory()
        memory.update([[0, 0, 1]], 0, observations=[self.observation(self.a)])
        memory.update([], 1, observations=[])
        tracks = memory.update([[.4, 0, 1]], .1, observations=[self.observation(self.b)])
        self.assertEqual(tracks[-1].id, 1)

    def test_merge_does_not_steal_id_or_train_gallery(self):
        memory = self.memory()
        obs = self.observation(self.a)
        track = memory.update([[0, 0, 1]], 0, observations=[obs])[0]
        tracks = memory.update([[.03, 0, 1]], .2, observations=[self.observation(self.a, (.1, .18, .3))])
        self.assertEqual(len(tracks), 1)
        self.assertFalse(track.visible)
        self.assertEqual(len(track.templates), 1)
        memory.update([[.35, 0, 1]], .2, observations=[obs])
        self.assertTrue(track.visible)
        self.assertEqual(track.id, 0)

    def test_probation_missing_retention_expiry_and_live_parameters(self):
        memory = _AppearanceMemory(Parameters(probation_period=1.))
        memory.update([[0, 0, 1]], 0)
        self.assertFalse(memory.tracks[0].confirmed)
        memory.update([[0, 0, 1]], .6)
        memory.update([[0, 0, 1]], .5)
        self.assertTrue(memory.tracks[0].confirmed)
        memory.update([], .5)
        self.assertFalse(memory.tracks[0].visible)
        memory.parameters.forget_after = .6
        memory.update([], .2)
        self.assertEqual(memory.tracks, [])
        self.assertEqual(memory.update([[0, 0, 1]], 0)[0].id, 1)

    def test_reference_conversion_pose_loss_and_snapshot_independence(self):
        camera = Camera(2, 2, np.eye(3), np.zeros(5), 'none')
        extrinsics = np.eye(4)
        extrinsics[0, 3] = .02
        calibration = Calibration(camera, camera, extrinsics)
        persistor = ObjectPersistor(calibration, Parameters(probation_period=0))
        detection = Detection(np.ones((2, 2), bool), np.array([0, 0, 1.]), np.array([.05, .05, .1]))
        first = persistor.update([detection], Pose(0, np.eye(4)), 0)
        np.testing.assert_allclose(first[0].position, [.02, 0, 1])
        moved_camera = np.eye(4)
        moved_camera[0, 3] = -.1
        detection.centroid[0] = -.1
        second = persistor.update([detection], Pose(1, moved_camera), .1)
        self.assertEqual(second[0].id, first[0].id)
        second[0].position[:] = 99
        self.assertLess(persistor.memory.tracks[0].position[0], 1)
        lost = persistor.update([detection], None, .1)
        self.assertFalse(lost[0].visible)
        self.assertTrue(first[0].visible)
        switched = persistor.update([detection], Pose(2, np.eye(4), 'odom'), .1)
        self.assertEqual(switched[0].id, 1)

    def test_swapped_objects_follow_appearance(self):
        memory = self.memory()
        observations = [self.observation(self.blue), self.observation(self.red)]
        memory.update([[0, 0, 1], [.3, 0, 1]], 0, observations=observations)
        tracks = memory.update([[.3, 0, 1], [0, 0, 1]], .1, observations=observations)
        self.assertEqual([(t.id, t.detection_index) for t in tracks], [(0, 0), (1, 1)])

    def test_tall_neighbor_is_not_mistaken_for_a_merge(self):
        memory = self.memory()
        observations = [self.observation(self.blue, (.02, .04, .18)), self.observation(self.red, (.02, .04, .07))]
        positions = [[0, 0, 1], [.08, 0, 1]]
        memory.update(positions, 0, observations=observations)
        tracks = memory.update(positions, .1, observations=observations)
        self.assertTrue(all(t.visible for t in tracks))
        self.assertEqual(memory.next_id, 2)

    def test_gallery_freeze_and_original_view_retention(self):
        memory = self.memory()
        obs = self.observation(self.a)
        track = memory.update([[0, 0, 1]], 0, observations=[obs])[0]
        track.gallery_frozen = True
        memory.update([[0, 0, 1]], 2, observations=[obs])
        self.assertEqual(len(track.templates), 1)
        track.gallery_frozen = False
        for _ in range(5):
            memory.update([[0, 0, 1]], 1.1, observations=[obs])
        self.assertEqual(len(track.templates), 3)
        self.assertIs(track.templates[0], self.a)

    def test_missing_duplicate_cannot_alternate_with_current_owner(self):
        memory = self.memory(forget_after=1.)
        obs = self.observation(self.a)
        memory.update([[0, 0, 1], [.3, 0, 1]], 0,
                      observations=[obs, self.observation(self.b)])
        owner, duplicate = memory.tracks
        owner.id, duplicate.id = 11, 22
        duplicate.position = np.array([.006, 0, 1])
        duplicate.detection_index = None
        owner.gallery_frozen = duplicate.gallery_frozen = True
        for i in range(20):
            def fluctuating(first, second, parameters):
                # The stale gallery wins on alternate frames, despite there
                # being one continuous physical detection in the same place.
                strong = (first is self.b) if i % 2 else (first is self.a)
                return Similarity(.98 if strong else .65, .95, strong, 20 if strong else 0)
            with patch('rgbd.detection.persistence.compare', side_effect=fluctuating):
                tracks = memory.update([[0, 0, 1]], .1, observations=[obs])
            self.assertEqual([t.id for t in tracks if t.visible], [11])
        self.assertEqual([t.id for t in memory.tracks], [11])

    def test_missing_object_can_return_beside_an_active_neighbor(self):
        memory = self.memory()
        a, b = self.observation(self.a), self.observation(self.b)
        memory.update([[.3, 0, 1], [0, 0, 1]], 0, observations=[a, b])
        memory.update([[0, 0, 1]], .1, observations=[b])
        tracks = memory.update([[.02, 0, 1], [0, 0, 1]], .1, observations=[a, b])
        self.assertEqual([(t.id, t.detection_index) for t in tracks], [(0, 0), (1, 1)])


if __name__ == '__main__':
    unittest.main()
