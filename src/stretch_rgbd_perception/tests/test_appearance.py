import unittest

import cv2
import numpy as np

from rgbd.detection.appearance import RGBObjectAppearance, compare, describe
from rgbd.geometry import rgb_ownership
from rgbd.types import Calibration, Camera, Detection, Frame


def signature(seed, color=(240, 240, 240)):
    image = np.full((160, 100, 3), color, np.uint8)
    rng = np.random.default_rng(seed)
    for x, y in rng.integers([15, 20], [85, 140], (65, 2)):
        cv2.circle(image, (int(x), int(y)), int(rng.integers(1, 4)), (15, 15, 15), -1)
    cv2.putText(image, f'BOX {seed}', (12, 55), cv2.FONT_HERSHEY_SIMPLEX, .42, (0, 0, 0), 1)
    mask = np.zeros((160, 100), np.uint8)
    mask[10:150, 8:92] = 1
    return describe(image, mask)


class AppearanceTests(unittest.TestCase):
    def test_textured_white_boxes_are_distinguished(self):
        first, second = signature(2), signature(5)
        self.assertTrue(compare(first, first).reliable)
        self.assertGreater(compare(first, first).score-compare(first, second).score, .08)
        self.assertFalse(first.descriptors.flags.writeable)

    def test_background_texture_is_excluded_and_settings_are_live(self):
        camera = Camera(200, 160, np.array([[180., 0, 100], [0, 180, 80], [0, 0, 1]]), np.zeros(5), 'none')
        extractor = RGBObjectAppearance(Calibration(camera, camera, np.eye(4)))
        rgb = np.random.default_rng(4).integers(0, 256, (160, 200, 3), np.uint8)
        rgb[40:130, 70:130] = (200, 40, 20)
        mask = np.zeros((160, 200), bool)
        mask[40:130, 70:130] = True
        detections = [Detection(mask, np.zeros(3), np.array([.03, .055, .14]))]
        frame = Frame(0, rgb, np.ones((160, 200), np.float32))
        observation = extractor.observe(frame, detections)[0]
        self.assertIsNotNone(observation.signature)
        self.assertIsNone(observation.signature.descriptors)
        extractor.parameters.min_pixels = 999999
        self.assertIsNone(extractor.observe(frame, detections)[0].signature)

    def test_rgb_z_buffer_keeps_nearest_background_occluder(self):
        # Tiny RGB focal length makes depth samples collide in one RGB pixel.
        depth_camera = Camera(4, 2, np.array([[100., 0, 0], [0, 100, 0], [0, 0, 1.]]), np.zeros(5), 'none')
        rgb_camera = Camera(4, 2, np.array([[1., 0, 0], [0, 1, 0], [0, 0, 1.]]), np.zeros(5), 'none')
        depth = np.ones((2, 4), np.float32)
        depth[0, 0] = .5  # Foreground occluder is not an accepted object.
        mask = np.ones((2, 4), bool)
        mask[0, 0] = False
        frame = Frame(0, np.zeros((2, 4, 3), np.uint8), depth)
        ownership = rgb_ownership(frame, [Detection(mask, np.zeros(3), np.ones(3))],
                                  Calibration(rgb_camera, depth_camera, np.eye(4)), stride=1)
        self.assertEqual(ownership[0, 0], -1)


if __name__ == '__main__':
    unittest.main()
