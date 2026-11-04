import unittest
from unittest.mock import Mock

import numpy as np

from rgbd.classification.memory import ClassificationMemory
from rgbd.classification.ocr import OCRReader, Parameters, Reading, original_pixels
from rgbd.classification.matcher import ProductMatcher


def reading(text, confidence=.95):
    return Reading(0, text, confidence, (), 0)


class OCRTests(unittest.TestCase):
    def test_polygons_map_back_from_every_rotated_scaled_crop(self):
        original = np.array([[2., 3.], [15, 3], [15, 8], [2, 8]])
        width, height = 20, 12
        for k in range(4):
            x, y = original.T
            rotated = (original, np.column_stack((y, width-1-x)),
                       np.column_stack((width-1-x, height-1-y)), np.column_stack((height-1-y, x)))[k]
            shape = (height, width) if k % 2 == 0 else (width, height)
            enlarged = (rotated+.5)*2-.5
            returned = original_pixels(enlarged, k, width, height, shape, (shape[0]*2, shape[1]*2), (30, 40))
            np.testing.assert_allclose(returned, original+[30, 40])

    def test_two_detection_axes_one_recognition_batch_and_live_confidence(self):
        reader = object.__new__(OCRReader)
        reader.parameters = Parameters()
        reader.reader = Mock()
        reader.reader.text_det.return_value = (np.array([[[0, 0], [7, 0], [7, 5], [0, 5]]]), None)
        reader.reader.get_crop_img_list.return_value = [np.zeros((6, 8, 3), np.uint8)]
        reader.reader.text_rec.return_value = ([('TEST', .8)]*4, None)
        image = np.zeros((30, 40, 3), np.uint8)
        result = reader.read(image, [(10, 5, 30, 25)])
        self.assertEqual(len(result), 4)
        self.assertEqual(reader.reader.text_det.call_count, 2)
        self.assertEqual(reader.reader.text_rec.call_count, 1)
        reader.parameters.min_confidence = .9
        self.assertEqual(reader.read(image, [(10, 5, 30, 25)]), ())


class ClassificationTests(unittest.TestCase):
    def setUp(self):
        self.matcher = ProductMatcher()
        self.memory = ClassificationMemory()

    def identify(self, *texts):
        return self.matcher.identify([reading(text) for text in texts])

    def test_dictionary_alias_fuzzy_and_confidence_threshold(self):
        self.assertEqual(self.matcher.match('MELATONIN').label, 'Melatonina')
        self.assertTrue(self.matcher.match('Sepramycina', .9).accepted)
        self.assertFalse(self.matcher.match('Colgate', .4).accepted)
        self.assertIsNone(self.matcher.match('co'))
        self.matcher.parameters.min_ocr_confidence = .3
        self.assertTrue(self.matcher.match('Colgate', .4).accepted)

    def test_better_different_product_overwrites_remembered_identity(self):
        first = self.memory.update(7, self.identify('Melatonima'), 10)
        self.assertEqual(first.label, 'Melatonina')
        second = self.memory.update(7, self.identify('Colgate'), 20)
        self.assertEqual(second.label, 'Colgate')
        self.assertEqual(second.timestamp_ns, 20)
        self.assertIn(7, self.memory.conflicts)

    def test_ambiguous_unreadable_and_weaker_readings_keep_best(self):
        best = self.memory.update(7, self.identify('Colgate'), 10)
        for observation in (self.identify(''), self.identify('Colgate', 'Melatonina'), self.identify('Melatonima')):
            self.assertIs(self.memory.update(7, observation, 20), best)
        self.memory.prune([])
        self.assertFalse(self.memory.labels)


if __name__ == '__main__':
    unittest.main()
