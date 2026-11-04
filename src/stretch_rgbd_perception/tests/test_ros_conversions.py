"""Adapter geometry/wire contracts without requiring ROS on the laptop."""
import sys
from pathlib import Path
from types import SimpleNamespace as NS
import unittest
import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from rgbd_perception.conversions import image_array, frame_from_messages, preview_to_native, select_mask


def image(data, width, height, encoding, step, bigendian=False):
    return NS(data=data, width=width, height=height, encoding=encoding, step=step,
              is_bigendian=bigendian, header=NS(stamp=NS(sec=3, nanosec=42)))


class ROSConversionTests(unittest.TestCase):
    def test_padded_bgr_rows_and_big_endian_depth(self):
        color = image(bytes([1,2,3,4,5,6,99,99]), 2, 1, 'bgr8', 8)
        depth = image(np.array([1000,2500,0], dtype='>u2').tobytes(), 2, 1, '16UC1', 6, True)
        frame = frame_from_messages(color, depth, 0)
        np.testing.assert_array_equal(frame.color, [[[3,2,1],[6,5,4]]])
        np.testing.assert_allclose(frame.depth, [[1.,2.5]])
        self.assertEqual(frame.timestamp_ns, 3000000042)

    def test_float_depth_preserves_meters_nan_and_zero(self):
        depth = image(np.array([1.,np.nan,0], dtype='<f4').tobytes(), 3, 1, '32FC1', 12)
        np.testing.assert_allclose(image_array(depth), [[1.,np.nan,0]], equal_nan=True)

    def test_preview_clicks_invert_all_image_rotations(self):
        native = np.arange(24).reshape(4,6)
        for k in range(4):
            rotated = np.rot90(native, -k)
            for v,u in np.ndindex(rotated.shape):
                x,y = preview_to_native(u,v,6,4,k)
                self.assertEqual(native[y,x],rotated[v,u])

    def test_prompt_selects_only_owner_and_rejects_background_or_invalid(self):
        labels = np.full((5,6),-1)
        labels[1:3,1:3]=0;labels[1:3,4:6]=1
        mask,reason=select_mask(labels,(1,1))
        self.assertFalse(reason)
        np.testing.assert_array_equal(mask,labels==0)
        for click in ((0,0),(-1,0),(6,0),(float('nan'),1)):
            mask,reason=select_mask(labels,click)
            self.assertIsNone(mask);self.assertTrue(reason)


if __name__ == '__main__':
    unittest.main()
