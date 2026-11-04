"""Synchronous crop OCR. Model execution only; no IDs, memory or background jobs."""

from dataclasses import dataclass
from time import perf_counter

import cv2
import numpy as np


@dataclass
class Parameters:
    scale: float = 2.
    min_confidence: float = .3
    rotations: tuple = (0, 1, 2, 3)


@dataclass(frozen=True)
class Reading:
    region: int
    text: str
    confidence: float
    polygon: tuple  # Full-resolution source RGB coordinates, never current-frame coordinates.
    rotation: int  # Tested direction in degrees, not measured text orientation.


def original_pixels(polygon, rotation, width, height, rotated_shape, enlarged_shape, origin):
    points = (np.asarray(polygon, float)+.5)/np.array(
        [enlarged_shape[1]/rotated_shape[1], enlarged_shape[0]/rotated_shape[0]])-.5
    x, y = points.T
    if rotation == 1:
        points = np.column_stack((width-1-y, x))
    elif rotation == 2:
        points = np.column_stack((width-1-x, height-1-y))
    elif rotation == 3:
        points = np.column_stack((y, height-1-x))
    return tuple(tuple(float(v) for v in point) for point in points+origin)


class OCRReader:
    """RapidOCR 1.x: detect two perpendicular axes, recognize both directions.

    Perspective-rectified text lines handle tilt; quarter turns cover vertical
    and upside-down printing. Reuse one reader. threads is initialization-only;
    parameters can change between read() calls. No worker/thread is owned here.
    """

    def __init__(self, parameters=None, *, threads=2):
        from rapidocr_onnxruntime import RapidOCR
        self.parameters = Parameters() if parameters is None else parameters
        self.reader = RapidOCR(text_score=.3, intra_op_num_threads=threads,
                               inter_op_num_threads=1, det_limit_type='max')
        self.last_ms = 0.

    def read(self, rgb, boxes):
        """Exclusive RGB boxes -> Reading tuple; empty boxes produce no readings."""
        p = self.parameters
        if p.scale <= 0 or not 0 <= p.min_confidence <= 1 or any(k not in range(4) for k in p.rotations):
            raise ValueError('Invalid OCR scale/confidence/rotations')
        started = perf_counter()
        lines, targets = [], []
        for region, (x0, y0, x1, y1) in enumerate(boxes):
            if not (0 <= x0 < x1 <= rgb.shape[1] and 0 <= y0 < y1 <= rgb.shape[0]):
                raise ValueError('Crop must lie inside the RGB image')
            crop = cv2.cvtColor(rgb[y0:y1, x0:x1], cv2.COLOR_RGB2BGR)
            height, width = crop.shape[:2]
            done = set()
            for k in p.rotations:
                if k % 2 in done:
                    continue
                done.add(k % 2)
                rotated = np.ascontiguousarray(np.rot90(crop, k))
                enlarged = cv2.resize(rotated, None, fx=p.scale, fy=p.scale, interpolation=cv2.INTER_CUBIC)
                polygons, _ = self.reader.text_det(enlarged)
                if polygons is None or not len(polygons):
                    continue
                for polygon, line in zip(polygons, self.reader.get_crop_img_list(enlarged, polygons)):
                    source = original_pixels(polygon, k, width, height, rotated.shape, enlarged.shape, (x0, y0))
                    for direction in p.rotations:
                        if direction % 2 == k % 2:
                            lines.append(line if direction == k else np.ascontiguousarray(np.rot90(line, 2)))
                            targets.append((region, source, direction*90))
        readings = []
        if lines:
            recognized, _ = self.reader.text_rec(lines)
            for (region, polygon, rotation), (text, confidence) in zip(targets, recognized):
                if confidence >= p.min_confidence:
                    readings.append(Reading(region, text, float(confidence), polygon, rotation))
        self.last_ms = (perf_counter()-started)*1000
        return tuple(readings)


if __name__ == '__main__':
    from rgbd.demos.detection import main
    main('ocr')
