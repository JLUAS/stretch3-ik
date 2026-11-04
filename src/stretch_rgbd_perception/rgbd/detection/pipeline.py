"""Source-independent detection: frame + supplied pose -> detections and IDs."""

from dataclasses import dataclass
from numbers import Integral
from time import perf_counter

from rgbd.detection.appearance import RGBObjectAppearance, Observation
from rgbd.detection.persistence import ObjectPersistor
from rgbd.detection.tabletop import TabletopSegmenter, Segmentation
from rgbd.types import Frame, Pose, Track


@dataclass
class Result:
    frame: Frame
    pose: Pose | None
    segmentation: Segmentation
    observations: tuple[Observation, ...]
    tracks: tuple[Track, ...]
    dt: float
    timings_ms: dict
    events: tuple

    @property
    def status(self):
        return self.segmentation.error or ('pose unavailable' if self.pose is None else self.segmentation.plane_status)


class DetectionPipeline:
    """Call serially with synchronized shared Frame/Pose inputs.

    Pose estimation/filtering, acquisition, OCR and GUI stay outside. Missing
    pose permits fresh depth segmentation but ages IDs without matching them.
    allow_reverse is for bag browsing only; memory ages by abs(capture dt).
    """

    def __init__(self, calibration, *, segmenter=None, extractor=None, persistor=None, allow_reverse=False):
        self.calibration = calibration
        self.segmenter = TabletopSegmenter(calibration) if segmenter is None else segmenter
        self.extractor = RGBObjectAppearance(calibration) if extractor is None else extractor
        self.persistor = ObjectPersistor(calibration, comparison_parameters=self.extractor.parameters) if persistor is None else persistor
        self.allow_reverse = allow_reverse
        self.previous_timestamp = None
        self.direction = 0

    def reset(self):
        self.segmenter.reset()
        self.persistor.reset()
        self.previous_timestamp, self.direction = None, 0

    def process(self, frame, pose=None):
        c = self.calibration
        if not isinstance(frame.timestamp_ns, Integral):
            raise ValueError('Capture timestamp must be integer nanoseconds')
        if frame.color.shape != (c.color.height, c.color.width, 3) or frame.color.dtype.name != 'uint8':
            raise ValueError('Expected calibrated RGB uint8 frame')
        if pose is not None and pose.timestamp_ns != frame.timestamp_ns:
            raise ValueError('Pose and RGB-D must share a capture timestamp')
        delta = 0 if self.previous_timestamp is None else frame.timestamp_ns-self.previous_timestamp
        if self.previous_timestamp is not None and (delta == 0 or (delta < 0 and not self.allow_reverse)):
            raise ValueError('Capture timestamps must increase; allow_reverse is for playback')
        direction = 0 if not delta else (1 if delta > 0 else -1)
        if direction and self.direction not in (0, direction):
            self.segmenter.reset()
        dt = abs(delta)/1e9
        started = perf_counter()
        segmentation = self.segmenter.segment(frame.depth, pose)
        stage = perf_counter()
        observations = self.extractor.observe(frame, segmentation.detections)
        appearance_ms = (perf_counter()-stage)*1000
        stage = perf_counter()
        tracks = self.persistor.update(segmentation.detections, pose, dt, observations)
        timings = dict(segmentation=segmentation.timings_ms['total'], appearance=appearance_ms,
                       persistence=(perf_counter()-stage)*1000, total=(perf_counter()-started)*1000)
        self.direction, self.previous_timestamp = direction, frame.timestamp_ns
        return Result(frame, pose, segmentation, observations, tracks, dt, timings, self.persistor.events)


if __name__ == '__main__':
    from rgbd.demos.detection import main
    main('detection')
