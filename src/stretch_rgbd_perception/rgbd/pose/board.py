"""RGB GridBoard -> raw pose in the shared optical-camera convention.

Run this module to browse a DB3 recording. No source, GUI, temporal filtering,
or object memory is owned by BoardPoseEstimator.
"""

from dataclasses import astuple, dataclass, field
from time import perf_counter

import cv2
import numpy as np

from rgbd.types import Camera, Frame, Pose, rotate_camera_clockwise


@dataclass
class Board:
    """Printed letter GridBoard; dimensions before printer scaling, in meters.

    Origin: board center. X right, Y up, Z out of the printed face, as viewed
    with the board's marker layout upright. Independent of image orientation.
    Changing any field rebuilds board geometry on the next estimate().
    """

    size: tuple[int, int] = (3, 4)
    marker_length: float = .025
    marker_separation: float = .0375
    scale: float = .96
    dictionary: int = cv2.aruco.DICT_4X4_50
    first_id: int = 0
    reference_frame: str = "board"


@dataclass
class Parameters:
    min_markers: int = 2
    max_reprojection_error: float | None = 3.  # RMS pixels; None disables.


@dataclass
class Observation:
    ids: np.ndarray = field(default_factory=lambda: np.empty((0, 1), np.int32))
    corners: tuple = ()  # Always original input-image coordinates.
    reprojection_error: float | None = None
    processing_ms: float = 0.
    status: str = "not processed"


class BoardPoseEstimator:
    def __init__(self, camera: Camera, board=None, parameters=None):
        self.camera = camera
        self.board = Board() if board is None else board
        self.parameters = Parameters() if parameters is None else parameters
        # Full OpenCV parameter object is public and read on each estimate.
        self.detector_parameters = cv2.aruco.DetectorParameters()
        self.detector_parameters.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX
        self.observation = Observation()
        self._geometry_key = None

    def _configure(self):
        board = self.board
        if astuple(board) != self._geometry_key:
            if (min(board.size) < 1 or board.marker_length <= 0 or board.marker_separation < 0
                    or not np.isfinite(board.scale) or board.scale <= 0):
                raise ValueError("Require positive board size/marker length/scale and nonnegative separation")
            dictionary = cv2.aruco.getPredefinedDictionary(board.dictionary)
            ids = np.arange(board.first_id, board.first_id+np.prod(board.size), dtype=np.int32)
            self._grid = cv2.aruco.GridBoard(board.size, board.marker_length*board.scale,
                                           board.marker_separation*board.scale, dictionary, ids)
            dimensions = np.array(board.size)*board.marker_length + (np.array(board.size)-1)*board.marker_separation
            self._center = np.r_[dimensions*board.scale/2, 0.]
            self._detector = cv2.aruco.ArucoDetector(dictionary)
            self._geometry_key = astuple(board)
        self._detector.setDetectorParameters(self.detector_parameters)

    def estimate(self, frame: Frame) -> Pose | None:
        started = perf_counter()
        self.observation = Observation()
        try:
            return self._estimate(frame)
        finally:
            self.observation.processing_ms = (perf_counter()-started)*1000

    def _estimate(self, frame):
        camera, parameters = self.camera, self.parameters
        if frame.color.shape != (camera.height, camera.width, 3) or frame.color.dtype != np.uint8:
            raise ValueError("Expected RGB uint8 with calibrated image dimensions")
        if np.any(camera.D) and camera.distortion_model.lower() not in (
                "brown conrady", "modified brown conrady", "plumb_bob", "rational_polynomial"):
            raise ValueError("Nonzero distortion must use an OpenCV Brown model")
        if parameters.min_markers < 2:
            raise ValueError("Require at least two board markers")
        if parameters.max_reprojection_error is not None and (
                not np.isfinite(parameters.max_reprojection_error) or parameters.max_reprojection_error <= 0):
            raise ValueError("Reprojection limit must be positive or None")
        self._configure()
        gray = cv2.cvtColor(frame.color, cv2.COLOR_RGB2GRAY)
        corners, ids, _ = self._detector.detectMarkers(gray)
        valid = [] if ids is None else [i for i, marker in enumerate(ids.ravel()) if marker in self._grid.getIds()]
        self.observation.corners = tuple(corners[i] for i in valid)
        self.observation.ids = np.array([ids[i] for i in valid], dtype=np.int32).reshape(-1, 1)
        self.observation.status = "not enough board markers"
        if len(valid) < parameters.min_markers:
            return None
        objects, pixels = self._grid.matchImagePoints(self.observation.corners, self.observation.ids)
        # Center the board, then flip board Y/Z to retain the public Y-up convention.
        objects = (objects.reshape(-1, 3)-self._center)*[1, -1, -1]
        pixels = pixels.reshape(-1, 2)
        # Implicit 180-degree PnP rotation: rotate calibration as well as pixels.
        # Undo optical rotation before exposing a source-independent rigid pose.
        rotated = rotate_camera_clockwise(rotate_camera_clockwise(camera))
        pnp_pixels = np.array([camera.width-1, camera.height-1])-pixels
        success, rvec, tvec = cv2.solvePnP(objects, pnp_pixels, rotated.K, rotated.D,
                                         flags=cv2.SOLVEPNP_ITERATIVE)
        self.observation.status = "PnP failed"
        if not success:
            return None
        undo_rotation = np.diag([-1., -1., 1.])
        transform = np.eye(4)
        transform[:3, :3] = undo_rotation @ cv2.Rodrigues(rvec)[0]
        transform[:3, 3] = undo_rotation @ tvec.ravel()
        projected, _ = cv2.projectPoints(objects, cv2.Rodrigues(transform[:3, :3])[0],
                                         transform[:3, 3], camera.K, camera.D)
        rms = float(np.sqrt(np.mean(np.sum((projected.reshape(-1, 2)-pixels)**2, axis=1))))
        self.observation.reprojection_error = rms
        if not np.isfinite(transform).all() or np.any((objects @ transform[:3, :3].T+transform[:3, 3])[:, 2] <= 0):
            self.observation.status = "invalid pose"
            return None
        if parameters.max_reprojection_error is not None and rms > parameters.max_reprojection_error:
            self.observation.status = "reprojection error too high"
            return None
        self.observation.status = "pose OK"
        return Pose(frame.timestamp_ns, transform, self.board.reference_frame)

    def annotate(self, color_rgb, pose=None, *, axis_length=.05):
        """Return BGR diagnostic image; never modify the input RGB frame."""
        image = cv2.cvtColor(color_rgb, cv2.COLOR_RGB2BGR)
        observation = self.observation
        if len(observation.ids):
            cv2.aruco.drawDetectedMarkers(image, observation.corners, observation.ids)
        if pose is not None:
            matrix = pose.reference_to_camera
            cv2.drawFrameAxes(image, self.camera.K, self.camera.D,
                             cv2.Rodrigues(matrix[:3, :3])[0], matrix[:3, 3], axis_length)
        return image


if __name__ == "__main__":
    from rgbd.demos.board_pose import main
    main()
