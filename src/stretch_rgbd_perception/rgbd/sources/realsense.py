"""RealSense SDK source: latest synchronized RGB-D, calibrated like bag frames."""

import numpy as np
import pyrealsense2 as rs

from rgbd.types import Camera, Calibration, Frame, rotate_calibration


def camera_from_profile(profile):
    i = profile.as_video_stream_profile().get_intrinsics()
    K = np.array([[i.fx, 0, i.ppx], [0, i.fy, i.ppy], [0, 0, 1]])
    return Camera(i.width, i.height, K, np.array(i.coeffs),
                  str(i.model).removeprefix('distortion.').replace('_', ' '))


class RealSenseLive:
    def __init__(self, *, serial=None, rotations=1, width=848, height=480, fps=30):
        self.rotations = rotations
        devices = list(rs.context().query_devices())
        if serial is None:
            matches = [d for d in devices if 'D435' in d.get_info(rs.camera_info.name)]
            if len(matches) != 1:
                raise RuntimeError('Specify --serial: expected exactly one D435 camera')
            serial = matches[0].get_info(rs.camera_info.serial_number)
        self.pipeline = rs.pipeline()
        config = rs.config()
        config.enable_device(serial)
        config.enable_stream(rs.stream.depth, width, height, rs.format.z16, fps)
        config.enable_stream(rs.stream.color, width, height, rs.format.rgb8, fps)
        self.queue = rs.frame_queue(1)
        profile = self.pipeline.start(config, self.queue)
        try:
            depth, color = profile.get_stream(rs.stream.depth), profile.get_stream(rs.stream.color)
            extrinsics = depth.get_extrinsics_to(color)
            transform = np.eye(4)
            transform[:3, :3] = np.array(extrinsics.rotation).reshape(3, 3, order='F')
            transform[:3, 3] = extrinsics.translation
            self.calibration = rotate_calibration(Calibration(
                camera_from_profile(color), camera_from_profile(depth), transform), rotations)
            self.depth_scale = profile.get_device().first_depth_sensor().get_depth_scale()
            self.name = profile.get_device().get_info(rs.camera_info.name)
            self.serial = serial
        except Exception:
            self.pipeline.stop()
            raise
        self.previous = None
        self.skipped = 0
        self.pair_delta_ms = 0

    def read(self):
        while True:
            frames = self.queue.wait_for_frame(5000).as_frameset()
            depth, color = frames.get_depth_frame(), frames.get_color_frame()
            if not depth or not color:
                continue
            self.pair_delta_ms = abs(depth.get_timestamp()-color.get_timestamp())
            timestamp = round(depth.get_timestamp()*1e6)
            if self.pair_delta_ms > 50 or (self.previous is not None and timestamp <= self.previous):
                continue
            number = depth.get_frame_number()
            self.skipped = 0 if self.previous is None else max(0, number-self.number-1)
            self.previous, self.number = timestamp, number
            rgb = np.asanyarray(color.get_data()).copy()
            meters = np.asanyarray(depth.get_data()).astype(np.float32)*self.depth_scale
            return Frame(timestamp, np.ascontiguousarray(np.rot90(rgb, -self.rotations)),
                         np.ascontiguousarray(np.rot90(meters, -self.rotations)))

    def close(self):
        self.pipeline.stop()

    def __iter__(self):
        while True:
            yield self.read()

    def __enter__(self):
        return self

    def __exit__(self, *args):
        self.close()
