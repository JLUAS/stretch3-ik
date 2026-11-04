"""Read uncompressed RealSense Viewer .db3 recordings without ROS installed.

Usage: with RealSenseReader(path, rotations=1) as source:
           frame = source[0]

Public arrays use RGB uint8 and depth float32 meters (zero means invalid).
Calibration uses optical axes: X right, Y down, Z forward. The 4x4 transform
maps depth-camera points to color-camera points, with translation in meters.
Images and calibration share the requested orientation. Only timestamps are
indexed; image payloads are loaded on demand. Unmatched depth frames are skipped.
"""

from bisect import bisect_left
from pathlib import Path

import numpy as np
from rosbags.highlevel import AnyReader
from rosbags.typesys import Stores, get_typestore
from rgbd.types import Camera, Calibration, Frame  # Re-export the existing reader API.
from rgbd.types import rotate_camera_clockwise as _rotate_camera_clockwise


class RealSenseReader:
    """Indexed RGB-D source for one RealSense device, with fixed calibration.

    Each depth frame is paired to the nearest color message using bag timestamps.
    Color messages may be reused. max_pair_delta_ms limits their separation.
    Calibration is read once; changes in stream dimensions are rejected.
    """

    def __init__(self, path, *, rotations=0, max_pair_delta_ms=50):
        if not isinstance(rotations, int) or rotations not in range(4):
            raise ValueError("rotations must be an integer from 0 to 3 (clockwise quarter-turns)")
        if not np.isfinite(max_pair_delta_ms) or max_pair_delta_ms < 0:
            raise ValueError("max_pair_delta_ms must be finite and nonnegative")
        self.rotations = rotations
        self._bag = AnyReader(
            [Path(path)], default_typestore=get_typestore(Stores.ROS2_HUMBLE)
        )
        self._bag.open()
        try:
            self._color = self._connection("/Color_0/image/data")
            self._depth = self._connection("/Depth_0/image/data")
            color_root = self._color.topic.removesuffix("/image/data")
            depth_root = self._depth.topic.removesuffix("/image/data")
            if color_root.split("/sensor_")[0] != depth_root.split("/sensor_")[0]:
                raise ValueError("Color and depth must belong to the same device")
            color = self._camera(color_root)
            depth = self._camera(depth_root)
            transform = self._transform(color_root) @ np.linalg.inv(
                self._transform(depth_root)
            )
            sensor_root = depth_root.rsplit("/", 1)[0]
            self._depth_scale = float(self._text(sensor_root + "/option/Depth_Units/value"))
            if not np.isfinite(self._depth_scale) or self._depth_scale <= 0:
                raise ValueError("Invalid recorded depth scale")
            self._native_shapes = (color.height, color.width), (depth.height, depth.width)
            for _ in range(rotations):
                color, depth = _rotate_camera_clockwise(color), _rotate_camera_clockwise(depth)
                axes = np.eye(4)
                axes[:3, :3] = ((0, -1, 0), (1, 0, 0), (0, 0, 1))
                transform = axes @ transform @ axes.T
            self.calibration = Calibration(color, depth, transform)

            times = {self._color.id: [], self._depth.id: []}
            for connection, timestamp, _ in self._bag.messages(
                connections=[self._color, self._depth]
            ):
                times[connection.id].append(timestamp)
            color_times, depth_times = times[self._color.id], times[self._depth.id]
            if not color_times or not depth_times:
                raise ValueError("Recording must contain both color and depth images")
            if any(len(set(stream)) != len(stream) for stream in times.values()):
                raise ValueError("Duplicate image timestamps are not supported")
            self._pairs = []
            for timestamp in depth_times:
                position = bisect_left(color_times, timestamp)
                candidates = color_times[max(0, position - 1):position + 1]
                nearest = min(candidates, key=lambda value: abs(value - timestamp))
                if abs(nearest - timestamp) <= max_pair_delta_ms * 1e6:
                    self._pairs.append((nearest, timestamp))
            if not self._pairs:
                raise ValueError("No color/depth pairs within the timestamp tolerance")
        except Exception:
            self.close()
            raise

    def __len__(self):
        return len(self._pairs)

    def __iter__(self):
        return (self[index] for index in range(len(self)))

    def __getitem__(self, index):
        color_time, depth_time = self._pairs[index]
        color = _image(self._message(self._color, color_time), color=True)
        depth = _image(self._message(self._depth, depth_time), color=False)
        if (color.shape[:2], depth.shape) != self._native_shapes:
            raise ValueError("Image dimensions changed from recorded calibration")
        depth = depth.astype(np.float32) * self._depth_scale
        if self.rotations:
            color, depth = np.rot90(color, -self.rotations), np.rot90(depth, -self.rotations)
        return Frame(depth_time, np.ascontiguousarray(color), np.ascontiguousarray(depth))

    @property
    def fps(self):
        """Average paired depth-frame rate; None for a single frame."""
        if len(self) < 2:
            return None
        seconds = (self._pairs[-1][1] - self._pairs[0][1]) / 1e9
        return (len(self) - 1) / seconds

    def close(self):
        self._bag.close()

    def __enter__(self):
        return self

    def __exit__(self, *args):
        self.close()

    def _connection(self, suffix):
        matches = [c for c in self._bag.connections if c.topic.endswith(suffix)]
        if len(matches) != 1:
            raise ValueError(f"Expected one topic ending in {suffix!r}, found {len(matches)}")
        return matches[0]

    def _message(self, connection, timestamp=None):
        messages = self._bag.messages(
            connections=[connection], start=timestamp,
            stop=None if timestamp is None else timestamp + 1,
        )
        try:
            _, _, payload = next(messages)
        except StopIteration:
            raise ValueError(f"Missing message on {connection.topic}") from None
        finally:
            messages.close()
        return self._bag.deserialize(payload, connection.msgtype)

    def _text(self, topic):
        connection = self._connection(topic)
        if connection.msgtype != "std_msgs/msg/String":
            raise ValueError(f"Expected RealSense String message on {topic}")
        return self._message(connection).data

    def _camera(self, root):
        values = _fields(self._text(root + "/camera_info"))
        K = np.array([
            [float(values["fx"]), 0, float(values["ppx"])],
            [0, float(values["fy"]), float(values["ppy"])],
            [0, 0, 1],
        ])
        return Camera(int(values["width"]), int(values["height"]), K,
                      np.fromstring(values["coeffs"], sep=","), values["model"])

    def _transform(self, root):
        values = _fields(self._text(root + "/tf/ref_0"))
        transform = np.eye(4)
        # SDK rs2_extrinsics.rotation is serialized in column-major order.
        transform[:3, :3] = np.fromstring(values["rotation"], sep=",").reshape(3, 3, order="F")
        transform[:3, 3] = np.fromstring(values["translation"], sep=",")
        return transform


def _fields(text):
    return dict(item.split("=", 1) for item in text.split(";") if "=" in item)


def _image(message, *, color):
    allowed = {"rgb8", "bgr8"} if color else {"mono16", "16uc1"}
    encoding = message.encoding.lower()
    if encoding not in allowed:
        raise ValueError(f"Unsupported RealSense image encoding: {message.encoding}")
    dtype = np.dtype("u1" if color else (">u2" if message.is_bigendian else "<u2"))
    channels = 3 if color else 1
    # Respect row padding instead of assuming every image is tightly packed.
    rows = np.frombuffer(message.data, dtype=dtype).reshape(message.height, message.step // dtype.itemsize)
    pixels = rows[:, :message.width * channels]
    image = pixels.reshape(message.height, message.width, 3) if color else pixels
    return image[:, :, ::-1].copy() if encoding == "bgr8" else image.copy()


if __name__ == "__main__":
    from rgbd.demos.viewer import main
    main()
