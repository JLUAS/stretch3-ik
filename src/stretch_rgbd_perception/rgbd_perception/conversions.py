"""ROS-shaped image/calibration conversions, testable without importing ROS."""
import numpy as np
from scipy.spatial.transform import Rotation

from rgbd.types import Camera, Frame
from rgbd.geometry import rgb_ownership


def stamp_ns(message):
    return message.header.stamp.sec * 1_000_000_000 + message.header.stamp.nanosec


def image_array(message):
    formats = {'rgb8': ('u1', 3), 'bgr8': ('u1', 3), 'mono8': ('u1', 1),
               '16UC1': ('u2', 1), '32FC1': ('f4', 1)}
    if message.encoding not in formats:
        raise ValueError(f'unsupported image encoding: {message.encoding}')
    kind, channels = formats[message.encoding]
    dtype = np.dtype(('>' if message.is_bigendian else '<') + kind)
    row_bytes = message.width * channels * dtype.itemsize
    if message.step < row_bytes or len(message.data) < message.height * message.step:
        raise ValueError('invalid image step or data length')
    rows = np.frombuffer(message.data, np.uint8).reshape(message.height, message.step)
    array = rows[:, :row_bytes].copy().view(dtype).reshape(message.height, message.width, channels)
    if channels == 1:
        array = array[:, :, 0]
    if message.encoding == 'bgr8':
        array = array[:, :, ::-1]
    return np.ascontiguousarray(array)


def frame_from_messages(color, depth, rotations):
    if color.encoding not in ('rgb8', 'bgr8') or depth.encoding not in ('16UC1', '32FC1'):
        raise ValueError('expected RGB/BGR color and 16UC1/32FC1 depth')
    rgb = image_array(color)
    meters = image_array(depth).astype(np.float32)
    if depth.encoding == '16UC1':
        meters *= .001  # RealSense ROS depth is millimeters.
    return Frame(stamp_ns(color), np.ascontiguousarray(np.rot90(rgb, -rotations)),
                 np.ascontiguousarray(np.rot90(meters, -rotations)))


def camera_from_info(info):
    return Camera(info.width, info.height, np.array(info.k).reshape(3, 3),
                  np.array(info.d), info.distortion_model)


def transform_matrix(transform):
    t, q = transform.translation, transform.rotation
    matrix = np.eye(4)
    matrix[:3, :3] = Rotation.from_quat([q.x, q.y, q.z, q.w]).as_matrix()
    matrix[:3, 3] = (t.x, t.y, t.z)
    return matrix


def optical_rotation(rotations):
    matrix = np.eye(4)
    matrix[:3, :3] = np.linalg.matrix_power(np.array([[0., -1, 0], [1, 0, 0], [0, 0, 1]]), rotations)
    return matrix


def native_labels(frame, segmentation, calibration, rotations):
    labels = rgb_ownership(frame, segmentation.detections, calibration, stride=1, scale=1.)
    # Never convex-fill a grasp mask or overwrite a pixel owned by another
    # object/background depth surface.
    # rgb_ownership uses -1 for both unknown and measured background, so leave
    # these pixels untouched rather than risk adding background to a grasp.
    return np.ascontiguousarray(np.rot90(labels, rotations))


def select_mask(labels, click):
    u, v = click
    if not np.isfinite([u, v]).all():
        return None, 'non-finite prompt coordinates'
    u, v = int(round(u)), int(round(v))
    if not (0 <= u < labels.shape[1] and 0 <= v < labels.shape[0]):
        return None, 'prompt outside the color image'
    owner = labels[v, u]
    if owner < 0:
        return None, 'no depth-segmented object at the prompt'
    mask = labels == owner
    if mask.mean() >= .4:
        return None, 'object mask covers too much of the image'
    return mask, ''


def preview_to_native(u, v, width, height, rotations):
    # Invert clockwise np.rot90 pixel mapping without changing optical geometry.
    return ((u, v), (v, height-1-u), (width-1-u, height-1-v),
            (width-1-v, u))[rotations % 4]
