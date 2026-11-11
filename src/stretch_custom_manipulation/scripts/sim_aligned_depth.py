#!/usr/bin/env python3
"""MuJoCo sim only: publish /camera/aligned_depth_to_color/* like the RealSense driver.

The sim publishes float depth (metres) from a depth camera 1.5 cm off the color
camera. object_localizer expects 16UC1 millimetres registered to the color
image, so each depth pixel is back-projected, moved into the color optical frame
and re-projected (nearest surface wins).
"""
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import CameraInfo, Image
from tf2_ros import Buffer, TransformListener


def quat_matrix(q, t):
    x, y, z, w = q.x, q.y, q.z, q.w
    m = np.eye(4)
    m[:3, :3] = [[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                 [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                 [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]]
    m[:3, 3] = [t.x, t.y, t.z]
    return m


def register(depth_m, k_depth, k_color, color_from_depth, shape):
    """Depth image (m) -> depth in the color image (mm, uint16, 0 = no data)."""
    h, w = depth_m.shape
    v, u = np.mgrid[0:h, 0:w]
    z = depth_m.ravel()
    ok = np.isfinite(z) & (z > 0)
    u, v, z = u.ravel()[ok], v.ravel()[ok], z[ok]
    p = np.stack([(u - k_depth[0, 2]) * z / k_depth[0, 0], (v - k_depth[1, 2]) * z / k_depth[1, 1], z])
    p = color_from_depth[:3, :3] @ p + color_from_depth[:3, 3:4]
    front = p[2] > 0
    p = p[:, front]
    cu = np.round(k_color[0, 0] * p[0] / p[2] + k_color[0, 2]).astype(int)
    cv = np.round(k_color[1, 1] * p[1] / p[2] + k_color[1, 2]).astype(int)
    inside = (cu >= 0) & (cu < shape[1]) & (cv >= 0) & (cv < shape[0])
    cu, cv, mm = cu[inside], cv[inside], np.round(p[2][inside] * 1000).astype(np.int64)
    out = np.full(shape, np.iinfo(np.uint16).max, np.int64)
    np.minimum.at(out, (cv, cu), mm)  # nearest surface wins
    out[out == np.iinfo(np.uint16).max] = 0
    return out.clip(0, 65535).astype(np.uint16)


class Aligner(Node):
    def __init__(self):
        super().__init__('sim_aligned_depth')
        self.tf = Buffer()
        TransformListener(self.tf, self)
        self.k_depth = self.color_info = self.extrinsic = None
        sub = self.create_subscription
        sub(CameraInfo, '/camera/depth/camera_info',
            lambda m: setattr(self, 'k_depth', np.array(m.k).reshape(3, 3)), qos_profile_sensor_data)
        sub(CameraInfo, '/camera/color/camera_info', lambda m: setattr(self, 'color_info', m),
            qos_profile_sensor_data)
        sub(Image, '/camera/depth/image_rect_raw', self.on_depth, qos_profile_sensor_data)
        self.depth_pub = self.create_publisher(Image, '/camera/aligned_depth_to_color/image_raw', 5)
        self.info_pub = self.create_publisher(CameraInfo, '/camera/aligned_depth_to_color/camera_info', 5)

    def on_depth(self, msg):
        if self.k_depth is None or self.color_info is None or msg.encoding != '32FC1':
            return
        ci = self.color_info
        if self.extrinsic is None:  # fixed mount: look it up once
            try:
                t = self.tf.lookup_transform(ci.header.frame_id, msg.header.frame_id, Time()).transform
            except Exception:
                return
            self.extrinsic = quat_matrix(t.rotation, t.translation)
        d = np.frombuffer(msg.data, np.float32).reshape(msg.height, -1)[:, :msg.width]
        out = register(d, self.k_depth, np.array(ci.k).reshape(3, 3), self.extrinsic, (ci.height, ci.width))
        img = Image(header=msg.header, height=ci.height, width=ci.width, encoding='16UC1',
                    is_bigendian=0, step=ci.width * 2, data=out.tobytes())
        img.header.frame_id = ci.header.frame_id
        info = CameraInfo()
        info.header, info.height, info.width = img.header, ci.height, ci.width
        info.distortion_model, info.d, info.k, info.r, info.p = ci.distortion_model, ci.d, ci.k, ci.r, ci.p
        self.depth_pub.publish(img)
        self.info_pub.publish(info)


def main():
    rclpy.init()
    rclpy.spin(Aligner())


if __name__ == '__main__':
    main()
