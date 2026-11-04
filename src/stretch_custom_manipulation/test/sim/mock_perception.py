#!/usr/bin/env python3
"""Synthetic head-camera perception for isolated tests (domain 20 only).

Renders what the head D435i would return for the mock world: the camera-
facing half of the target cylinder (with optional extrinsic bias and depth
noise) as the object cloud, plus tabletop and neighbor surfaces as the scene
cloud. `/mock/click` stands in for the operator click; prompts on
/perception/segment_prompt stand in for a re-segmentation request.
Publishes the legacy cloud topics and, when the message exists, the
normalized ObjectObservation.
"""
import json
import math
import os

import numpy as np
import rclpy
from geometry_msgs.msg import Point
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, PointCloud2
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Header, String
from std_srvs.srv import Trigger

try:
    from stretch_custom_manipulation.msg import ObjectObservation
except ImportError:  # baseline build
    ObjectObservation = None

CAMERA_IN_BASE = np.array([0.0, 0.0, 1.30])


def cylinder_surface(c, r, h, cam, step=0.004):
    pts = []
    for z in np.arange(c[2] - h / 2, c[2] + h / 2, step):
        for a in np.arange(0, 2 * math.pi, step / r):
            n = np.array([math.cos(a), math.sin(a), 0.0])
            p = np.array(c) + r * n
            p[2] = z
            if n @ (cam - p) > 0:
                pts.append(p)
    top = c[2] + h / 2
    if cam[2] > top:
        for x in np.arange(-r, r, step):
            for y in np.arange(-r, r, step):
                if x * x + y * y < r * r:
                    pts.append([c[0] + x, c[1] + y, top])
    return np.array(pts)


class MockPerception(Node):
    def __init__(self):
        super().__init__('mock_perception')
        if os.environ.get('ROS_DOMAIN_ID') != '20':
            raise RuntimeError('mock perception is restricted to domain 20')
        self.truth = None
        self.world = {}
        self.rng = np.random.default_rng(0)
        self.count = 0
        self.create_subscription(String, '/mock/truth', lambda m: setattr(self, 'truth', json.loads(m.data)), 10)
        self.create_subscription(String, '/mock/world', self.on_world,
                                 QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.create_subscription(Point, '/perception/segment_prompt', lambda p: self.observe('prompt'), 1)
        self.create_service(Trigger, '/mock/click', self.on_click)
        self.cloud_pub = self.create_publisher(PointCloud2, '/perception/object_cloud', 1)
        self.scene_pub = self.create_publisher(PointCloud2, '/perception/filtered_points', qos_profile_sensor_data)
        self.obs_pub = (self.create_publisher(ObjectObservation, '/perception/object_observation', 1)
                        if ObjectObservation else None)
        self.info_pub = self.create_publisher(CameraInfo, '/camera/color/camera_info', 1)
        self.create_timer(0.2, self.publish_scene)

    def on_world(self, msg):
        self.world = json.loads(msg.data)
        self.rng = np.random.default_rng(self.world.get('seed', 0))

    def camera(self):
        bx, by, th = self.truth['base']
        c, s = math.cos(th), math.sin(th)
        return np.array([bx + c * CAMERA_IN_BASE[0] - s * CAMERA_IN_BASE[1],
                         by + s * CAMERA_IN_BASE[0] + c * CAMERA_IN_BASE[1], CAMERA_IN_BASE[2]])

    def scene_points(self):
        t = self.truth
        c = np.array(t['center'])
        floor = c[2] - t['height'] / 2
        xs = np.arange(c[0] - 0.25, c[0] + 0.25, 0.01)
        ys = np.arange(c[1] - 0.15, c[1] + 0.15, 0.01)
        table = np.array([[x, y, floor] for x in xs for y in ys])
        cam = self.camera()
        parts = [table]
        for n in t['neighbors']:
            parts.append(cylinder_surface(n['center'], n['radius'], n['height'], cam, 0.008))
        if not t['held']:
            parts.append(cylinder_surface(c, t['radius'], t['height'], cam, 0.008))
        return np.vstack(parts)

    def publish_scene(self):
        # Head D435i color intrinsics measured on the robot (GRASPING_PLAN.md).
        info = CameraInfo(width=424, height=240, k=[303.06, 0.0, 212.06, 0.0, 303.07, 122.79, 0.0, 0.0, 1.0])
        info.header.frame_id = 'camera_color_optical_frame'
        info.header.stamp = self.get_clock().now().to_msg()
        self.info_pub.publish(info)
        if self.truth is None:
            return
        header = Header(frame_id='odom', stamp=self.get_clock().now().to_msg())
        self.scene_pub.publish(point_cloud2.create_cloud_xyz32(header, self.scene_points().tolist()))

    def on_click(self, _, response):
        ok, why = self.observe('click')
        response.success, response.message = ok, why
        return response

    def observe(self, source):
        t = self.truth
        if t is None:
            return False, 'no truth yet'
        self.count += 1
        lost = source == 'prompt' and self.world.get('faults', {}).get('lost_on_reobserve')
        header = Header(frame_id='odom', stamp=self.get_clock().now().to_msg())
        bias = np.array(self.world.get('perception_bias', [0, 0, 0]), float)
        noise = self.world.get('depth_noise', 0.002)
        pts = cylinder_surface(t['center'], t['radius'], t['height'], self.camera())
        cam = self.camera()
        rays = pts - cam
        rays /= np.linalg.norm(rays, axis=1, keepdims=True)
        pts = pts + rays * self.rng.normal(0, noise, (len(pts), 1)) + bias
        oid = f'mock-{self.count}'
        if not lost:
            self.cloud_pub.publish(point_cloud2.create_cloud_xyz32(header, pts.tolist()))
        if self.obs_pub:
            o = ObjectObservation(header=header, object_id=oid, source='mock_perception',
                                  confidence=0.0 if lost else 0.95, valid=not lost,
                                  reason='target not found' if lost else '')
            if not lost:
                o.cloud = point_cloud2.create_cloud_xyz32(header, pts.tolist())
                lo, hi = pts.min(0), pts.max(0)
                o.centroid.x, o.centroid.y, o.centroid.z = ((lo + hi) / 2).tolist()
                o.size.x, o.size.y, o.size.z = (hi - lo).tolist()
            self.obs_pub.publish(o)
        return not lost, oid


def main():
    rclpy.init()
    rclpy.spin(MockPerception())


if __name__ == '__main__':
    main()
