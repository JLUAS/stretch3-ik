#!/usr/bin/env python3
"""Segmentation + aligned depth -> ObjectObservation in odom (Open3D).

Keeps a short buffer of aligned depth frames, deprojects the masked pixels
of the frame closest in time to the mask, transforms them to odom with TF at
that stamp, drops points inside the gripper envelope, removes outliers and
keeps the largest cluster. Every segmentation gets an answer: a valid
observation, or an invalid one with the reason.
"""
import collections
from copy import deepcopy

import numpy as np
import open3d as o3d
import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.time import Time
from sensor_msgs.msg import CameraInfo, Image, PointCloud2
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Header
from tf2_ros import Buffer, TransformException, TransformListener
from visualization_msgs.msg import Marker

from stretch_perception_interfaces.msg import ObjectObservation

import grasp_planning as gp


def stamp_seconds(header):
    return header.stamp.sec + header.stamp.nanosec * 1e-9


class ObjectLocalizer(Node):
    def __init__(self):
        super().__init__('object_localizer')
        self.frame = self.declare_parameter('target_frame', 'odom').value
        self.max_skew = self.declare_parameter('max_stamp_skew', 0.1).value
        # Bounded history for delayed prompt masks; matching is by capture time.
        self.depths = collections.deque(maxlen=150)
        self.info = None
        self.tf = Buffer()
        TransformListener(self.tf, self)
        self.create_subscription(
            Image, '/camera/aligned_depth_to_color/image_raw', self.depths.append, 5)
        self.create_subscription(
            CameraInfo, '/camera/aligned_depth_to_color/camera_info',
            lambda m: setattr(self, 'info', m), 1)
        self.create_subscription(ObjectObservation, '/perception/segmentation', self.on_segmentation, 1)
        self.cloud_pub = self.create_publisher(PointCloud2, '/perception/object_cloud', 1)
        self.obs_pub = self.create_publisher(ObjectObservation, '/perception/object_observation', 1)
        self.marker_pub = self.create_publisher(Marker, '/perception/object_marker', 1)

    def on_segmentation(self, seg):
        obs = ObjectObservation(header=deepcopy(seg.header), object_id=seg.object_id, source=seg.source,
                                confidence=seg.confidence, mask=seg.mask)
        obs.header.frame_id = self.frame
        if not seg.valid:
            reason = seg.reason
        else:
            reason = self.localize(seg.mask, obs)
        obs.valid = not reason
        obs.reason = reason
        if reason:
            self.get_logger().error(f'{seg.object_id}: {reason}')
        self.obs_pub.publish(obs)

    def localize(self, mask_msg, obs):
        """Fills obs from the mask; returns the reason it failed, or ''."""
        if self.info is None or not self.depths:
            return 'no depth image or camera info yet'
        t = stamp_seconds(mask_msg.header)
        depth = min(self.depths, key=lambda d: abs(stamp_seconds(d.header) - t))
        if abs(stamp_seconds(depth.header) - t) > self.max_skew or \
                depth.encoding != '16UC1' or \
                (depth.width, depth.height) != (mask_msg.width, mask_msg.height):
            return 'no aligned depth frame matches the mask'
        mask = np.frombuffer(mask_msg.data, np.uint8).reshape(
            mask_msg.height, mask_msg.step)[:, :mask_msg.width]
        d = np.frombuffer(depth.data, np.uint16).reshape(
            depth.height, depth.step // 2)[:, :depth.width]
        k = self.info.k
        points = gp.mask_points(mask, d, k[0], k[4], k[2], k[5])
        if len(points) < 30:
            return f'only {len(points)} valid depth points on the object'
        try:
            tf = self.tf.lookup_transform(self.frame, depth.header.frame_id,
                                          Time.from_msg(depth.header.stamp),
                                          Duration(seconds=0.3))
        except TransformException as e:
            return f'TF {self.frame} <- camera unavailable: {e}'
        points = gp.apply(gp.matrix_from_transform(tf.transform), points)
        # The gripper can be in view (pre-grasp, after a miss): drop points in
        # its envelope. A sphere around the grasp center also ate the near
        # face of an object 10 cm ahead of the fingers.
        try:
            at = self.tf.lookup_transform(self.frame, 'link_grasp_center',
                                          Time.from_msg(depth.header.stamp),
                                          Duration(seconds=0.3))
            points = gp.outside_gripper(points, gp.matrix_from_transform(at.transform))
        except TransformException as e:
            return f'TF {self.frame} <- link_grasp_center unavailable: {e}'
        if len(points) < 30:
            return 'object points are all inside the gripper envelope'
        cloud = o3d.geometry.PointCloud(o3d.utility.Vector3dVector(points))
        cloud = cloud.voxel_down_sample(0.005)
        cloud, _ = cloud.remove_statistical_outlier(nb_neighbors=20, std_ratio=2.0)
        labels = np.asarray(cloud.cluster_dbscan(eps=0.02, min_points=10))
        if labels.size == 0 or labels.max() < 0:
            return 'object points did not form a cluster'
        largest = np.argmax(np.bincount(labels[labels >= 0]))
        points = np.asarray(cloud.points)[labels == largest]
        header = Header(stamp=depth.header.stamp, frame_id=self.frame)
        obs.header.stamp = depth.header.stamp
        obs.cloud = point_cloud2.create_cloud_xyz32(header, points.tolist())
        self.cloud_pub.publish(obs.cloud)
        lo, hi = points.min(0), points.max(0)
        obs.centroid.x, obs.centroid.y, obs.centroid.z = ((lo + hi) / 2).tolist()
        obs.size.x, obs.size.y, obs.size.z = (hi - lo).tolist()
        marker = Marker(header=header, ns='object', id=0, type=Marker.CUBE,
                        action=Marker.ADD)
        marker.pose.position.x, marker.pose.position.y, marker.pose.position.z = \
            ((lo + hi) / 2).tolist()
        marker.pose.orientation.w = 1.0
        marker.scale.x, marker.scale.y, marker.scale.z = (hi - lo).tolist()
        marker.color.g = marker.color.a = 0.6
        self.marker_pub.publish(marker)
        self.get_logger().info(
            f'Object: {len(points)} points, size {np.round(hi - lo, 3).tolist()} m')
        return ''


def main():
    rclpy.init()
    node = ObjectLocalizer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
